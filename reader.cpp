#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <cyber/init.h>
#include <cyber/cyber.h>

#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include "foxglove/PointCloud.pb.h"
#include "asio.hpp"
#include "base64.hpp"


void CallBack(const std::shared_ptr <foxglove::PointCloud> &msg) {
    LOG(INFO)  << "callback";

}

uint64_t NanoSecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
                            .count());
}
std::string SerializeFdSet(const google::protobuf::Descriptor *toplevel_descriptor) {
    google::protobuf::FileDescriptorSet fd_set;
    std::queue<const google::protobuf::FileDescriptor *> to_add;
    to_add.push(toplevel_descriptor->file());

    std::unordered_set<std::string> seen_dependencies;

    while (!to_add.empty()) {
        const google::protobuf::FileDescriptor *next = to_add.front();
        to_add.pop();
        next->CopyTo(fd_set.add_file());
        for (int i = 0; i < next->dependency_count(); ++i) {
            const auto &dep = next->dependency(i);
            if (seen_dependencies.find(dep->name()) == seen_dependencies.end()) {
                seen_dependencies.insert(dep->name());
                to_add.push(dep);
            }
        }
    }
    return fd_set.SerializeAsString();
}


bool StartFoxgloveWebsocket() {
        LOG(INFO) << "FoxgloveWebsocketSender init...";
        const auto logHandler =[](foxglove::WebSocketLogLevel, char const *msg) {
            std::cout << msg << std::endl;
        };
        foxglove::ServerOptions serverOptions;
        auto server = std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(
        "C++ Protobuf example server", logHandler, serverOptions);

        foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;
        hdlrs.subscribeHandler =[&](foxglove::ChannelId chanId, foxglove::ConnHandle) {
            std::cout << "first client subscribed to " << chanId << std::endl;
        };
        hdlrs.unsubscribeHandler =[&](foxglove::ChannelId chanId, foxglove::ConnHandle) {
            std::cout << "last client unsubscribed from " << chanId << std::endl;
        };
        server->setHandlers(std::move(hdlrs));
        server->start("0.0.0.0", 8765);

        const auto channelIds = server->addChannels({
            {
                .topic = "example_msg",
                .encoding = "protobuf",
                .schemaName = foxglove::PointCloud::descriptor()->full_name(),
                .schema = Base64Encode(
                        SerializeFdSet(foxglove::PointCloud::descriptor())),
            }
        });
        const auto chanId = channelIds.front();

        bool running = true;

        auto signals = asio::signal_set(server->getEndpoint().get_io_service(), SIGINT);

        signals.async_wait([&](std::error_code const &ec, int sig) {
            if (ec) {
                std::cerr << "signal error: " << ec.message() << std::endl;
                return;
            }
            std::cerr << "received signal " << sig << ", shutting down" << std::endl;
            running = false;
        });

        while (running) {
            const auto now = NanoSecondsSinceEpoch();
            foxglove::PointCloud msg;
            msg.mutable_timestamp()->CopyFrom(google::protobuf::util::TimeUtil::NanosecondsToTimestamp(now));
            // add x, y, z, i field
            std::string fieldNames[] = {"x", "y", "z", "i"};
            for (int i = 0; i < 3; i++) {
                auto *field = msg.add_fields();
                field->set_name(fieldNames[i]);
                field->set_type(foxglove::PackedElementField::FLOAT32);
                field->set_offset(i * 4);
            }
            msg.set_frame_id("lidar pointcloud");
            msg.set_point_stride(16);

            size_t point_cloud_size = 10000;
            std::vector<char> point_data(point_cloud_size);
            // random point cloud using stl
            std::generate(point_data.begin(), point_data.end(), []() {
                return static_cast<char>(std::rand() % 256);
            });
            msg.mutable_data()->assign(point_data.begin(), point_data.end());

            const auto serializedMsg = msg.SerializeAsString();
            server->broadcastMessage(chanId, now, reinterpret_cast<const uint8_t *>(serializedMsg.data()),
                                     serializedMsg.size());

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        server->removeChannels({ chanId });
        server->stop();

        return true;
}

int main() {
    // init cyber
    apollo::cyber::Init("foxglove_asio");
    auto node = apollo::cyber::CreateNode("foxglove_asio");
    auto listener = node->CreateReader<foxglove::PointCloud>(
            "PointCloud", CallBack);

    // start websocket
    StartFoxgloveWebsocket();
    apollo::cyber::WaitForShutdown();
    return 0;
}

