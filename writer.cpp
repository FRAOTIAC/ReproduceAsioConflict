//
// Created by Lucas on 7/7/2023.
//
#include <cyber/init.h>
#include <cyber/cyber.h>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include "foxglove/PointCloud.pb.h"


uint64_t NanoSecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
                            .count());
}
// generate foxglove::PointCloud
foxglove::PointCloud MsgGenerator() {
    const auto now = NanoSecondsSinceEpoch();
    foxglove::PointCloud msg;
    msg.mutable_timestamp()->CopyFrom(google::protobuf::util::TimeUtil::NanosecondsToTimestamp(now));
    // add x, y, z, i field
    std::string fieldNames[] = {"x", "y", "z", "i"};
    for (int i = 0; i < 3; i++) {
        auto *field = msg.add_fields();
        field->set_name(fieldNames[i]);
        field->set_type(foxglove::PackedElementField::FLOAT32);
        field->set_offset(i*4);
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

    return msg;
}


int main() {
    apollo::cyber::Init("writer");
    auto node = apollo::cyber::CreateNode("writer");
    auto writer = node->CreateWriter<foxglove::PointCloud>("/PointCloud");

    bool running = true;
    while(running) {
        auto msg = MsgGenerator();
        writer->Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    apollo::cyber::WaitForShutdown();
    return 0;
}