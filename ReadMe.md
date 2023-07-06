Steps to reproduce the issue:

1. First install CyberRT standalone by :https://github.com/minhanghuang/CyberRT/tree/v7.0.0
2. Then build this project by: 
```
mkdir build && cd build
cmake ..
make -j4
```
3. Run the test by:
```
./asio_conflict_test
```