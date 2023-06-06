echo "==========="
cd build
rm -rf *
# 会覆盖掉 cmake lists中 option的定义
# cmake -DBUILD_ENABLE_DEBUG=ON ..
cmake ..
make -j