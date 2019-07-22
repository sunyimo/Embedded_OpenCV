说明：
1，基线为opencv2.4.9；
2，去除对zlib的依赖；
3，编译DSP库时，动态内存操作根据__hexagon__编译开关选择（malloc/free）或者（qurt_malloc/qurt_free）;
4，统一编译为静态库；

编译：

cd build/

# 编译Android和hexagon DSP库
make -j8

# 编译Android库
make opencv_arm

# 编译hexagon DSP库
make opencv_dsp

# Android和hexagon DSP库内的某一个module可单独编译，详见Makefile

安装：

make install #将lib和include拷贝到prefix目录下，prefix默认为build/install