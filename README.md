# 基于ARToolKit与ORB-SLAM的AR扩展跟踪功能实现

## 1. 项目简介
本项目通过对传统的AR引擎——ARToolKit及开源的视觉SLAM方案——ORB-SLAM进行研究，实现在ARToolKit引擎已有功能的基础上融合视觉SLAM技术增添扩展跟踪功能，以增强AR应用程序的鲁棒性和用户体验。

## 2. 项目编译及运行
### 2.1 准备工作
- 运行环境：Ubuntu16.04
- 编译环境：gcc, g++(C++11), Cmake（2.8以上）
- 相关库依赖：本程序是由ARToolKit工程与ORB-SLAM2工程结合形成的，所以本程序的库依赖为ARToolKit和ORB-SLAM2库依赖的并集，关于两者库依赖的添加，可参考[ARToolKit5-github](https://github.com/artoolkit/artoolkit5)以及[ORB-SLAM2-github](https://github.com/raulmur/ORB_SLAM2)
。若遇到问题，也可参考`编译过程中可能遇到的问题.txt`
### 2.2 程序编译
在Linux终端下，使用cd命令进入到`Extended-Tracking`根目录下，输入：
```
chmod +x build.sh
./build.sh
```
即可自动完成整个扩展跟踪程序的编译工作；其编译流程为：
1. 进入`Thirdparty/DBoW2`目录下编译ORB-SLAM2部分所需的`DBoW2`库。 
2. 进入`Thirdparty/g2o`目录下编译ORB-SLAM2部分所需的`g2o`库。
3. 进入`lib/SRC`目录下根据`makefile`文件编译ARToolKit部分所需的相关库。
4. 根据根目录下的`CmakeLists.txt`文件先编译`ORB-SLAM2`部分所需的共享库`libORB_SLAM2.so`，随后编译扩展跟踪程序示例。

编译完成后，将在`Thirdparty/DBoW2/lib`下得到`DBoW2`库，在`Thirdparty/g2o/lib`下得到`g2o`库，在`lib`下得到ARToolKit相关库和ORB-SLAM2共享库`libORB_SLAM2.so`，在`bin`下得到扩展跟踪程序示例的可执行文件`extended-tracking`。
### 2.3 程序运行
1. 将`相关参考材料`目录下的`pinball.jpg`图片文件打印到A4纸上作为程序的自然图像模板。
2. 打开命令行终端，输入以下代码并回车（指定ARToolKit调用的摄像头驱动类型）：

    ```
    export ARTOOLKIT5_VCONF="-device=LinuxV4L2"
    ```
3. 在`Extended-Tracking`目录下，进入到`bin`目录：

    ```
    cd bin
    ```
    运行程序：
    ```
    ./extended-tracking Vocabulary/ORBvoc.txt TUM1.yaml
    ```
    其中`Vocabulary/ORBvoc.txt`参数为ORB字典文件的路径，`TUM1.yaml`参数保存的是相机内参
和ORB-SLAM2程序所需的相关设置信息；这两个参数主要供程序的ORB-SLAM2部分初始化使
用。
4. 如果想让程序调用USB摄像头，则应使接入的USB摄像头的设备号为**video0**（ARToolKit使用
video0设备作为摄像头），可以使用如下命令查看当前的摄像头设备：

    ```
    ls /dev/v*
    ```
    将在终端显示如下信息：
    ![](http://static.zybuluo.com/LiTAOo/wn5t0m8ufphty6jor5a5hpuu/QQ%E5%9B%BE%E7%89%8720180706161706.png)
    如果是笔记本电脑，则Linux系统默认将内置的前置摄像头作为**video0**设备，在电脑正常运行情况下接入USB摄像头，此时的USB摄像头会成为**video1**设备。为了将USB摄像头设置成**video0**设备，可以在电脑关机状态下插入USB摄像头再开机，开机后USB摄像头会被自动设置为**video0**设备。
5. 程序运行效果截图如下：
    ![](https://litaooooo.github.io/page-examples/extended-tracking.png)
### 2.4 程序清理
在`Extended-Tracking`目录下，输入：
```
chmod +x clean.sh
./clean.sh
```
即可清理在编译过程中产生的所有中间文件、库文件以及可执行文件。
### 2.5 其它相关问题
- **如何制作并训练自定义的自然图像模板？**

    可参考`相关参考文件/ARToolKit for unity使用整理/ARToolkit for unity使用整理.pdf`文件中的相关内容，在得到`.fset`,`.fset3`,`.iset`文件后，将其放入`Extended-Tracking/bin/DataDFT`替换掉相应格式的文件，随后修改`bin/Data2/markers.dat`文件中的相关信息即可。
- **如何标定当前相机的内参信息？**

    可参考`相关参考文件/ARToolKit for unity使用整理/ARToolkit for unity使用整理.pdf`文件中的相关内容，在得到`camera_para.dat`文件后，将其放入`bin/Data2`替换掉同名文件，随后再根据标定结果修改`bin/TUM1.yaml`文件中的内参信息即可。

需要注意的是，以上过程所涉及的应用程序可以在**Windows**操作系统下运行`相关参考材料/ARToolKit各版本（官网下载）/ARToolkit5_windows/ARToolKit5/bin`目录下的相关程序来完成。

### 2.6 程序效果演示
请参考：[AR扩展跟踪demo功能演示](https://v.youku.com/v_show/id_XMzgyMTI5MDE2OA==.html)
