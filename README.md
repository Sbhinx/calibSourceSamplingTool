# 关于

本项目可实现对激光雷达和摄像头联合标定数据文件的采集

# 硬件

Windows PC (推荐Nvidia GPU)  

[Robosense激光雷达](https://www.robosense.cn/rslidar/RS-Helios)

USB 摄像头

# 开发环境(可选)

C++：VS 2022

# 部署

#### Step1  拉取仓库 

~~~cmd
https://github.com/Sbhinx/calibSourceSamplingTool.git
~~~

使用release可直接跳到Step4

#### Step2  环境配置   

关于Robosese激光雷达的配置，可参考[rs_driver]([rs_driver/README_CN.md at main · RoboSense-LiDAR/rs_driver](https://github.com/RoboSense-LiDAR/rs_driver/blob/main/README_CN.md))

本项目使用以下库：

[PCL]([PointCloudLibrary/pcl: Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl))：点云处理

[OpenCV]([opencv/opencv: Open Source Computer Vision Library](https://github.com/opencv/opencv)) ：图像处理

[rs_driver]([RoboSense-LiDAR/rs_driver: RoboSense LiDAR cross-platform driver kernel for advanced development](https://github.com/RoboSense-LiDAR/rs_driver))：Robosense激光雷达驱动

#### Step3  连接硬件

将电脑与激光雷达数据盒通过网线连接。 

将电源适配器与激光雷达数据盒连接。

并按照rs_driver中的教程确保雷达正确连接。

摄像头也确保正确连接。

#### Step4 用法(Usage)  

必要项

~~~c++
X:\calibSourceSamplingTool>calibSamplingTool.exe <camID> <width> <height>
/*
* camID-相机ID
* width-图像宽度
* height-图像高度
*/
~~~

可选项

~~~C++
X:\calibSourceSamplingTool>calibSamplingTool.exe -help
~~~
