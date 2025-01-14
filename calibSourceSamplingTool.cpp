#include <rs_driver/api/lidar_driver.hpp>
#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <direct.h> 



//#define ORDERLY_EXIT
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;


robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

class LidarService{

public:

    //构造/析构方法
    LidarService() = default;
    
    //共享变量
    std::shared_ptr<PointCloudMsg> sharedmsg;

    //一般函数
    void processCloud(){
        bool to_exit_process = false;
        while (!to_exit_process)
        {
            std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
            if (msg.get() == NULL)
            {
                continue;
            }

            // Well, it is time to process the point cloud msg, even it is time-consuming.
            //RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

            //写到共享变量sharedmsg
            sharedmsg = msg;

            free_cloud_queue.push(msg);
        
        }
    }

    void savePCD(std::shared_ptr<PointCloudMsg> rawPcd)
    {
        //增加一个使用PCL库编码msg的函数

        if (!rawPcd) {
            throw std::invalid_argument("rawPCD为空帧！");
        }

        // 创建 PCL 点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pclCloud->width = rawPcd->width;
        pclCloud->height = rawPcd->height;
        pclCloud->is_dense = rawPcd->is_dense;
        pclCloud->points.resize(rawPcd->points.size() / 3);

        // 将 PointCloudMsg 数据拷贝到 PCL 点云
        for (size_t i = 0; i < pclCloud->points.size(); ++i) {
            pclCloud->points[i].x = rawPcd->points[i].x;
            pclCloud->points[i].y = rawPcd->points[i].y;
            pclCloud->points[i].z = rawPcd->points[i].z;
        }

        // 获取保存路径和文件名
        std::string pcdFolder = getPCDFolderPath();
        std::string filePath = pcdFolder+ "\\" + getNextPCDFileName() + ".pcd";

        // 保存点云到 PCD 文件
        if (pcl::io::savePCDFileBinary(filePath, *pclCloud) < 0) {
            throw std::runtime_error("保存pcd文件失败!");
        }

        std::cout << "PCD file 保存至: " << filePath << std::endl;
    }
    
    //静态函数
    static std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
    {
        // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
        //       so please DO NOT do time-consuming task here.
        std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
        if (msg.get() != NULL)
        {
            return msg;
        }

        return std::make_shared<PointCloudMsg>();
    }

    static void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
    {
        // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
        //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
        stuffed_cloud_queue.push(msg);
    }

    static void exceptionCallback(const robosense::lidar::Error& code)
    {
        // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
        //       so please DO NOT do time-consuming task here.
        RS_WARNING << code.toString() << RS_REND;
    }

    // pcd文件夹路径
    // 查询或创建 pcd 文件夹
    static std::string getPCDFolderPath() {
        char exePath[MAX_PATH];
        // 获取当前可执行文件路径
        if (GetModuleFileNameA(NULL, exePath, MAX_PATH) == 0) {
            throw std::runtime_error("Failed to get executable path.");
        }

        // 获取可执行文件所在目录
        std::string exeDir = std::string(exePath).substr(0, std::string(exePath).find_last_of("\\/"));

        // 拼接 pcd 文件夹路径
        std::string pcdFolder = exeDir + "\\pcd";

        // 检查文件夹是否存在，如果不存在则创建
        struct stat info;
        if (stat(pcdFolder.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            if (_mkdir(pcdFolder.c_str()) != 0) {
                throw std::runtime_error("Failed to create directory: " + pcdFolder);
            }
        }

        return pcdFolder;
    }

    // 获取保存的文件名（按000, 001, 002等格式递增）
    static std::string getNextPCDFileName() {
        static int pcd_counter = 0;  // 静态变量，保证跨调用记住上次的编号
        std::stringstream ss;
        ss<< "pcd_" << std::setw(3) << std::setfill('0') << pcd_counter++;  // 格式化为三位数，填充0
        return ss.str();
    }


    
    
private:
    bool to_exit_process = false;
};

class CameraService {
    
public:

    std::shared_ptr<cv::Mat> sharedframe;

    std::shared_ptr< cv::VideoCapture> sharedCap;

    int cameraID;
    int width;
    int height;

    CameraService(int camID = 0, int w = 1, int h = 1) : cameraID(camID), width(w), height(h) { 
        sharedframe = std::make_shared<cv::Mat>(height, width, CV_8UC3); 
    }

    
    void saveImage(std::shared_ptr<cv::Mat> sharedframe){
        // 保存图像为JPG文件
        static int img_counter = 0;

        // 获取保存路径和文件名
        std::string pcdFolder = getIMGFolderPath();
        std::string filePath = pcdFolder + "\\" + getNextIMGFileName() + ".jpg";

        if (!cv::imwrite(filePath, *sharedframe)) {
            std::cerr << "保存jpg文件失败" << std::endl;
        }
        else {
            std::cout << "JPG file 保存到:" << filePath << std::endl;
        }
    }

    void processImage(cv::VideoCapture cap, std::shared_ptr<cv::Mat> &shareframe) {

        //用以不停采集图像数据的线程函数

        // 创建一个窗口
        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

        while (true) {
            cv::Mat frame;

            // 读取摄像头的一帧
            cap >> frame;

            // 检查帧是否为空
            if (frame.empty()) {
                std::cerr << "Error: Received an empty frame!" << std::endl;
                break;
            }

            //类似于 指针操作
            shareframe = std::make_shared<cv::Mat>(frame);

            // 显示图像
            cv::imshow("Camera", frame);

            // 按下 'q' 键退出
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        // 释放摄像头
        cap.release();

        // 销毁窗口
        cv::destroyAllWindows();

    }

    bool initCamera() {
        //打开摄像头,并打开图像处理线程
        cv::VideoCapture cap(cameraID);
        // 检查摄像头是否打开成功
        if (!cap.isOpened()) {
            std::cerr << "Error: Cannot open the camera!" << std::endl;
            return false;
        }

        //int width = 1366;
        //int height = 768;
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);   // 设置宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 设置高度

        // 获取并输出设置的分辨率
        int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        std::cout << "分辨率设置为: " << actualWidth << "x" << actualHeight << std::endl;


        //给sharedCap分配地址
        sharedCap = std::make_shared<cv::VideoCapture>(cap);

        return true;
    }

    static std::string getIMGFolderPath() {
        char exePath[MAX_PATH];
        // 获取当前可执行文件路径
        if (GetModuleFileNameA(NULL, exePath, MAX_PATH) == 0) {
            throw std::runtime_error("Failed to get executable path.");
        }

        // 获取可执行文件所在目录
        std::string exeDir = std::string(exePath).substr(0, std::string(exePath).find_last_of("\\/"));

        // 拼接 pcd 文件夹路径
        std::string pcdFolder = exeDir + "\\img";

        // 检查文件夹是否存在，如果不存在则创建
        struct stat info;
        if (stat(pcdFolder.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            if (_mkdir(pcdFolder.c_str()) != 0) {
                throw std::runtime_error("Failed to create directory: " + pcdFolder);
            }
        }

        return pcdFolder;
    }

    // 获取保存的文件名（按000, 001, 002等格式递增）
    static std::string getNextIMGFileName() {
        static int pcd_counter = 0;  // 静态变量，保证跨调用记住上次的编号
        std::stringstream ss;
        ss << "img_" << std::setw(3) << std::setfill('0') << pcd_counter++;  // 格式化为三位数，填充0
        return ss.str();
    }



private:

};




int main(int argc, char* argv[]){
    
    std::mutex mtx;

    //实例化LidarService、cam
    LidarService rslidar;
    CameraService cam1(1, 1366, 768);


    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Core Version: v" << robosense::lidar::getDriverVersion() << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    robosense::lidar::RSDriverParam param;                  ///< Create a parameter object
    param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
    param.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
    param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
    param.lidar_type = robosense::lidar::LidarType::RSHELIOS_16P;   ///< Set the lidar type. Make sure this type is correct
    param.print();

    robosense::lidar::LidarDriver<PointCloudMsg> driver;               ///< Declare the driver object
    driver.regPointCloudCallback(LidarService::driverGetPointCloudFromCallerCallback, LidarService::driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
    driver.regExceptionCallback(LidarService::exceptionCallback);  ///< Register the exception callback function
    if (!driver.init(param))                         ///< Call the init function
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }
    //启动点云线程
   std::thread cloud_handle_thread = std::thread(&LidarService::processCloud, &rslidar);

    driver.start();  ///< The driver thread will start
    RS_DEBUG << "RoboSense Lidar start......" << RS_REND;

    cam1.initCamera();

    //启动相机线程
    //std::thread image_handle_thread = std::thread(&CameraService::processImage, &cam1, cam1.sharedCap);
    std::thread image_handle_thread = std::thread([&cam1]() {
        cam1.processImage(*cam1.sharedCap, cam1.sharedframe);
        });

    

    
    for (int i = 0; i < 20; i++) {

        //10s倒计时用以调整
        for (int j = 10; j > 0; j--) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); //睡1s
            std::cout << "请调整标定板位置" << j << "秒后会进行雷达帧和照片帧的采集" << std::endl;
        }

        //采集数据帧
        {
            std::lock_guard<std::mutex> lock(mtx); // 加锁保护共享资源
            rslidar.savePCD(rslidar.sharedmsg);
            cam1.saveImage(cam1.sharedframe);
        }
    }

    
    std::cout << "已完成20组IMG/PCD采集" << std::endl;

    return 0;
}

