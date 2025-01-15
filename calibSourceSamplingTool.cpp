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

//共享指针初始化
//相机ID 宽 高
std::shared_ptr<int> g_camID = NULL;
std::shared_ptr<int> g_height = NULL;
std::shared_ptr<int> g_width = NULL;

int DEFAULT_INTERVAL = 10;
int DEFAULT_COUNT = 20;
//采样次数和间隔
std::shared_ptr<int> g_sampleInterval = std::make_shared<int>(DEFAULT_INTERVAL);
std::shared_ptr<int> g_sampleCount = std::make_shared<int>(DEFAULT_COUNT);




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
    std::shared_ptr< pcl::PointCloud<pcl::PointXYZI>::Ptr> sharedpclCloud;

    //一般函数
    void processCloud(){

        // 创建 PCL 点云对象
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>());

        bool to_exit_process = false;
        while (!to_exit_process)
        {
            std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
            if (msg.get() == NULL)
            {
                continue;
            }

            pclCloud->width = msg->width;
            pclCloud->height = msg->height;
            pclCloud->is_dense = msg->is_dense;
            pclCloud->points.resize(msg->points.size());

            // 填充点云数据
            for (size_t i = 0; i < msg->points.size(); ++i) {
                pcl::PointXYZI p;
                p.x = msg->points[i].x;
                p.y = msg->points[i].y;
                p.z = msg->points[i].z;
                p.intensity = msg->points[i].intensity; // 保留强度信息
                pclCloud->points[i] = p;
            }


            //写到共享变量sharedmsg
            sharedpclCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>::Ptr>(pclCloud);

            free_cloud_queue.push(msg);
        
        }
    }

    void savePCD(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>::Ptr> sharedpclCloud) {
        if (!sharedpclCloud) {

            std::cout << "PCD为空帧！" << std::endl;

            throw std::invalid_argument("rawPCD为空帧！");
        }

        

        // 获取保存路径和文件名
        std::string pcdFolder = getPCDFolderPath(); // 假设实现了这个函数
        if (pcdFolder.empty()) {
            throw std::runtime_error("PCD文件夹路径无效！");
        }
        std::string filePath = pcdFolder + "\\" + getNextPCDFileName() + ".pcd";

        // 保存点云到 PCD 文件
        if (pcl::io::savePCDFileBinary(filePath, **sharedpclCloud) < 0) {
            throw std::runtime_error("保存PCD文件失败!");
        }

        std::cout << "PCD file 保存至: " << filePath << std::endl;
    }

    
    // 将msg中的点云信息以时间戳命名并保存为PCD
/*
void savePointCloudToPCD(const std::shared_ptr<PointCloudMsg>& msg)
{

    // 获取当前可执行文件的路径
    std::string exePath = getExecutablePath();
    std::string directory = exePath + "\\saved_pcd";

    std::string timestamp = getCurrentTimestamp();
    std::stringstream ss;
    ss << directory << "./saved_pcd" << "pointcloud_" << timestamp << "_frame_" << std::setw(5) << std::setfill('0') << msg->seq << ".pcd";
    std::string filename = ss.str();
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (const auto& point : msg->points)
    {
        pcl::PointXYZI p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = point.intensity;
        cloud.push_back(p);
    }

    if (pcl::io::savePCDFileBinary(filename, cloud) == -1)
    {
        RS_ERROR << "Failed to save PCD file " << filename << RS_REND;
    }
    else
    {
        RS_MSG << "Saved point cloud to " << filename << RS_REND;
    }
}
*/



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
            throw std::runtime_error("错误：获取exe路径失败！");
        }

        // 获取可执行文件所在目录
        std::string exeDir = std::string(exePath).substr(0, std::string(exePath).find_last_of("\\/"));

        // 拼接 pcd 文件夹路径
        std::string pcdFolder = exeDir + "\\pcd";

        // 检查文件夹是否存在，如果不存在则创建
        struct stat info;
        if (stat(pcdFolder.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            if (_mkdir(pcdFolder.c_str()) != 0) {
                throw std::runtime_error("创建目录失败: " + pcdFolder);
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
    std::shared_ptr<cv::Mat> sharedframe; ///< 共享指针，用于存储当前帧图像数据
    std::shared_ptr<cv::VideoCapture> sharedCap; ///< 共享指针，用于摄像头捕获对象

    int cameraID; ///< 摄像头ID
    int width; ///< 图像宽度
    int height; ///< 图像高度

    CameraService(int camID = 0, int w = 1, int h = 1)
        : cameraID(camID), width(w), height(h) {
        sharedframe = std::make_shared<cv::Mat>(height, width, CV_8UC3); ///< 初始化共享帧
    }

    void saveImage(std::shared_ptr<cv::Mat> sharedframe) {
        // 保存图像为 JPG 文件
        static int img_counter = 0;

        // 获取保存路径和文件名
        std::string imgFolder = getIMGFolderPath();
        std::string filePath = imgFolder + "\\" + getNextIMGFileName() + ".jpg";

        if (!cv::imwrite(filePath, *sharedframe)) {
            std::cerr << "保存 JPG 文件失败！" << std::endl;
        }
        else {
            std::cout << "JPG 文件已保存到: " << filePath << std::endl;
        }
    }

    void processImage(cv::VideoCapture cap, std::shared_ptr<cv::Mat>& shareframe) {
        // 用于不断采集图像数据的线程函数

        // 创建一个窗口用于显示摄像头画面
        cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

        while (true) {
            cv::Mat frame;

            // 从摄像头读取一帧图像
            cap >> frame;

            // 检查图像是否为空
            if (frame.empty()) {
                std::cerr << "错误: 接收到空帧！" << std::endl;
                break;
            }

            // 更新共享帧数据
            shareframe = std::make_shared<cv::Mat>(frame);
            cv::Mat shrink;
            cv::Size dsize = cv::Size(int(width / 4), int(height / 4));
            shrink.create(dsize, frame.type());
            cv::resize(frame, shrink, dsize, 0, 0, cv::INTER_AREA);

            // 显示图像
            cv::imshow("Camera", shrink);

            // 按下 'q' 键退出循环
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }

        // 释放摄像头资源
        cap.release();

        // 销毁窗口
        cv::destroyAllWindows();
    }

    bool initCamera() {
        // 打开摄像头并启动图像处理线程
        cv::VideoCapture cap(cameraID);

        // 检查摄像头是否成功打开
        if (!cap.isOpened()) {
            std::cerr << "错误: 无法打开摄像头！" << std::endl;
            return false;
        }

        // 设置分辨率
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);   // 设置宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 设置高度

        // 获取实际分辨率并输出
        int actualWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actualHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        std::cout << "分辨率已设置为: " << actualWidth << "x" << actualHeight << std::endl;

        // 分配摄像头捕获对象的共享指针
        sharedCap = std::make_shared<cv::VideoCapture>(cap);

        return true;
    }

    static std::string getIMGFolderPath() {
        char exePath[MAX_PATH];
        // 获取当前可执行文件的路径
        if (GetModuleFileNameA(NULL, exePath, MAX_PATH) == 0) {
            throw std::runtime_error("无法获取可执行文件路径！");
        }

        // 获取可执行文件所在目录
        std::string exeDir = std::string(exePath).substr(0, std::string(exePath).find_last_of("\\/"));

        // 拼接 img 文件夹路径
        std::string imgFolder = exeDir + "\\img";

        // 检查文件夹是否存在，如果不存在则创建
        struct stat info;
        if (stat(imgFolder.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            if (_mkdir(imgFolder.c_str()) != 0) {
                throw std::runtime_error("创建目录失败: " + imgFolder);
            }
        }

        return imgFolder;
    }

    // 获取保存文件的名称（格式为 img_000, img_001 等递增）
    static std::string getNextIMGFileName() {
        static int img_counter = 0;  // 静态变量，用于记录编号
        std::stringstream ss;
        ss << "img_" << std::setw(3) << std::setfill('0') << img_counter++;  // 格式化为三位数，补零
        return ss.str();
    }

private:
};



class ProgramHelper {
public:
    // 显示用法信息
    static void showUsage(const std::string& programName) {
        std::cout << "用法：" << programName << " <摄像头ID> <宽度> <高度>\n"
            << "选项说明：\n"
            << "  <摄像头ID> <宽度> <高度>   正常运行程序，指定摄像头ID和分辨率。\n"
            << "  -v                         显示程序版本信息。\n"
            << "  -help                      显示此帮助信息。\n"
            << "  -t <采样间隔> <采样次数>   指定采样间隔和采样次数。\n"
            << std::endl;
    }

    // 显示版本信息
    static void showVersion() {
        std::cout << "版本：alpha\n"
            << "作者：sbhinx\n"
            << "日期：2025-01-15\n"
            << std::endl;
    }

    static int handleArguments(int argc, char* argv[]) {
        const std::string programName = argv[0];
        int sampleInterval = 0;  // 采样间隔
        int sampleCount = 0;     // 采样次数

        // 处理 -v 或 -help 参数
        if (argc == 2) {
            std::string option = argv[1];
            if (option == "-v") {
                showVersion();
                return 0;
            }
            else if (option == "-help") {
                showUsage(programName);
                return 0;
            }
            else {
                std::cerr << "错误：未知选项 '" << option << "'！\n";
                std::cerr << "提示：使用 -help 查看用法。" << std::endl;
                return 1;
            }
        }
        // 处理摄像头ID、宽度和高度参数
        else if (argc >= 4) {
            try {
                // 解析摄像头ID、宽度和高度
                int camID = std::stoi(argv[1]);
                g_camID = std::make_shared<int>(camID);

                int width = std::stoi(argv[2]);
                g_width = std::make_shared<int>(width);

                int height = std::stoi(argv[3]);
                g_height = std::make_shared<int>(height);

                // 输出参数（可替换为实际的程序逻辑）
                std::cout << "运行程序：\n"
                    << "  摄像头ID：" << camID << "\n"
                    << "  分辨率：" << width << "x" << height << "\n"
                    << "程序启动中..." << std::endl;

                // 检查是否存在 -t 参数
                if (argc >= 5 && std::string(argv[4]) == "-t") {
                    // 解析采样间隔和采样次数

                    if (argc < 7) {
                        std::cerr << "错误：-t 参数后面需要跟采样间隔和采样次数！\n";
                        std::cerr << "提示：使用 -help 查看用法。" << std::endl;
                        return 1;
                    }

                    sampleInterval = std::stoi(argv[5]);
                    g_sampleInterval = std::make_shared<int>(sampleInterval);

                    sampleCount = std::stoi(argv[6]);
                    g_sampleCount = std::make_shared<int>(sampleCount);

                    if (sampleInterval <= 0 || sampleCount <= 0) {
                        std::cerr << "错误：采样间隔和采样次数必须为正整数！\n";
                        std::cerr << "提示：使用 -help 查看用法。" << std::endl;
                        return 1;
                    }

                    std::cout << "采样间隔: " << sampleInterval << " 秒\n";
                    std::cout << "采样次数: " << sampleCount << " 次\n";
                }

                return 0;
            }
            catch (const std::exception& e) {
                std::cerr << "错误：参数必须是整数！\n";
                std::cerr << "提示：使用 -help 查看用法。" << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "错误：参数数量错误！\n";
            std::cerr << "提示：使用 -help 查看用法。" << std::endl;
            return 1;
        }
    }


};

int main(int argc, char* argv[]) {

        // 调用静态类的方法处理参数
    if (ProgramHelper::handleArguments(argc, argv)== 1) {
        return 1;
        }

    //声明锁
    std::mutex mtx;

    //实例化LidarService、cam
    LidarService rslidar;

    CameraService cam1(*g_camID, *g_width, *g_height);
    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver版本: v" << robosense::lidar::getDriverVersion() << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    robosense::lidar::RSDriverParam param;                  ///< 创建参数对象
    param.input_type = robosense::lidar::InputType::ONLINE_LIDAR; ///< 设置雷达输入类型为在线模式
    param.input_param.msop_port = 6699;   ///< 设置雷达 MSOP 数据端口号，默认值为 6699
    param.input_param.difop_port = 7788;  ///< 设置雷达 DIFOP 数据端口号，默认值为 7788
    param.lidar_type = robosense::lidar::LidarType::RSHELIOS_16P;   ///< 设置雷达类型，请确保类型设置正确
    param.print(); ///< 打印参数信息

    robosense::lidar::LidarDriver<PointCloudMsg> driver;               ///< 声明雷达驱动对象
    driver.regPointCloudCallback(LidarService::driverGetPointCloudFromCallerCallback,
    LidarService::driverReturnPointCloudToCallerCallback); ///< 注册点云回调函数
    driver.regExceptionCallback(LidarService::exceptionCallback);  ///< 注册异常回调函数
    
    if (!driver.init(param)){                         ///< 调用初始化函数
            RS_ERROR << "驱动初始化失败..." << RS_REND;
            return -1;
    }
    // 启动点云处理线程
    std::thread cloud_handle_thread = std::thread(&LidarService::processCloud, &rslidar);

    driver.start();  ///< 启动雷达驱动线程
    RS_DEBUG << "已开始处理雷达数据包......" << RS_REND;

    cam1.initCamera(); ///< 初始化相机

    // 启动相机处理线程
    // std::thread image_handle_thread = std::thread(&CameraService::processImage, &cam1, cam1.sharedCap);
    std::thread image_handle_thread = std::thread([&cam1]() {
        cam1.processImage(*cam1.sharedCap, cam1.sharedframe); ///< 调用相机图像处理函数
            });

    // 数据采集循环
    for (int i = 0; i < *g_sampleCount; i++) {
        // 10 秒倒计时用于调整标定板位置
            
        for (int j = *g_sampleInterval; j > 0; j--) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 休眠 1 秒
            std::cout << "请调整标定板位置，" << j << " 秒后将进行雷达帧和照片帧采集" << std::endl;
        }

        // 采集数据帧
        {
            std::lock_guard<std::mutex> lock(mtx); // 加锁以保护共享资源
            rslidar.savePCD(rslidar.sharedpclCloud); ///< 保存雷达点云数据
            cam1.saveImage(cam1.sharedframe);   ///< 保存相机图像数据
        }
    }

    std::cout << "已完成 "<< *g_sampleCount<<"组图像/点云数据采集（可手动退出程序）" << std::endl;

    // 等待线程结束
    image_handle_thread.join();
    cloud_handle_thread.join();

    return 0;
}