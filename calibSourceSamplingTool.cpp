#include <rs_driver/api/lidar_driver.hpp>
#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


//#define ORDERLY_EXIT
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;


robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;


std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
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


void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
    //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
    stuffed_cloud_queue.push(msg);
}


void exceptionCallback(const robosense::lidar::Error& code)
{
    // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
    //       so please DO NOT do time-consuming task here.
    RS_WARNING << code.toString() << RS_REND;
}

//增加一个使用PCL库编码msg的函数
void codecPCD(std::shared_ptr<PointCloudMsg> rawPcd, const std::string& filePath) {
    // 检查输入是否为空
    if (!rawPcd) {
        throw std::invalid_argument("rawPCD为空帧！");
    }

    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // 设置点云基本属性
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

    // 保存点云到 PCD 文件
    if (pcl::io::savePCDFileBinary(filePath, *pclCloud) < 0) {
        throw std::runtime_error("保存pcd文件失败");
    }

    std::cout << "PCD file saved to " << filePath << std::endl;
}



bool to_exit_process = false;

void processCloud(void)
{
    while (!to_exit_process)
    {
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
        if (msg.get() == NULL)
        {
            continue;
        }

        // Well, it is time to process the point cloud msg, even it is time-consuming.
        RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

#if 0
        for (auto it = msg->points.begin(); it != msg->points.end(); it++)
        {
            std::cout << std::fixed << std::setprecision(3)
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")"
                << std::endl;
        }
#endif

        free_cloud_queue.push(msg);
    }
}

void processImage(void) {
    //
}

int main(int argc, char* argv[])
{
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
    driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
    driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function
    if (!driver.init(param))                         ///< Call the init function
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }

    std::thread cloud_handle_thread = std::thread(processCloud);

    driver.start();  ///< The driver thread will start
    RS_DEBUG << "RoboSense Lidar start......" << RS_REND;

#ifdef ORDERLY_EXIT
    std::this_thread::sleep_for(std::chrono::seconds(10));

    driver.stop();

    to_exit_process = true;
    cloud_handle_thread.join();
#else
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif

    return 0;
}
