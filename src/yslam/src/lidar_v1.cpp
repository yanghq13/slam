// 添加头文件后需要在CMakelists添加相应的包
// ros用
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
// 数学
#include <cmath>
// 世界统一坐标系
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


ros::Publisher PubLaserCloudUp, PubLaserCloudDown;


// 点云数据回调函数
void cloud_CallHandler(const sensor_msgs::PointCloud2 ros_cloud)
{
    // 初始化
    static tf::TransformBroadcaster br;
    tf::Transform t;
    tf::Quaternion q;
    // 设置原点
    t.setOrigin(tf::Vector3(0,0,0));
    // 设置四元数
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    // 转换坐标系
    t.setRotation(q);
    // 发送变换信息,PointCloud2的header.stamp为时间戳
    br.sendTransform(tf::StampedTransform(t,ros_cloud.header.stamp,"map","map_child"));

    // 点云转换pcl格式
    pcl::PointCloud<pcl::PointXYZ> laserClouIn;
    pcl::fromROSMsg(ros_cloud,laserClouIn);
    // 计数用于循环
    int cloudSize = laserClouIn.points.size();
    int count = cloudSize;
    // PointXYZI包含强度intensity
    pcl::PointXYZI point;
    // 判定各点的线数，按线数保存
    int N_SCANS = 16; // 激光线数
    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);
    for(int i=0;i<cloudSize;i++)
    {
        // 获取输入点云
        point.x = laserClouIn.points[i].x;
        point.y = laserClouIn.points[i].y;
        point.z = laserClouIn.points[i].z;

        // 点云仰角,三角函数求的都是弧度
        float angle = 0;
        angle = atan2(point.z , sqrt(point.x * point.x + point.y * point.y));
        angle = angle * 180 / M_PI;

        // 按线数保存
        int scanID = 0;
        if(N_SCANS == 16)
        {
            // angle在[-15°，+15°]之间,对应1~16线,+0.5相当于四舍五入
            // 这个公式算出来的 [-15°,0°]相当1~8线
            // [0°,15°]相当9~16线
            scanID = int((angle + 15) / 2 + 0.5);
            // ROS_INFO("点云ID是：[%d]", scanID);
            // 防止scanID越界
            if(scanID > (N_SCANS-1) || scanID <0)
            {
                // scanID越界之后，继续下一个循环
                count --;
                continue;
            }
        }

        // 放入数组
        laserCloudScans[scanID].push_back(point);
    }

    // 按1-8,9-16线保存,Ptr为pcl的点云指针，c++的智能指针，不需要写delete
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudUp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDown(new pcl::PointCloud<pcl::PointXYZI>());
    // [-15°,0°]相当1~8线,属于down
    for(int i=0;i<N_SCANS/2;i++)
    {
        *laserCloudDown += laserCloudScans[i];
    }
    // [0°,15°]相当9~16线,属于up
    for(int i=N_SCANS/2;i<N_SCANS;i++)
    {
        *laserCloudUp += laserCloudScans[i];
    }

    // pcl转ros输出格式，map是统一坐标系
    sensor_msgs::PointCloud2 laserCloudUpOutMsg;
    pcl::toROSMsg(*laserCloudUp, laserCloudUpOutMsg);
    laserCloudUpOutMsg.header.stamp = ros_cloud.header.stamp; // 时间戳
    laserCloudUpOutMsg.header.frame_id = "map";
    PubLaserCloudUp.publish(laserCloudUpOutMsg);

    sensor_msgs::PointCloud2 laserCloudDownOutMsg;
    pcl::toROSMsg(*laserCloudDown, laserCloudDownOutMsg);
    laserCloudDownOutMsg.header.stamp = ros_cloud.header.stamp; // 时间戳
    laserCloudDownOutMsg.header.frame_id = "map";
    PubLaserCloudDown.publish(laserCloudDownOutMsg);
}


int main(int argc, char **argv)
{
    // ros初始化
    ros::init(argc,argv,"lidar");
    ros::NodeHandle n;
    setlocale(LC_ALL,""); // 中文语言包
    ROS_INFO("初始化成功");
    // std::cout<<"初始化成功"<<std::endl;
    // 订阅消息
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 100, cloud_CallHandler);
    ROS_INFO("订阅消息成功");
    // 发布消息
    PubLaserCloudUp = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_up", 100);
    PubLaserCloudDown = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_down",100);
    ROS_INFO("发布消息成功");
    // ros消息回调函数,即程序到这就不会往下执行了,相当于在这里停止
    ros::spin();
    return 0;
}






