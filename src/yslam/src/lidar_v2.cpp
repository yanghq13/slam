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


typedef pcl::PointXYZRGB PointType;
ros::Publisher PubLaserCloudEdge;
ros::Publisher PubLaserCloudPlane;
ros::Publisher PubLaserCloudGround;


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
    pcl::PointCloud<PointType> laserClouIn;
    pcl::fromROSMsg(ros_cloud,laserClouIn);
    // 计数用于循环
    int cloudSize = laserClouIn.points.size();
    int count = cloudSize;
    // PointXYZRGB
    PointType point;
    // 判定各点的线数，按线数保存
    int N_SCANS = 16; // 激光线数
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for(int i=0;i<cloudSize;i++)
    {
        // 获取输入点云
        point.x = laserClouIn.points[i].x;
        point.y = laserClouIn.points[i].y;
        point.z = laserClouIn.points[i].z;
        point.r = 1;

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
    pcl::PointCloud<PointType>::Ptr laserCloudEdge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudPlane(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudGround(new pcl::PointCloud<PointType>());

    // lego-loam识别地面，只对1~6线的数据处理
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<int(laserCloudScans[i].size())&&j<int(laserCloudScans[i+1].size());j++)
        {
            float diffX,diffY,diffZ,angle;
            diffX = laserCloudScans[i+1].points[j].x - laserCloudScans[i].points[j].x;
            diffY = laserCloudScans[i+1].points[j].y - laserCloudScans[i].points[j].y;
            diffZ = laserCloudScans[i+1].points[j].z - laserCloudScans[i].points[j].z;
            angle = atan2(diffZ,sqrt(diffX*diffX+diffY*diffY)) * 180/M_PI;
            if(abs(angle)<10)
            {
                laserCloudGround->push_back(laserCloudScans[i+1].points[j]);
                laserCloudGround->push_back(laserCloudScans[i].points[j]);
                laserCloudScans[i+1].points[j].r = 2;
                laserCloudScans[i].points[j].r = 2;
            }
        }
    }

    // 计算曲率
    for(int i=0;i<N_SCANS;i++)
    {
        for(int j=3;j<int(laserCloudScans[i].size())-3;j++)
        {
            float diffX,diffY,diffZ,curvature;
            int diff_num = 6;
            diffX = laserCloudScans[i].points[j-3].x + laserCloudScans[i].points[j-2].x +
                    laserCloudScans[i].points[j-1].x - diff_num * laserCloudScans[i].points[j].x +
                    laserCloudScans[i].points[j+1].x + laserCloudScans[i].points[j+2].x +
                    laserCloudScans[i].points[j+3].x;
            diffY = laserCloudScans[i].points[j-3].y + laserCloudScans[i].points[j-2].y +
                    laserCloudScans[i].points[j-1].y - diff_num * laserCloudScans[i].points[j].y +
                    laserCloudScans[i].points[j+1].y + laserCloudScans[i].points[j+2].y +
                    laserCloudScans[i].points[j+3].y;
            diffZ = laserCloudScans[i].points[j-3].z + laserCloudScans[i].points[j-2].z +
                    laserCloudScans[i].points[j-1].z - diff_num * laserCloudScans[i].points[j].z +
                    laserCloudScans[i].points[j+1].z + laserCloudScans[i].points[j+2].z +
                    laserCloudScans[i].points[j+3].z;
            curvature = double(diffX*diffX + diffY*diffY + diffZ*diffZ);
            laserCloudScans[i].points[j].g = curvature;
            // ROS_INFO("%f",curvature);
        }
    }

    // 根据曲率选取edge点和plane点
    for(int i=0;i<N_SCANS;i++)
    {
        for(int j=5;j<int(laserCloudScans[i].size())-5;j++)
        {
            if(laserCloudScans[i].points[j].r == 1)
            {
                if(laserCloudScans[i].points[j].g > 0.2)
                {
                    laserCloudEdge->push_back(laserCloudScans[i].points[j]);
                    j = j+5;
                }
                if(laserCloudScans[i].points[j].g < 0.1)
                {
                    laserCloudPlane->push_back(laserCloudScans[i].points[j]);
                    j = j+5;
                }
            }
        }
    }


    // pcl转ros输出格式，map是统一坐标系
    sensor_msgs::PointCloud2 laserCloudEdgeOutMsg;
    pcl::toROSMsg(*laserCloudEdge, laserCloudEdgeOutMsg);
    laserCloudEdgeOutMsg.header.stamp = ros_cloud.header.stamp; // 时间戳
    laserCloudEdgeOutMsg.header.frame_id = "map";
    PubLaserCloudEdge.publish(laserCloudEdgeOutMsg);

    sensor_msgs::PointCloud2 laserCloudPlaneOutMsg;
    pcl::toROSMsg(*laserCloudPlane, laserCloudPlaneOutMsg);
    laserCloudPlaneOutMsg.header.stamp = ros_cloud.header.stamp; // 时间戳
    laserCloudPlaneOutMsg.header.frame_id = "map";
    PubLaserCloudPlane.publish(laserCloudPlaneOutMsg);

    sensor_msgs::PointCloud2 laserCloudGroundOutMsg;
    pcl::toROSMsg(*laserCloudGround, laserCloudGroundOutMsg);
    laserCloudGroundOutMsg.header.stamp = ros_cloud.header.stamp; // 时间戳
    laserCloudGroundOutMsg.header.frame_id = "map";
    PubLaserCloudGround.publish(laserCloudGroundOutMsg);
}


int main(int argc, char **argv)
{
    // ros初始化
    ros::init(argc,argv,"lidar_v2");
    ros::NodeHandle n;
    setlocale(LC_ALL,""); // 中文语言包
    ROS_INFO("初始化成功");
    // std::cout<<"初始化成功"<<std::endl;
    // 订阅消息
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 100, cloud_CallHandler);
    ROS_INFO("订阅消息成功");
    // 发布消息
    PubLaserCloudEdge = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_edge", 100);
    PubLaserCloudPlane = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_plane", 100);
    PubLaserCloudGround = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ground", 100);
    ROS_INFO("发布消息成功");
    // ros消息回调函数,即程序到这就不会往下执行了,相当于在这里停止
    ros::spin();
    return 0;
}


