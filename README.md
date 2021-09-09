# slam-入门
这是一个十分简单的slam入门教程。

# 1.安装

需要安装ros，pcl

需要下载nsh_indoor_outdoor.bag，将该文件移动到yslam/src/yslam/bag

需要克隆

```bash
git clone https://github.com/yanghq13/slam.git
```

# 2.运行

打开一个终端，运行v1

```
roslaunch yslam run_lidar.launch
```

打开一个终端，运行v2

```
roslaunch yslam run_lidar_v2.launch
```

# 3.效果

成功运行v1之后，会看到这样的画面

![v1](https://github.com/yanghq13/slam/blob/main/image/v1.png)

成功运行v2之后，会看到这样的画面

![v2](https://github.com/yanghq13/slam/blob/main/image/v2.png)

