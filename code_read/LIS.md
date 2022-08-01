# Lidar-IMU 系统

参考：https://blog.csdn.net/iwanderu/article/details/123058727

paper：

## 一、概述

从launch或CMakeLists.txt文件中可以看出：有关LIS的有4个可执行文件

1. imuPreintegration
2. imageProjection
3. featureExtraction
4. mapOptmization

有关VIS的有3个可执行文件：

1. visual_odometry
1. visual_feature
1. visual_loop



```xml
<launch>

    <arg name="project" default="lvi_sam"/>
    <!-- Lidar odometry param -->
    <rosparam file="$(find lvi_sam)/config/params_lidar.yaml" command="load" />
    <!-- VINS config -->
    <param name="vins_config_file" type="string" value="$(find lvi_sam)/config/params_camera.yaml" />
    
    <!-- Lidar odometry -->
    <node pkg="$(arg project)"      type="$(arg project)_imuPreintegration"   name="$(arg project)_imuPreintegration"    output="screen"     respawn="true"/>
    <node pkg="$(arg project)"      type="$(arg project)_imageProjection"     name="$(arg project)_imageProjection"      output="screen"     respawn="true"/>
    <node pkg="$(arg project)"      type="$(arg project)_featureExtraction"   name="$(arg project)_featureExtraction"    output="screen"     respawn="true"/>
    <node pkg="$(arg project)"      type="$(arg project)_mapOptmization"      name="$(arg project)_mapOptmization"       output="screen"     respawn="true"/>

    <!-- Visual feature and odometry -->
    <node pkg="$(arg project)"      type="$(arg project)_visual_feature"      name="$(arg project)_visual_feature"       output="screen"     respawn="true"/>
    <node pkg="$(arg project)"      type="$(arg project)_visual_odometry"     name="$(arg project)_visual_odometry"      output="screen"     respawn="true"/>
    <node pkg="$(arg project)"      type="$(arg project)_visual_loop"         name="$(arg project)_visual_loop"          output="screen"     respawn="true"/>

    <!-- Image conversion -->
    <node pkg="image_transport" type="republish" name="$(arg project)_republish" args="compressed in:=/camera/image_raw raw out:=/camera/image_raw" output="screen" respawn="true"/>

</launch>

```

## 二、LIS——imageProjection

### i. main函数声明节点

```c++
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    ImageProjection IP;//声明一个ImageProjection类的对象IP
    
    ROS_INFO("\033[1;32m----> Lidar Cloud Deskew Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);//涉及到多线程
    spinner.spin();
    
    return 0;
}
```

#### 附：ros多线程

参考：https://zhuanlan.zhihu.com/p/375418691

### ii. 构造函数

```c++
ImageProjection():deskewFlag(0)
{
    //Imu原始测量数据订阅：
    //话题：imuTopic -> 在yaml文件里可以找到
    //回调函数：ImageProjection::imuHandler
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
    //里程计订阅
    //话题：PROJECT_NAME+"/vins/odometry/imu_propagate_ros"
    //回调函数：ImageProjection::odometryHandler
    subOdom = nh.subscribe<nav_msgs::Odometry>(PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    //激光雷达原始数据订阅
    //话题：pointCloudTopic -> 在yaml文件里可以找到
    //回调函数：ImageProjection::cloudHandler
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
	
    //发布ExtractedCloud ， lvi_sam_visual_feature节点接收
    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> (PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
    //发布LaserCloudInfo ， lvi_sam_featureExtraction节点接收
    pubLaserCloudInfo = nh.advertise<lvi_sam::cloud_info>      (PROJECT_NAME + "/lidar/deskew/cloud_info", 5);
	
    allocateMemory();//分配内存
    resetParameters();//重置参数

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}
```

#### 附：C++ std::deque

参考：https://blog.csdn.net/sinat_31608641/article/details/108089691

#### 附：C++ std::mutex  std::lock_guard

参考：

### iii. 回调函数

```c++
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
}

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (!cachePointCloud(laserCloudMsg))
        return;

    if (!deskewInfo())
        return;

    projectPointCloud();

    cloudExtraction();

    publishClouds();

    resetParameters();
}
```

### iv. 点云数据处理

基础参考：https://blog.csdn.net/scott198510/article/details/123105304

​	在`cloudHandler函数`中，依次讲解其中所涉及到的函数：

参考：https://blog.csdn.net/QLeelq/article/details/111942416

#### (1)、cachePointCloud函数

​	这个函数实际上是真正的雷达接收函数，将接收的数据存入cloudQueue当中，取队列当中的front数据进行存入currentCloundMsg,并删除cloudQueue的front数据。最终会转成pcl格式的laserCloudIn数据。

```c++
bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // cache point cloud
    cloudQueue.push_back(*laserCloudMsg);//把新来的数据插入队列
	//如果队列小(数据少->前两帧)，则返回
    if (cloudQueue.size() <= 2)
        return false;
    else
    {
        //把前端数据拷贝到currentCloudMsg中
        currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();//前端删除
		
        //记录时间
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanNext = cloudQueue.front().header.stamp.toSec();
    }

    // convert cloud
    //ros的格式转到pcl的格式
    pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

    // check dense flag
    if (laserCloudIn->is_dense == false)
    {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    // check ring channel
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
        ringFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        {
            if (currentCloudMsg.fields[i].name == "ring")
            {
                ringFlag = 1;
                break;
            }
        }
        if (ringFlag == -1)
        {
            ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }     

    // check point time
    //这个在处理畸变的时候会起作用。
    if (deskewFlag == 0)
    {
        deskewFlag = -1;
        for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        {
            if (currentCloudMsg.fields[i].name == timeField)
            {
                deskewFlag = 1;
                break;
            }
        }
        if (deskewFlag == -1)
            ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
}
```

#### (2)、deskwInfo函数

中转，等待IMU数据可靠后，转到imuDeskewInfo()和odomDeskewInfo()函数。

```c++
bool deskewInfo()
{
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);

    // make sure IMU data available for the scan
    if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    imuDeskewInfo();

    odomDeskewInfo();

    return true;
}
```

#### (3)、imuDeskewInfo函数

该函数用于处理畸变：计算imu转角。

```c++
void imuDeskewInfo()
{
    cloudInfo.imuAvailable = false;
	//只取当前帧时间前10ms的imu数据，其他抛弃。
    while (!imuQueue.empty())
    {
        if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            imuQueue.pop_front();
        else
            break;
    }

    if (imuQueue.empty())
        return;

    imuPointerCur = 0;
	//循环读取imu队列数据
    for (int i = 0; i < (int)imuQueue.size(); ++i)
    {
        sensor_msgs::Imu thisImuMsg = imuQueue[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        // get roll, pitch, and yaw estimation for this scan
        if (currentImuTime <= timeScanCur)
            imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
		
        //如果当前IMU时间在下一帧+10ms之后退出
        if (currentImuTime > timeScanNext + 0.01)
            break;
		
        //第一个IMU数据认为是当前坐标系(为后续积分准备)
        //
        if (imuPointerCur == 0){
            imuRotX[0] = 0;
            imuRotY[0] = 0;
            imuRotZ[0] = 0;
            imuTime[0] = currentImuTime;
            ++imuPointerCur;
            continue;
        }

        // get angular velocity
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // integrate rotation
        //积分出这一段时间的imu转角
        double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        imuTime[imuPointerCur] = currentImuTime;
        ++imuPointerCur;
    }

    --imuPointerCur;
	//IMU数据太少，直接返回，IMU数据不可用
    if (imuPointerCur <= 0)
        return;
	//IMU数据可用
    cloudInfo.imuAvailable = true;
}
```

#### (4)、odomDeskewInfo函数(unfinished)

读取odom数据，并根据协方差判断是否可信。

```c++
void odomDeskewInfo()
{
    cloudInfo.odomAvailable = false;
	//和IMU类似
    while (!odomQueue.empty())
    {
        if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            odomQueue.pop_front();
        else
            break;
    }

    if (odomQueue.empty())
        return;

    if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        return;

    // get start odometry at the beginning of the scan
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        startOdomMsg = odomQueue[i];
		//由于之前已经将小于timeScanCur超过0.01的数据推出  所以startOdomMsg已经可代表起始激光扫描的起始时刻的里程计消息
        if (ROS_TIME(&startOdomMsg) < timeScanCur)
            continue;
        else
            break;
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    //利用里程计消息生成初始位姿估计，在地图优化中使用的，具体需要看对应代码
    cloudInfo.odomX = startOdomMsg.pose.pose.position.x;
    cloudInfo.odomY = startOdomMsg.pose.pose.position.y;
    cloudInfo.odomZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.odomRoll  = roll;
    cloudInfo.odomPitch = pitch;
    cloudInfo.odomYaw   = yaw;
    cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    //获得一帧扫描末尾的里程计消息，这个就跟初始位姿估计没有关系，只是用于去畸变运动补偿
    odomDeskewFlag = false;
	//相关时间先后关系的判断  odom队列中最后的数据大于下一帧扫描的时间
    if (odomQueue.back().header.stamp.toSec() < timeScanNext)
        return;

    nav_msgs::Odometry endOdomMsg;
	
    //取出下一帧雷达数据时刻的里程计数据。
    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        endOdomMsg = odomQueue[i];
        if (ROS_TIME(&endOdomMsg) < timeScanNext)
            continue;
        else
            break;
    }
	//协方差矩阵的判断
    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;
	//获得起始变换
    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);
	//获得结束时的变换
    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);
	 //获得一帧扫描起始与结束时刻间的变换（相对）
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    float rollIncre, pitchIncre, yawIncre;
     //通过tranBt 获得增量值  后续去畸变用到
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odomDeskewFlag = true;
}
```

#### (5)、projectPointCloud函数

​	这个函数用于将点云信息laserCloudIn投影到深度图信息，并进行运动去畸变deskewPoint()。

​	比如3D激光雷达 是16x1800的，说明，点云数据16行1800列。我们已经知道这些点相对雷达的x,y,z坐标，我们想知道这些点相对于雷达的具体分布如何。

​	遍历所有点，通过传感器信息得到点的行，再根据点所对应的水平角度判断点所在的列。我们弄一个16x1800的矩阵，把对应的点的深度信息放入即可。求深度直接勾股定理。

​	之后，对每一个点去畸变，并且按照顺序存入fullCloud。点的index为 `列+行x每行点数(1800)`。

```c++
void projectPointCloud()
{
    //这里的laserCloudIn是激光雷达原始信息的PCL形式，（原始信息是ROS消息形式）
    int cloudSize = (int)laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        PointType thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.intensity = laserCloudIn->points[i].intensity;
		
        //从点云数据中获得线束信息作为行最终距离图像就是16*1800
        //如果不符合范围则continue
        int rowIdn = laserCloudIn->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        if (rowIdn % downsampleRate != 0)
            continue;
		//激光点的水平角度
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        //角分辨率 360/1800，一圈有1800个点，根据该雷达类型，有十六线
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        //计算在距离图像上点属于哪一列
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        //超了一圈的情况
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;
		//调用utility.h里的函数计算点到雷达的距离
        float range = pointDistance(thisPoint);
		//点的过滤，太近太远都不要
        if (range < 1.0)
            continue;

        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        // for the amsterdam dataset
        // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
        //     continue;
        // if (thisPoint.z < -2.0)
        //     continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
        // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

        int index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
}
```



#### (6)、deskewPoint函数

​	这个是真正的运动去畸变函数，可以当做一个工具，平时处理雷达数据的时候也可以使用。
​	去畸变后的函数保存点云到thisPoint -> fullCloud -> extractedCloud最后通过cloudInfo发布出去。

```c++
//realtime是该点的准确时间(在一帧数据中)
PointType deskewPoint(PointType *point, double relTime)
{
    //IMU数据不可用 或者 不去畸变 -> 返回本体
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
        return *point;
	//当前点在一帧数据中的准确时间+当前雷达初始时间 = 当前点时间
    double pointTime = timeScanCur + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if (firstPointFlag == true)
    {
        transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        firstPointFlag = false;
    }

    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;
	
    //根据imu odom 对点云去畸变
    PointType newPoint;
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}
```



#### (7)、findRotation函数

有插值过程，和2D激光雷达去畸变原理类似。

```c++
void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur)
    {
        if (pointTime < imuTime[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
    {
        *rotXCur = imuRotX[imuPointerFront];
        *rotYCur = imuRotY[imuPointerFront];
        *rotZCur = imuRotZ[imuPointerFront];
    } else {
        //插值
        int imuPointerBack = imuPointerFront - 1;
        double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
        *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
        *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
}
```



#### (8)、findPosition函数



```c++
void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanNext - timeScanCur);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
}
```

#### (9)、cloudExtraction函数

点云提取函数，提取完了之后保存到pcl格式的extractedCloud中

```c++
void cloudExtraction()
{
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i)
    {
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            if (rangeMat.at<float>(i,j) != FLT_MAX)
            {
                // mark the points' column index for marking occlusion later
                cloudInfo.pointColInd[count] = j;
                // save range info
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                // save extracted cloud
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
}
```



#### (10)、发布去畸变后的点云数据

发布extractedCloud点云->pcl形式->ROS形式

发布点云信息cloudInfo ，包含每个点的深度信息 以及 index。

```c++
void publishClouds()
{
    cloudInfo.header = cloudHeader;
    //发布去畸变的点，并且把去畸变的点赋值给 cloudInfo.cloud_deskewed中
    //最后发布cloudInfo
    cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
    pubLaserCloudInfo.publish(cloudInfo);
}
```



## 三、LIS——featureExtraction

点云特征提取

### i. main函数声明节点

​	这是一个节点，就是一个可执行文件，如下：

```c++
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Lidar Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}
```

我们可以看出，主要功能概述可能在构造函数中。

### ii. 构造函数

```c++
FeatureExtraction()
{
    //订阅雷达数据：
    //话题：PROJECT_NAME+"/lidar/deskew/cloud_info"
    //根据前面分析，这个话题发布的是 去畸变后的点云+点云深度信息
    subLaserCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
	
    //发布：点云特征
    pubLaserCloudInfo = nh.advertise<lvi_sam::cloud_info> (PROJECT_NAME + "/lidar/feature/cloud_info", 5);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_corner", 5);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_surface", 5);

    initializationValue();
}
```

### iii. 回调函数

里面包含了提取特征的所有过程

```c++
void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr& msgIn)
{
    //extractedCloud是去畸变后的点云
    cloudInfo = *msgIn; // new cloud info
    cloudHeader = msgIn->header; // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction
	
    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
}
```

### iv. 提取特征过程

根据回调函数，我们依次分析四个函数内容以及功能。

#### (1)、calculateSmoothness函数

计算每个点的曲率，用来进行特征提取。

​	计算曲率方法比较粗暴：直接取每个点的前后5个点，计算每个点与该点的向量差的和。如果是一条直线的话，曲率是0；如果是角点的话，曲率较大。

```c++
void calculateSmoothness()
{
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
            + cloudInfo.pointRange[i+5];            

        cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}
```

#### (2)、markOccludedPoints函数

该函数用于去除质量差的点，包括但不限于：直线与激光束几乎平行；被遮挡的等等。

```c++
void markOccludedPoints()
{
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {
        // occluded points
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

        //列数差不超过10的情况下
        //深度1>深度2 0.3时，不取点1前面的5个点
        //深度2>深度1 0.3时，不取点2后面的5个点
        if (columnDiff < 10){
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        //判断平行条件
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1;
    }
}
```

#### (3)、extractFeatures函数

提取特征：直线、平面等特征

```c++
// 进行角点与面点的提取，角点直接保存，面点要经过降采样之后再保存
void extractFeatures()
{
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++)
    {
        surfaceCloudScan->clear();

        for (int j = 0; j < 6; j++)
        {

            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 20){
                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                {

                    cloudLabel[ind] = -1;
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0){
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }
}
```











