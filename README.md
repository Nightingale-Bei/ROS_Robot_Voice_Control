# ROS_Robot_Voice_Control




## 项目简介

本项目基于ROS机器人操作系统，使用科大讯飞的离线命令词识别SDK，设计一个离线语音识别程序。将语音识别与机器人控制结合，由语音识别程序识别输入的语音，再由控制程序接收语音识别结果并发送控制指令，采用ros_control控制框架，在Gazebo仿真中实现语音控制巡视小车车体的运动和摄像机的转动。

控制系统的总设计如图所示。


![image-20250303094633923](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303094633923.png)



## 开发环境

- Ubuntu  20.04 LTS
- ROS Noetic Ninjemys （ros-noetic-desktop-full）



## 演示视频

运行效果[视频链接](https://www.bilibili.com/video/BV1no9eY3EQZ)。





---
---
以下为项目简要描述，详细开发文档参考[报告](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/%E5%BC%80%E5%8F%91%E6%8A%A5%E5%91%8A.pdf)





## 语音识别

本项目选择使用科大讯飞平台离线**命令词**SDK来设计语音识别程序。

命令词识别（关键词检测）：仅仅辨识用户预设的特定关键词，而不是别其他词语。由于识别结果的范围只在语法规则文件所列出的规则里，故有很好的识别准确度，且减少程序的大小。

命令词的识别需要一个语法规则文件，本项目采用 巴科斯范式语法（BNF）构建控制命令。命令词构建参考[编写指南](https://developer.xfyun.cn/thread/7595)。

BNF代码如下：

```
#BNF+IAT 1.0 UTF-8;
!grammar call;
!slot <want>;
!slot <direction>;
!slot <action>;
!start <callstart>;
<callstart>:[<want>]<control>;
<want>:向|看;
<control>:<direction><action>|<direction>|<action>;
<direction>:前|后|左|左边|右|右边|上|下|中间;
<action>:走|转|看|停止|前进|后退|左转|右转;
```

小车的控制指令分为车体运动指令和2D摄像机转动指令。

车体运动指令为：“前进”、“后退”、“左转”、“右转”和“停止”；

2D摄像机转动指令为：“向左看”、“向右看”、“向前看”、“向后看”、“看中间”、“向上看”和“向下看”。



语音识别程序的流程图如图所示。

<img src="https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303095127442.png" alt="image-20250303095127442" style="zoom:67%;" />





## 机器人建模

采用 URDF（Unified Robot Description Format）自定义建立一个巡视小车模型。其效果如下图所示(Rviz中的效果)。

模型由底盘、驱动轮、万向轮、转轴、2D摄像机和RGB-D摄像机组成。其中，驱动轮由电机驱动，2D摄像机可在Z轴无限位旋转和Y轴限位旋转。

![image-20250303093305913](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image.png)

使用命令行工具urdf_to_graphiz命令查看小车的URDF模型的整体结构，如下图所示。

![image-20250303094830453](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303094830453.png)







## 控制框架

### ROS_Control

ROS中的ros_control是为控制机器人而设计的一个中间件，它向开发者提供了一系列工具和接口，包括控制器接口、传动装置接口、硬件接口和控制器工具箱。这些组件可以加快机器人应用功能包的开发进程，提升开发效率。

![image-20250303094254218](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303094254218.png)

![image-20250303094403423](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303094403423.png)



### 控制程序

在接收到符合关键词的文本指令后，语音控制程序便会向Gazebo控制节点发布控制消息。控制程序的流程图如下图所示。

<img src="https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303095318745.png" alt="image-20250303095318745" style="zoom:67%;" />

### 机器人控制器配置

在小车URDF模型的基础上，为小车的两个驱动轮关节配置速度控制器joint_velocity_controller，为2D摄像机的两个旋转轴关节配置位置控制器joint_position_controller。

控制器需要使用YAML配置文件加载入ROS的参数服务器，YAML配置文件定义控制器的类型、关联的关节和PID的各项参数及其最值。



## 控制仿真

### Gazebo

ROS中的Gazebo是一个功能强大的三维物理仿真平台，在Gazebo中，控制机器人的控制器是通过使用ros_control和一个Gazebo插件适配器来完成。

![image-20250303094518521](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303094518521.png)



### Gazebo加载模型

Gazebo导入小车模型界面

![image-20250303095554369](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303095554369.png)





### 仿真环境搭建

使用Gazebo的模型库中的物件模型，构建一个仿真环境。

![image-20250303095104474](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303095104474.png)



### 配置控制器插件

控制小车在Gazebo中运动，需要使用Gazebo插件libgazebo_ros_control.so 。

并在机器人的藐视文件添加如下插件声明。

```	xml
<plugin name="controllers" filename="libgazebo_ros_control.so">
   <robotNamespace>/mbot</robotNamespace>
   <controlPeriod>0.01</controlPeriod>
   <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
</plugin>
```





## 运行效果

启动Gazebo仿真软件，加载小车模型和仿真环境，加载控制器，同时启动语音识别程序。系统运行效果如下图所示。

![image-20250303100906195](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303100906195.png)

图中左侧两个终端窗口分别为语音识别程序运行窗口和控制程序运行窗口，中间为Gazebo仿真界面，右侧窗口的两个画面分别为2D摄像机和RGB-D摄像机的实时拍摄画面。



### 总体通信数据流

![image-20250303101032817](https://github.com/Nightingale-Bei/ROS_Robot_Voice_Control/blob/main/ReadmeFile/Readme.assets/image-20250303101032817.png)

语音识别节点/asr_1发布消息到/iat话题，控制节点/control订阅该话题接收识别文本消息，同时发布控制指令消息到四个控制器话题，再由Gazebo中的节点/gazebo订阅控制器话题，进而控制小车运动。同时节点/gazebo发布消息到/camera和/kinect两个摄像机话题。

