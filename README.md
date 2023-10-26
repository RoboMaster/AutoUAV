# Robomaster自主无人机

## 注意
组装文档中采用了悬挂方案安装飞控，该方案能够极大地减小震动对imu的影响。但是长时间的拉伸会使得皮筋产生脆化而失去弹性。将皮筋更换为工业橡皮筋可缓解该问题。同时碳纤维板上预留了飞控安装孔位，也可以选择直接将飞控安装到碳板上，但是该做法的imu减震性能不佳。

## 简介
为了降低选手参与自主无人机赛事的门槛，减少用于调试安装设备的精力，推出了基于DJI Avata平台，Nvidia Jetson Xiaver NX 和 Ardupilot的开源自主无人机方案。  
*docs/RMUA2023自主无人机开源手册_bom.xlsx*包含有所需设备的详细信息，根据这个表格可以获得所需的所有零部件。  
*docs/RMUA2023自主无人机开源手册.md*详细介绍了安装，调试，参数配置，程序开发的全过程。按照该手册的指引，能够从0开始获得可以自主定位，手动或者离线控制的自主无人机。    
*models*中包含有所需的非标件模型文件。   
*catkin_ws_d430_ros*含有多个基本离线控制程序案例以供参考：
1. 视觉定位 (*catkin_ws_d430_ros/src/d430_slam*)；
2. 定点悬浮 (*catkin_ws_d430_ros/src/offboard*)；
3. 位置控制巡航 (*catkin_ws_d430_ros/src/offboard*)；
4. 速度控制巡航 (*catkin_ws_d430_ros/src/offboard*)；

## License
本项目使用*GNU GPLv3*，查看*LICENSE*文件获得更多信息。


