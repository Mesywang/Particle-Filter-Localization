#include <ros/ros.h>
#include "localization_node.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"
#include "stdlib.h"

using namespace std;


int main(int argc, char** argv)
{

    ros::init(argc, argv,"localization_node");	 //初始化节点
    // ros::NodeHandle n;
    ros::NodeHandle n;

    string package_path;
    string map_local_path = "pf_localization/data/map/wean.dat";
    string log_local_path = "pf_localization/data/log/robotdata1.log";

    n.getParam("/pf_localization/package_path_param",package_path);
    string complete_map_path = package_path + map_local_path;
    string complete_log_path = package_path + log_local_path;
    const char* complete_map_path_c = complete_map_path.c_str();
    const char* complete_log_path_c = complete_log_path.c_str();

    LoadMap load_map(complete_map_path_c);          //加载map
    LoadLog load_log(complete_log_path_c);          //加载odometry和lidar数据

    // LoadMap load_map("/home/wsy/catkin_ws/src/pf_localization/data/map/wean.dat");          //加载map
    // LoadLog load_log("/home/wsy/catkin_ws/src/pf_localization/data/log/robotdata1.log");    //加载odometry和lidar数据

    map_type* map = load_map.GetMap();                      //获取地图
    vector<log_data> logfile_data = load_log.GetLog();      //获取数据集

    load_map.PublishMap(n);     //发布map，rviz订阅此topic

    PFLocalization pf_localization(map, logfile_data, n);    
    pf_localization.InitParticles();    //粒子集初始化
    sleep(5);		                    //粒子初始化后延时5秒后开始跑数据集
    pf_localization.MCLAlgorithm();     //运行蒙特卡洛定位算法

    return 0;
}

