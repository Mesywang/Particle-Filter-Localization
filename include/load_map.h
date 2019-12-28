#ifndef LOAD_MAP_H
#define LOAD_MAP_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"



typedef struct
{
	int resolution;					//地图分辨率
	int size_x,size_y;				//地图尺寸
	int min_x,min_y,max_x,max_y;	//地图有效区域
	float offset_x, offset_y;		//地图偏移量
	float** prob;					//地图概率值
} map_type;


class LoadMap
{
    public:
        LoadMap(const char* map_str);	 			//LoadMap类构造函数
		~LoadMap();									//LoadMap类析构函数
		int ReadFromData(const char* map_name);	    //从wean.data中读取数据	
		void PublishMap(ros::NodeHandle node_);		//将地图转换格式,并发布topic在rviz中显示
		map_type* GetMap();							//将地图传递到类外

    private:
		map_type* map;
   	 	ros::Publisher map_pub;
   		ros::Publisher map_meta_pub;
};

#endif


