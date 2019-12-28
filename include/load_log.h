#ifndef LOAD_LOG_H
#define LOAD_LOG_H


#include "load_map.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#define total_readings 180

using namespace std;


typedef struct 
{
	int data_type; 							// 0-里程计数据,1-激光雷达数据
	float x_robot,y_robot,theta_robot; 		// 机器人位姿
	float x_lidar,y_lidar,theta_lidar; 		// 激光雷达位姿
	float* readings; 						// 激光雷达读数
	float time_stamp;						// 时间戳
}log_data;


class LoadLog
{
    public:
        LoadLog(const char* log_str);
		~LoadLog();
		int ReadFromData(const char* logfile, vector<log_data>& logfile_data);
		vector<log_data> GetLog();
		void ShowLogData();

    private:
		vector<log_data> log;
};


#endif