#include "load_log.h"



LoadLog::LoadLog(const char* log_str)
{
	ReadFromData(log_str,this->log);
}

LoadLog::~LoadLog()
{
	
}

int LoadLog::ReadFromData(const char* logfile, vector<log_data>& logfile_data)
{
	ifstream file (logfile);
	string logline;
	log_data logdata_indv;

	if(file.is_open())
	{
		while(getline(file,logline))
		{
			istringstream iss(logline);
			char c;
			iss >> c;
			int j = 0;			
			switch(c)
			{
				case 'L':
					logdata_indv.data_type = 1;		//此行数据包括odom和lidar
					iss >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;	//读取机器人位姿,单位：cm;cm;rad
					iss >> logdata_indv.x_lidar >> logdata_indv.y_lidar >> logdata_indv.theta_lidar;	//读取激光雷达位姿,单位：cm;cm;rad
					logdata_indv.readings = (float*) malloc(sizeof(float) * total_readings);
					while (j < total_readings)
					{
						iss >> logdata_indv.readings[j];
						logdata_indv.readings[j] /= 10.0;    //lidar读数换算成dm,因为地图的分辨率是1dm,统一单位方便后面计算
						j++;
						// cout << logdata_indv.readings[j] << endl;
					}
					iss >> logdata_indv.time_stamp;	   //读取时间戳
					break;
				case 'O':	//此行数据只包括odom
					logdata_indv.data_type = 0;
					iss >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;
					iss >> logdata_indv.time_stamp;
					break;
				default:
					break;
			}
			logfile_data.push_back(logdata_indv);	//存入vector中
		}
	}
	else
	{
		fprintf(stderr, "ERROR: Could not open file %s\n", logfile);
		return -1;
	}
	file.close();	
}


vector<log_data> LoadLog::GetLog()
{
	return this->log;
}


void LoadLog::ShowLogData()
{
	log_data indv;
	for (int i = 0;i < this->log.size();i++)
	{
		indv = this->log[i];
		cout << indv.data_type << endl;
	}
}
