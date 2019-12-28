#include "MCL_localization.h"


using namespace std;

//PFLocalization类的构造函数
PFLocalization::PFLocalization(map_type* mapdata,vector<log_data>& logdata,ros::NodeHandle node)
{

    this->map_ = mapdata;					//地图
    this->log_data_ = logdata;				//里程计和雷达数据
    this->numParticles_ = PARTICLES_NUM;	//粒子数
    this->map_threshold_ = 0.95;			//初始撒随机粒子时的地图阈值，保证粒子都在地图有效区域内
	this->node_ = node;						//ROS句柄
	this->resolution_ = 10;    				//地图分辨率10cm
	this->lidar_offset_ = 25;				//激光雷达安装偏移量25cm
	this->lidar_range_max_ = 1000; 			//雷达最大有效数据1000cm
	this->ray_tracing_step_ = 1;			//ray tracing步长
	this->obstacle_threshold_ = 0.2;		//地图概率小于此值则认为是障碍物


	this->alpha1_ = 0.025;					//里程计运动模型参数--旋转
	this->alpha2_ = 0.025;					//里程计运动模型参数--旋转
	this->alpha3_ = 0.4;					//里程计运动模型参数--平移
	this->alpha4_ = 0.4;					//里程计运动模型参数--平移


	this->sigmahit_ = 2;					//BeamModel中高斯分布的方差
	this->lambdashort_ = 1.5;				//BeamModel中指数分布的参数

	this->z_hit_ = 0.8;						//BeamModel中高斯分布Phit的权重
	this->z_short_ = 0.2;					//BeamModel中指数分布Pshort的权重
	this->z_rand_ = 0.0;					//BeamModel中均匀分布Prand的权重
	this->z_max_ = 0.0;						//BeamModel中均匀分布Pmax的权重

	//发布粒子集位姿
	publish_particlecloud_ = node_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
	this->particles_ros_.header.stamp = ros::Time::now();
	this->particles_ros_.header.frame_id = "map";
	this->particles_ros_.poses.resize(numParticles_);

	//发布机器人位姿
	publish_pose_ = node_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, true);
	this->robot_ros_.header.stamp = ros::Time::now();
	this->robot_ros_.header.frame_id = "map";
}

//PFLocalization类的析构函数
PFLocalization::~PFLocalization()
{
    
}


//粒子集初始化函数
void PFLocalization::InitParticles()
{
	int count = 1;
	while (count <= numParticles_)
	{
		particle_state particle_temp;

		particle_temp.x =  rand() / (float)RAND_MAX * (map_->max_x - map_->min_x) + map_->min_x; //初始化粒子X坐标
		particle_temp.y = rand() / (float)RAND_MAX * (map_->max_y - map_->min_y) + map_->min_y;  //初始化粒子Y坐标

		// particle_temp.x =  rand() / (float)RAND_MAX * (500 - 340) + 340; //初始化粒子X坐标
		// particle_temp.y = rand() / (float)RAND_MAX * (430 - map_->min_y) + map_->min_y;  //初始化粒子Y坐标

		if (map_->prob[(int) particle_temp.x][(int) particle_temp.y] <= map_threshold_)  //若随机出的粒子不在地图有效范围内,则重新生成粒子
			continue;

		count++;	//当有效粒子数 = 粒子总数时，跳出循环

		particle_temp.theta = rand() / (float)RAND_MAX * 2 * pi;  //初始化粒子角度theta 
		// particle_temp.theta = rand() / (float)RAND_MAX * (1/4 * pi - (-5/4 * pi)) - 5/4 * pi ;  //初始化粒子角度theta 

		//将粒子角度转换到 -pi ～ pi 之间
        if(particle_temp.theta > pi)
            particle_temp.theta -= 2 * pi;
        if(particle_temp.theta < -pi)
            particle_temp.theta += 2 * pi; 
		
		particle_temp.weight = 1.0/numParticles_;   // 初始化粒子权重为1/NUM

		particles_.push_back(particle_temp);   //存入粒子集
	}

	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	for(int i = 0; i < numParticles_; i++)
	{
		q = tf::createQuaternionMsgFromRollPitchYaw(0,0,particles_[i].theta);	//单位：rad
		pose_ros.position.x = 0.1 * particles_[i].x;	//单位：dm -> m
		pose_ros.position.y = 0.1 * particles_[i].y;
		pose_ros.position.z = 0.0;
		pose_ros.orientation = q;

		particles_ros_.poses[i] = pose_ros;
	}
	publish_particlecloud_.publish(particles_ros_);	  //发布粒子集状态
}


//蒙特卡洛定位算法函数
void PFLocalization::MCLAlgorithm()
{
	particle_state particle_state_temp;
	vector<particle_state> particles_temp;
	particles_temp.resize(numParticles_);
	
	for (int i = 1;i < log_data_.size(); i++)	//此层循环遍历数据集
	{	
		//取数据集中相邻两帧数据，作为里程计运动模型的相邻两次状态
		motion_.x_front = log_data_[i-1].x_robot;
		motion_.y_front = log_data_[i-1].y_robot;
		motion_.theta_front = log_data_[i-1].theta_robot;
		motion_.x_rear = log_data_[i].x_robot;
		motion_.y_rear =log_data_[i].y_robot;
		motion_.theta_rear = log_data_[i].theta_robot;	

		for(int j = 0; j < numParticles_; j++)
		{
			particle_state_temp = SampleMotionModelOdometry(particles_[j]);	  //从里程计运动模型中采样
			// particles_temp.push_back(particle_state_temp);
			particles_temp[j] = particle_state_temp;
		}

		// cout << "Reading (" << i << "/" << log_data_.size() << ") log data! ";
		// cout << "Robot Position (X,Y,theta): (" << log_data_[i].x_robot << "," << log_data_[i].y_robot << "," << log_data_[i].theta_robot << ")" << std::endl;

		particles_ = particles_temp;

		if(log_data_[i].data_type == ODOM_DATA)	  //此帧数据只包括odometry
		{
			Visualize();	//显示实时更新的粒子集状态

			continue;
		}
		else if(log_data_[i].data_type == LIDAR_DATA)	//此帧数据包括lidar
		{
			measurement_.readings = log_data_[i].readings;   //当前帧雷达数据首地址

			double total_weight = 0;
			for(int j = 0; j < numParticles_; j++)
			{
				// float weight = MeasurementBeamModel(particles_[j]);	  //根据光束模型计算每个粒子的权重，运行速度太慢且定位效果不好
				float weight = MeasurementScoreModel(particles_[j]);	  //根据似然法计算每个粒子的权重
				// cout << " each weight is : " << weight << endl;
				particles_[j].weight = weight;
				total_weight += particles_[j].weight;	  //粒子权重总和
			}

			for(int j = 0; j < numParticles_; j++)
			{
				particles_[j].weight /= total_weight;  //所有粒子权重归一化
				// avg_weight += particles_[j].weight/numParticles_;	
				// cout << " Normalized weight is : " << particles_[j].weight << endl;
			}

			float avg_weight = total_weight / numParticles_;
			// cout << " The average weight is : " << avg_weight << endl;

			LowVarianceSampler();		//低方差重采样算法

			CalRobotPose();		//计算机器人位姿，并发布topic
		}
		else
		{
			cout << " ERROR：The Log Data Is Error!!!" << endl;
		}

		Visualize();	//显示实时更新的粒子集状态

		// sleep(1);		 //1s

		if(i <= 20)
			usleep(300000);   //前20帧数据延迟大一些，更清晰的观察粒子的重采样情况
		else
			usleep(40000); 
	}

}


//概率机器人P103---基于里程计运动模型中采样算法	
particle_state PFLocalization::SampleMotionModelOdometry(particle_state particle)
{
	float deltarot1 = atan2(motion_.y_rear - motion_.y_front,motion_.x_rear - motion_.x_front) - motion_.theta_rear;
	float deltatrans1 = sqrt(pow((motion_.x_rear - motion_.x_front),2) + pow((motion_.y_rear - motion_.y_front),2));
	float deltarot2 = motion_.theta_rear - motion_.theta_front - deltarot1;

	float deltarot1_hat = deltarot1 - SampleStandardNormalDistribution(alpha1_*deltarot1 + alpha2_*deltatrans1);
	float deltatrans1_hat  = deltatrans1 - SampleStandardNormalDistribution(alpha3_*deltatrans1 + alpha4_*(deltarot1 + deltarot2));
	float deltarot2_hat  = deltarot2 - SampleStandardNormalDistribution(alpha1_*deltarot2 + alpha2_*deltatrans1);

	particle_state particle_temp;
	//地图是以dm为单位,初始化的粒子位置是基于地图生成的,所以也是dm单位,而里程计数据单位是cm,需在这里进行单位转换
	particle_temp.x = particle.x + (deltatrans1_hat * cos(particle.theta + deltarot1_hat)) / resolution_;
	particle_temp.y = particle.y + (deltatrans1_hat * sin(particle.theta + deltarot1_hat)) / resolution_;
	particle_temp.theta = particle.theta + deltarot1_hat + deltarot2_hat;
	particle_temp.weight = particle.weight;

	return particle_temp;
}


//从标准正态分布中采样
float PFLocalization::SampleStandardNormalDistribution(float var)
{
	float sum = 0;
	for (int i = 0;i < 12; i++)
		//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
		sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
	return (var / 6.0) * sum;
}



//根据光束模型计算每个粒子的权重
float PFLocalization::MeasurementBeamModel(particle_state particle)
{
	double q = 1,p = 1;
	robot_state lidar_pose;

	float step_x,step_y,zkt_star = 0, zkt = 0;

    //计算激光雷达在地图坐标系下的位姿
	lidar_pose.x = particle.x + (lidar_offset_ * cos(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.y = particle.y + (lidar_offset_ * sin(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.theta = particle.theta;

	//若雷达位姿在地图有效区域外,则终止此次计算,该粒子权重为0
	if(map_->prob[(int)lidar_pose.x][(int)lidar_pose.y] <= map_threshold_)
		return 0.0;
		
	for (int i = 0; i < LASER_BEAM_NUM; i++)
	{
		zkt = measurement_.readings[i];		//单位：dm,从log文件读取时就已经转换成dm了
		
		//此光束无效
		if (zkt > (lidar_range_max_ / resolution_))	  
			continue;

		//计算第i个激光束在世界坐标系下的角度
		float step_theta = ((double)i / 180.0) * pi + lidar_pose.theta;

		/******************************* Ray Tracing **********************************/
		int step = 1;
		int invaild_flag = 0;
		while(1)
		{
		    //减90度是因为激光雷达第一个激光点角度与机器人坐标系相差90度
			step_x = lidar_pose.x + step * cos( (step_theta - pi/2.0));
			step_y = lidar_pose.y + step * sin( (step_theta - pi/2.0));

			//若激光束打到地图以外区域,则此激光束权值为0
			if(step_x >= map_->max_x || step_y >= map_->max_y || step_x < map_->min_x || step_y < map_->min_y || map_->prob[(int)step_x][(int)step_y] < 0)
			{
				invaild_flag = 1;
				break;
			}
			else if(map_->prob[(int)step_x][(int)step_y] <= obstacle_threshold_)	//遇到障碍
			{
				zkt_star = step;	//Ray Tracing的结果,单位：dm
				break;
			}

			step += ray_tracing_step_;
		}
		if(invaild_flag == 1)
			continue;

		p = z_hit_ * ProbMeasurementHit(zkt_star,zkt) + z_short_ * ProbMeasurementShort(zkt_star,zkt) 
			    + z_max_ * ProbMeasurementMaxVal(zkt) + z_rand_ * ProbMeasurementRandVal(zkt);

		// q *= p;	   //累积乘法会导致q趋于无穷小
		q += p;	

		/******************************************************************************/
	}
	return q;
}


//计算高斯分布函数的概率
float PFLocalization::ProbMeasurementHit(float zkt_star, float zkt)
{
	if (zkt < 0 || zkt > (lidar_range_max_ / resolution_))
		return 0;
	else
	{
		float q;
		q = (1.0 / sqrt(2*pi*sigmahit_*sigmahit_)) * exp((-1/2*((zkt - zkt_star)*(zkt - zkt_star)))/(sigmahit_*sigmahit_));
		return q;	
	}
}


//计算指数分布函数的概率
float PFLocalization::ProbMeasurementShort(float zkt_star, float zkt)
{
	if(zkt < 0 || zkt < zkt_star)
		return 0;
	else
	{
		float q,eeta;
		eeta = 1 / (1 - exp(-1.0 * lambdashort_ * zkt_star));
		q = eeta * lambdashort_ * exp(-1.0 * lambdashort_ * zkt);
		return q;
	}
}


//计算均匀分布函数的概率
float PFLocalization::ProbMeasurementRandVal(float zkt)
{
	if(zkt < 0 || zkt >= (lidar_range_max_ / resolution_))
		return 0;
	else
		return 1.0 / (lidar_range_max_ / resolution_);
}


//计算是否为测量最大值的概率
float PFLocalization::ProbMeasurementMaxVal(float zkt)
{
	if(zkt == (lidar_range_max_ / resolution_))
		return 1;
	else
		return 0;
}


//根据得分模型计算每个粒子的权重
float PFLocalization::MeasurementScoreModel(particle_state particle)
{
	robot_state lidar_pose;
	float laser_end_x,laser_end_y,score = 0, zkt = 0;

    //计算激光雷达在map坐标系下的位姿
	lidar_pose.x = particle.x + (lidar_offset_ * cos(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.y = particle.y + (lidar_offset_ * sin(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.theta = particle.theta;

	//若雷达位姿在地图有效区域外,则终止此次计算,该粒子权重为0
	if(map_->prob[(int)lidar_pose.x][(int)lidar_pose.y] <= map_threshold_)   
		return 0.0;  
		
	for (int i = 0; i < LASER_BEAM_NUM; i++)
	{
		zkt = measurement_.readings[i];		//第i个激光束的测距，单位：dm,从log文件读取时就已经转换成dm了
		
		//若超出设置的lidar最大有效值，则此光束无效
		if (zkt > (lidar_range_max_ / resolution_))	  
			continue;

		//计算第i个激光束在世界坐标系下的角度
		float step_theta = ((double)i / 180.0) * pi + lidar_pose.theta - pi/2.0;

		laser_end_x = lidar_pose.x + zkt * cos(step_theta); 	//计算此激光束末端在map坐标系下的X坐标
		laser_end_y = lidar_pose.y + zkt * sin(step_theta); 	//计算此激光束末端在map坐标系下的Y坐标

		//若激光束末端在地图未知区域或无效区域，则跳过此次得分计算
		if(laser_end_x >= map_->max_x || laser_end_y >= map_->max_y || laser_end_x < map_->min_x || laser_end_y < map_->min_y 
		  															   || map_->prob[(int)laser_end_x][(int)laser_end_y] < 0)
		   continue;
		
		// if(map_->prob[(int)laser_end_x][(int)laser_end_y] >= 0 && map_->prob[(int)laser_end_x][(int)laser_end_y] < 0.15)
		// 	score++;

		score += map_->prob[(int)laser_end_x][(int)laser_end_y] < 0.15 ? 1 : 0; //累加，计算此帧lidar数据的得分
	}

	return score;	//返回当前帧lidar数据的得分，用此来表示粒子权重
}



//概率机器人P78---低方差重采样算法
void PFLocalization::LowVarianceSampler()
{
	vector<particle_state> particles_temp = particles_;
	float r = (rand() / (float)RAND_MAX) * (1.0 / (float)numParticles_); //初始化一个0~1之间的随机数
	float c = particles_[0].weight;
	int i = 0;

	for (int m = 0;m < numParticles_; m++)
	{
		float u = r + (float) m/ numParticles_; 		//移动随机数
		while (u > c && i < numParticles_ - 1)
		{ 
			i++;										//移动到下一个粒子
			c += particles_temp[i].weight;	
		}
		particles_[m] = particles_temp[i]; 	 			//复制被选择的粒子
		particles_[m].weight = 1.0 / numParticles_;		//重采样后粒子权重重置
		// cout << " each weight is : " << particles_[m].weight << endl;
	}	
}


//计算机器人位姿，并发布状态
void PFLocalization::CalRobotPose()
{
	float total_x = 0.0;
	float total_y = 0.0;
	float total_theta = 0.0;

	for(int i = 0; i < numParticles_; i++)
	{
		total_x += particles_[i].x;
		total_y += particles_[i].y;
		total_theta += particles_[i].theta;
	}	
	robot_pose_.x = total_x / numParticles_;
	robot_pose_.y = total_y / numParticles_;
	robot_pose_.theta = total_theta / numParticles_;

	cout << "Robot pose:  X: " << robot_pose_.x << ", Y: " << robot_pose_.y << ", Theta: " << robot_pose_.theta << endl;

	//显示机器人位姿
	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	q = tf::createQuaternionMsgFromRollPitchYaw(0,0,robot_pose_.theta);
	pose_ros.position.x = 0.1 * robot_pose_.x;	//单位：dm -> m
	pose_ros.position.y = 0.1 * robot_pose_.y;
	pose_ros.position.z = 0.0;
	pose_ros.orientation = q;
	robot_ros_.pose = pose_ros;

	publish_pose_.publish(robot_ros_);	//发布机器人状态


	static tf::TransformBroadcaster br;
  	tf::Transform transform;
 	transform.setOrigin( tf::Vector3(0.1 * robot_pose_.x, 0.1 * robot_pose_.y, 0.0) );
  	tf::Quaternion tf_q;
	tf_q.setRPY(0, 0, robot_pose_.theta);
  	transform.setRotation(tf_q);
 	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));	//发布base_link和map的tf关系

}


//显示实时更新的粒子集状态
void PFLocalization::Visualize()
{
	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	for(int i = 0; i < numParticles_; i++)
	{
		q = tf::createQuaternionMsgFromRollPitchYaw(0,0,particles_[i].theta);
		pose_ros.position.x = 0.1 * particles_[i].x;	//单位：dm -> m
		pose_ros.position.y = 0.1 * particles_[i].y;
		pose_ros.position.z = 0.0;
		pose_ros.orientation = q;

		particles_ros_.poses[i] = pose_ros;
	}
	publish_particlecloud_.publish(particles_ros_);	  //发布粒子集状态
}