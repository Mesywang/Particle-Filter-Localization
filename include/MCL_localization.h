#ifndef MCL_LOCALIZATION_H
#define MCL_LOCALIZATION_H

#include <ros/ros.h>
#include <vector>
#include <unistd.h>
#include "load_map.h"
#include "load_log.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>


#define   PARTICLES_NUM         3000
#define       pi            3.1415926536
#define    ODOM_DATA             0
#define    LIDAR_DATA            1
#define   LASER_BEAM_NUM        180


typedef struct  
{
	float x;
	float y;
	float theta;
}robot_state;


typedef struct  
{
	float x;
	float y;
	float theta;
	float weight;
}particle_state;


typedef struct 
{
	float x_front;
	float y_front;
	float theta_front;
	float x_rear;
	float y_rear;
	float theta_rear;
}motion_model;


typedef struct 
{
	float* readings;
}lidar_measure;


class PFLocalization
{
	public:
	
        PFLocalization(map_type* mapdata,vector<log_data>& logdata,ros::NodeHandle node);
        ~PFLocalization();
		void InitParticles();
		void MCLAlgorithm();
		void CalRobotPose();
		void Visualize();


    private:
        particle_state SampleMotionModelOdometry(particle_state particle);
        float SampleStandardNormalDistribution(float var);
        float MeasurementBeamModel(particle_state particle);
        float MeasurementScoreModel(particle_state particle);
        float ProbMeasurementHit(float zkt_star, float zkt);
        float ProbMeasurementShort(float zkt_star, float zkt);
        float ProbMeasurementMaxVal(float zkt);
        float ProbMeasurementRandVal(float zkt);
        void LowVarianceSampler();


        ros::Publisher publish_particlecloud_;
        ros::Publisher publish_pose_;
        ros::NodeHandle node_;
		map_type* map_;
		vector<log_data> log_data_;
		vector<particle_state> particles_;
        geometry_msgs::PoseArray particles_ros_;
        geometry_msgs::PoseStamped robot_ros_;
        robot_state robot_pose_;
        motion_model motion_;
        lidar_measure measurement_;

        float alpha1_,alpha2_,alpha3_,alpha4_;
        float sigmahit_, lambdashort_;
        float z_hit_,z_short_,z_rand_,z_max_;
        float threshold_, map_threshold_, obstacle_threshold_, lidar_offset_;
        int numParticles_, ray_tracing_step_, resolution_, lidar_range_max_;
        
};


#endif


