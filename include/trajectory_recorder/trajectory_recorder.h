#ifndef TRAJECTORY_RECORDER_H
#define TRAJECTORY_RECORDER_H

#include <fstream>
#include <chrono>
#include <ctime>

#include "ros/ros.h"
#include "ros/package.h"

#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "std_srvs/Trigger.h"
#include "trajectory_recorder/String.h"
#include "trajectory_recorder/Load_Trajectory.h"

struct loaded_trajectory {
    std::string time_description;
    std::vector<sensor_msgs::JointState> trajectory;
};

class trajectory_recorder_class {

    public:

        trajectory_recorder_class( 
            ros::NodeHandle &n, ros::Rate ros_rate,
            std::string topic_joint_states_subscriber);

        ~trajectory_recorder_class();

        void spinner (void);
        
    private:

        ros::NodeHandle nh;
        ros::Rate loop_rate;

        ros::Subscriber joint_states_subscriber;

        ros::ServiceServer start_registration_service, stop_registration_service, load_trajectory_service;

        bool use_ur_real_robot;
        bool start_registration, new_data_received;

        sensor_msgs::JointState joint_state;
        std::string output_file;

        void joint_states_Callback (const sensor_msgs::JointState::ConstPtr &);

        bool Start_Registration_Service_Callback (trajectory_recorder::String::Request &req, trajectory_recorder::String::Response &res);
        bool Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool Load_Trajectory_Service_Callback (trajectory_recorder::Load_Trajectory::Request &req, trajectory_recorder::Load_Trajectory::Response &res);

        void record_trajectory (std::string output_csv);
        void save_trajectory (std::string output_csv, std::vector<sensor_msgs::JointState>, std::time_t start_time, std::chrono::duration<double> elapsed_time);
        loaded_trajectory load_trajectory (std::string input_csv);

};

#endif /* TRAJECTORY_RECORDER_H */
