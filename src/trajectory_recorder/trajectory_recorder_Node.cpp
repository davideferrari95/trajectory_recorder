#include "trajectory_recorder/trajectory_recorder.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "trajectory_recorder_Node");

    ros::NodeHandle nh;
    ros::Rate loop_rate = 1000;

    // Parameters
    std::string topic_joint_states_subscriber;
    std::string csv_output_file, csv_input_file;

    // ---- LOADING "TOPIC NAME" PARAMETERS FROM THE ROS SERVER ---- //
    if (!nh.param<std::string>("/trajectory_recorder_Node/topic_joint_states", topic_joint_states_subscriber, "/joint_states")) {ROS_ERROR("Couldn't retrieve the Joint States Topic's name.");}
    
    trajectory_recorder_class tr (nh, loop_rate, topic_joint_states_subscriber);

    while (ros::ok()) {

        tr.spinner();

    }

return 0;

}
