#include "trajectory_recorder/trajectory_recorder.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

trajectory_recorder_class::trajectory_recorder_class( 
    ros::NodeHandle &n, ros::Rate ros_rate, 
    std::string topic_joint_states_subscriber):

    nh(n), loop_rate(ros_rate) {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/trajectory_recorder_Node/use_ur_real_robot", use_ur_real_robot, false)) {ROS_ERROR("Couldn't retrieve the Use Real Robot value.");}

    // ---- ROS SUBSCRIBERS ---- //
    joint_states_subscriber = nh.subscribe(topic_joint_states_subscriber, 1, &trajectory_recorder_class::joint_states_Callback, this);

    // ---- ROS SERVICES ---- //
    start_registration_service = nh.advertiseService("/trajectory_recorder/start_registration_service", &trajectory_recorder_class::Start_Registration_Service_Callback, this);
    stop_registration_service = nh.advertiseService("/trajectory_recorder/stop_registration_service", &trajectory_recorder_class::Stop_Registration_Service_Callback, this);
    load_trajectory_service = nh.advertiseService("/trajectory_recorder/load_trajectory_service", &trajectory_recorder_class::Load_Trajectory_Service_Callback, this);

    // Initializing the Class Variables
    start_registration = false;
    load_registration = false;
    new_data_received = false;

}

trajectory_recorder_class::~trajectory_recorder_class() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void trajectory_recorder_class::joint_states_Callback (const sensor_msgs::JointState::ConstPtr &msg) {

    joint_state = *msg;
    new_data_received = true;

    // Ur10e Real Robot has Inverted Joints
    if (use_ur_real_robot) {

        std::swap(joint_state.name[0], joint_state.name[2]);
        std::swap(joint_state.effort[0], joint_state.effort[2]);
        std::swap(joint_state.position[0], joint_state.position[2]);
        std::swap(joint_state.velocity[0], joint_state.velocity[2]);

    }

    ROS_DEBUG_THROTTLE(2, "joint position: %.2f %.2f %.2f %.2f %.2f %.2f", joint_state.position[0], joint_state.position[1], joint_state.position[2], joint_state.position[3], joint_state.position[4], joint_state.position[5]);
    ROS_DEBUG_THROTTLE(2, "joint velocity: %.2f %.2f %.2f %.2f %.2f %.2f", joint_state.velocity[0], joint_state.velocity[1], joint_state.velocity[2], joint_state.velocity[3], joint_state.velocity[4], joint_state.velocity[5]);
    
}

bool trajectory_recorder_class::Start_Registration_Service_Callback (trajectory_recorder::String::Request &req, trajectory_recorder::String::Response &res) {
    
    start_registration = true;
    output_file = req.message_data;
    
    res.success = true;
    return true;

}

bool trajectory_recorder_class::Stop_Registration_Service_Callback (std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    start_registration = false;
    
    res.success = true;
    return true;

}

bool trajectory_recorder_class::Load_Trajectory_Service_Callback (trajectory_recorder::String::Request &req, trajectory_recorder::String::Response &res) {

    load_registration = true;
    input_file = req.message_data;
    
    res.success = true;
    return true;

}


//----------------------------------------------------- FUNCTIONS ------------------------------------------------------//


void trajectory_recorder_class::record_trajectory (std::string output_csv) {

    ROS_INFO("Start Recording");

    std::vector<sensor_msgs::JointState> trajectory;
    trajectory.clear();

    auto start_point = std::chrono::system_clock::now();
    std::time_t start = std::chrono::system_clock::to_time_t(start_point);

    while (nh.ok() && start_registration) {

        // Callback Read "joint_state" Data from "/joint_states" Topic
        ros::spinOnce();

        // Append New Data
        if (new_data_received && start_registration) {
        
            trajectory.push_back(joint_state);
            new_data_received = false;
        
        }

    }

    ROS_INFO("Stop Recording");

    auto stop_point = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = stop_point - start_point;

    save_trajectory(output_csv, trajectory, start, duration);

}

void trajectory_recorder_class::save_trajectory (std::string output_csv, std::vector<sensor_msgs::JointState> trajectory, std::time_t start_time, std::chrono::duration<double> elapsed_time) {

    // Rename Output File when is Empty
    if (output_csv == "") {output_csv = "trajectory_log_default";}

    // Get ROS Package Path
    std::string package_path = ros::package::getPath("trajectory_recorder");
    ROS_INFO_STREAM_ONCE("Package Path:  " << package_path);
    std::string save_file = package_path + "/records/" + output_csv + ".csv";

    // OfStream Creation
    std::ofstream save(save_file);
    ROS_WARN_STREAM("Output File:  " << output_csv << ".csv");

    // Get Date and Time
    std::string date_time = std::string(std::ctime(&start_time));
    date_time.erase(date_time.size()-1);
    // for(char& c : date_time) {std::cout << c;} std::cout << std::endl;

    // First Row: time saving description
    save << "Trajectory Recorded at ,";
    for(char& c : date_time) {save << c;};
    save << ",Duration: " << elapsed_time.count() << "s\n\n";

    // Second Row: position or velocity
    save << "Position,Position,Position,Position,Position,Position,Velocity,Velocity,Velocity,Velocity,Velocity,Velocity\n";

    // Third Row: joint name (twice)
    for (unsigned int i = 0; i < 12; i++) {save << trajectory[0].name[i%6] << ",";}
    save << "\n";

    // Other Rows: joint positions and velocities
    for (unsigned int i = 0; i < trajectory.size(); i++) {

        // Joint Positions
        for (unsigned int j = 0; j < trajectory[i].position.size(); j++) {save << trajectory[i].position[j] << ",";}

        // Joint Velocities
        for (unsigned int j = 0; j < trajectory[i].velocity.size(); j++) {save << trajectory[i].velocity[j] << ",";}

        save << "\n";

    }

    save.close();

    ROS_INFO("Trajectory Saved Succesfully");

}

std::vector<sensor_msgs::JointState> trajectory_recorder_class::load_trajectory (std::string input_csv) {

    load_registration = false;


}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void trajectory_recorder_class::spinner (void) {

    ros::spinOnce();

    if (start_registration) {record_trajectory(output_file);}
    else if (load_registration) {load_trajectory(input_file);}
    
}
