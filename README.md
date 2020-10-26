# trajectory_recorder

Record Manipulator trajectory (sensor_msgs::JointState) and Save in a .csv file.
Load Manipulator trajectory (sensor_msgs::JointState) from a .csv file.

## Running the tests

Launch Node:

```
roslaunch trajectory_recorder trajectory_recorder.launch
```

Start Recording:

```
rosservice call /trajectory_recorder/start_registration_service "message_data: 'output_file_name'" 
```

Stop Recording:

```
rosservice call /trajectory_recorder/stop_registration_service
```

Load Records:

```
rosservice call /trajectory_recorder/load_trajectory_service "message_data: 'input_file_name'"
```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**
