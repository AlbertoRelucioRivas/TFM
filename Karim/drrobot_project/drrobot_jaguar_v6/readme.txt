In Terminal-1 window
roscore

In terminal-2 window
cd catkin_ws
source ./devel/setup.bash

catkin_make
rosrun drrobot_jaguar_v6 drrobot_jaguar_v6_node

In Terminal-3 window
cd catkin_ws
source ./devel/setup.bash

rostopic list

#test published sensor message
rostopic echo drrobot_jaguar_v6_gps_sensor
rostopic echo drrobot_jaguar_v6_imu_sensor
rostopic echo drrobot_jaguar_v6_motor_sensor
rostopic echo drrobot_jaguar_v6_motorboard_sensor

#test receive control command
rostopic pub /drrobot_jaguar_v6_basemotor_cmd drrobot_jaguar_v6/BaseMotorCmd -- 200 -200
rostopic pub /drrobot_jaguar_v6_flipmotor_cmd drrobot_jaguar_v6/FlipMotorCmd -- 30 -30 30 -30




