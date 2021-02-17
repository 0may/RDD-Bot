
rostopic pub --once /evo_robot/wheel_fr_controller/command std_msgs/Float64 "data: -0.0" & 
rostopic pub --once /evo_robot/wheel_fl_controller/command std_msgs/Float64 "data: +0.0" & 
rostopic pub --once /evo_robot/wheel_br_controller/command std_msgs/Float64 "data: -0.0" & 
rostopic pub --once /evo_robot/wheel_bl_controller/command std_msgs/Float64 "data: +0.0" &
