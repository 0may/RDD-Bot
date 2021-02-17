

rosservice call /evo_robot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['roller_bl_controller','roller_br_controller','roller_fl_controller','roller_fr_controller'], strictness: 2, start_asap: true, timeout: 1.0}" 

rosservice call /evo_robot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['wheel_bl_controller','wheel_br_controller','wheel_fl_controller','wheel_fr_controller'], strictness: 2, start_asap: true, timeout: 1.0}" 

rostopic pub --once /evo_robot/lift_fr_controller/command std_msgs/Float64 "data: -1005.0" & 
rostopic pub --once /evo_robot/lift_fl_controller/command std_msgs/Float64 "data: -1005.0" & 
rostopic pub --once /evo_robot/lift_br_controller/command std_msgs/Float64 "data: -1005.0" & 
rostopic pub --once /evo_robot/lift_bl_controller/command std_msgs/Float64 "data: -1005.0" &

sleep 5

rostopic pub --once /evo_robot/lift_fr_controller/command std_msgs/Float64 "data: +1005.0" & 
rostopic pub --once /evo_robot/lift_fl_controller/command std_msgs/Float64 "data: +1005.0" & 
rostopic pub --once /evo_robot/lift_br_controller/command std_msgs/Float64 "data: +1005.0" & 
rostopic pub --once /evo_robot/lift_bl_controller/command std_msgs/Float64 "data: +1005.0" &

sleep 5

rostopic pub --once /evo_robot/lift_fr_controller/command std_msgs/Float64 "data: +0.0" & 
rostopic pub --once /evo_robot/lift_fl_controller/command std_msgs/Float64 "data: +0.0" & 
rostopic pub --once /evo_robot/lift_br_controller/command std_msgs/Float64 "data: -0.0" & 
rostopic pub --once /evo_robot/lift_bl_controller/command std_msgs/Float64 "data: -0.0" &

rosservice call /evo_robot/controller_manager/switch_controller "{start_controllers: ['wheel_bl_controller','wheel_br_controller','wheel_fl_controller','wheel_fr_controller'], stop_controllers: [], strictness: 2, start_asap: true, timeout: 1.0}" 
