/********************************************/
/* ROS command to change the state machine: */
/********************************************/
// start
rosservice call /hdpc_com/changeState 1
// stop
rosservice call /hdpc_com/changeState 2
// reset
rosservice call /hdpc_com/changeState 3


/********************************************/
/* ROS command to send commands		    */
/********************************************/
// deactivate all
rostopic pub /hdpc_com/commands hdpc_com/Commands -r 10 "isActive: [False, False, False, False, False, False, False, False, False, False]
command: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

// 0 position, 0 velocity
rostopic pub /hdpc_com/commands hdpc_com/Commands -r 10 "isActive: [True, True, True, True, True, True, True, True, True, True]
command: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

// 0 position, 7 velocity
rostopic pub /hdpc_com/commands hdpc_com/Commands -r 10 "isActive: [True, True, True, True, True, True, True, True, True, True]
command: [7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 0.0, 0.0, 0.0, 0.0]"

// +90deg position,  1 rad/s velocity
rostopic pub /hdpc_com/commands hdpc_com/Commands -r 10 "isActive: [True, True, True, True, True, True, True, True, True, True]
command: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.57, 1.57, 1.57, 1.57]"

// -90deg position,  -1 rad/s velocity
rostopic pub /hdpc_com/commands hdpc_com/Commands -r 10 "isActive: [True, True, True, True, True, True, True, True, True, True]
command: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.57, -1.57, -1.57, -1.57]"
