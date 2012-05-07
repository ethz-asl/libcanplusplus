#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <hdpc_com/ChangeStateMachine.h>
#include <hdpc_com/HDPCStateMachineEnums.h>

#include <hdpc_drive/SetControlMode.h>
#include <hdpc_drive/Status.h>
#include <hdpc_drive/DirectDrive.h>
#include <hdpc_com/HDPCGeometry.h>
#include <hdpc_com/HDPCConst.h>

using namespace hdpc_com;
using namespace hdpc_drive;

class HDPCController
{
    protected:
        sensor_msgs::Joy joystate;
        hdpc_drive::Status state;
        ros::Subscriber state_sub;
        ros::Subscriber joy_sub;
        ros::Publisher control_pub;
        ros::Publisher directdrive_pub;
        ros::ServiceClient setModeClt;
        ros::ServiceClient state_machine_client;

        hdpc_com::HDPCGeometry geom;
        hdpc_drive::DirectDrive ddmsg;
        double elevation_boundary_rad;
        double max_linear_velocity;
        double max_rotational_velocity;
        double initial_const_velocity_m_s;
        double const_velocity_increment_m_s;
        double dd_wheel_velocity_rad_s;
        double dd_steering_increment_rad;
        double dd_steering_actual_value_rad;

        bool firstctrl,gotjoy;
    public:
        HDPCController(ros::NodeHandle & nh) : geom(nh) {
            firstctrl = true;
            gotjoy = false;
            joystate.buttons.resize(12);
            joystate.axes.resize(12);
            elevation_boundary_rad = atan(geom.rover_width/2.);

            nh.param("max_linear_velocity",max_linear_velocity,0.5);
            nh.param("max_rotational_velocity",max_rotational_velocity,0.5);
            nh.param("initial_const_velocity_m_s",initial_const_velocity_m_s,0.0);
            nh.param("const_velocity_increment_m_s",const_velocity_increment_m_s,0.1);
            nh.param("dd_wheel_velocity_rad_s",dd_wheel_velocity_rad_s,0.0174533);
            nh.param("dd_steering_increment_rad",dd_steering_increment_rad,0.0174533);

            setModeClt = nh.serviceClient<hdpc_drive::SetControlMode>("/hdpc_drive/set_control_mode");
            state_machine_client = nh.serviceClient<hdpc_com::ChangeStateMachine>("/hdpc_com/changeState");

            // Subscribe to the state
            state_sub = nh.subscribe("/hdpc_drive/status",1,&HDPCController::stateCallback, this);
            // Publishing the control
            control_pub = nh.advertise<geometry_msgs::Twist>("/hdpc_drive/ackermann",1);
            directdrive_pub = nh.advertise<hdpc_drive::DirectDrive>("/hdpc_drive/direct",1);

            joy_sub = nh.subscribe("/joy",1,&HDPCController::joyCallback, this);

            ROS_INFO("HDPC Teleop initialised and ready to roll!");
        }
        ~HDPCController() {
        }

        void stateCallback(const hdpc_drive::Status::ConstPtr& msg) {
            state = *msg;
        }

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
            assert(msg->buttons.size()>4);
            assert(msg->axes.size()>=2);
            joystate = *msg;
            gotjoy = true;
        }

        void set_control_mode(unsigned int mode, double elevation=M_PI/2) {
            hdpc_drive::SetControlMode req;
            req.request.request_mode = mode;
            req.request.desired_elevation = elevation;
            if (!setModeClt.call(req)) {
                ROS_ERROR("Set control mode %d failed",mode);
            }
        }

        void send_reset() {
            hdpc_com::ChangeStateMachine change_state;
            change_state.request.event = EVENT_RESET;
            if (!state_machine_client.call(change_state)) {
                ROS_WARN("Change state machine: reset request failed");
            }
        }

        void setdd(hdpc_drive::DirectDrive & ddmsg, int axle, double value) {
            if (axle < 6) {
                ddmsg.velocities_rad_per_sec[axle] = value;//*dd_wheel_velocity_rad_s;
            } else if (axle < 10) {
                ddmsg.steering_rad[axle-6] = value;//*dd_steering_increment_rad;
            }
            directdrive_pub.publish(ddmsg);
        }

        void joyctrl() {
            ros::Rate looprate(10);
            unsigned int current_axle = 0;
            int prev_control_mode = -1;
            double rotation_elevation = 0.0;
            double tnow, last_dd_cmd = -1, last_reset_cmd = -5;
            double last_const_velocity_cmd = -1,last_velocity_incr_cmd = -1;
            bool is_const_velocity = false;
            double current_const_velocity_m_s = initial_const_velocity_m_s;
            double sw_start_time=0, sw_current_time=0;

            while (ros::ok()) {
                bool elevation_change = false;
                looprate.sleep();
                ros::spinOnce();
                tnow = ros::Time::now().toSec();
                if (!gotjoy) continue;
                if (joystate.buttons[5] && joystate.buttons[10] && ((tnow - last_reset_cmd)>5.0)) {
                    ROS_WARN("HDPC Teleop: trigerring reset");
                    send_reset();
                    set_control_mode(HDPCConst::MODE_INIT);
                    prev_control_mode = 1;
                    last_reset_cmd = tnow;
                    ROS_INFO("HDPC Teleop: wait until end of init before continuing");
                } else if (joystate.buttons[5] || joystate.buttons[10]) {
                    ROS_INFO("HDPC Teleop: press button 6 and 11 simultaneously to trigger a reset");
                }

                switch (state.control_mode) {
                    case HDPCConst::MODE_STOPPED:
                        if (prev_control_mode != state.control_mode) {
                            ROS_INFO("Entered in mode STOPPED");
                            prev_control_mode = state.control_mode;
                        }
                        if (joystate.buttons[1]) {
                            rotation_elevation = M_PI/2;
                            set_control_mode(HDPCConst::MODE_INIT_ACKERMANN, rotation_elevation);
                            ROS_INFO("Entering Ackermann mode.");
                            ROS_INFO("Press 0 to exit. Velocity: Axis 0. Steering: Axis 1");
                        } else if (joystate.buttons[2]) {
                            rotation_elevation = 0.0;
                            set_control_mode(HDPCConst::MODE_INIT_ROTATION, rotation_elevation);
                            ROS_INFO("Entering Rotation mode.");
                            ROS_INFO("Press 0 to exit. Rot. Velocity: Axis 1.");
                            ROS_INFO("ICR Positon: [-/0/+] wiht button [3/2/4]");
                        } else if (joystate.buttons[3]) {
                            ddmsg = hdpc_drive::DirectDrive();
                            last_dd_cmd = -1;
                            current_axle = 0;
                            set_control_mode(HDPCConst::MODE_DIRECT_DRIVE);
                            ROS_INFO("Entering Direct Drive mode.");
                            ROS_INFO("Press 0 to exit. Control: Axis 0. Select DoF with [3/4]");
                        }
                        break;
                    case HDPCConst::MODE_DIRECT_DRIVE:
                        if (prev_control_mode != state.control_mode) {
                            ROS_INFO("Entered in mode DIRECT DRIVE");
                            prev_control_mode = state.control_mode;
                            is_const_velocity=0;
                        }
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving DirectDrive mode");
                        } else if (joystate.buttons[3] && (tnow - last_dd_cmd > 0.5)) {
                            setdd(ddmsg,current_axle,0.0);
                            dd_steering_actual_value_rad = 0.0;
                            current_axle = (current_axle - 1) % 10;
                            last_dd_cmd = tnow;
                            ROS_INFO("DirectDrive: controlling DoF %d",current_axle);
                        } else if (joystate.buttons[4] && (tnow - last_dd_cmd > 0.5)) {
                            setdd(ddmsg,current_axle,0.0);
                            dd_steering_actual_value_rad = 0.0;
                            current_axle = (current_axle + 1) % 10;
                            last_dd_cmd = tnow;
                            ROS_INFO("DirectDrive: controlling DoF %d",current_axle);
                        } else if (joystate.buttons[6] && (tnow-last_const_velocity_cmd)>0.5){
                            ROS_INFO("Entering const velocity mode");
                            is_const_velocity = !is_const_velocity;
                            last_const_velocity_cmd = tnow;
                            sw_start_time = tnow;
                            }
                        else {
                        	if (current_axle < 6) {
                        		if (is_const_velocity){
                        			for (int i_wheel = 0; i_wheel < 6; i_wheel++){
                        				ddmsg.velocities_rad_per_sec[i_wheel] = dd_wheel_velocity_rad_s;
                        			}
                        			sw_current_time = tnow - sw_start_time;
                        			ROS_INFO("Current time: %f",sw_current_time);
                        			if (sw_current_time >= 180) {
                        				is_const_velocity = 0;
                        				for (int i_wheel = 0; i_wheel < 6; i_wheel++){
                        					ddmsg.velocities_rad_per_sec[i_wheel] = 0.0;
                        				}
                        				ROS_INFO("Leaving const velocity mode");
                        			}
                        		} else
                        			ddmsg.velocities_rad_per_sec[current_axle] = joystate.axes[1]*dd_wheel_velocity_rad_s;
                        		//sw_current_time = tnow-sw_start_time;
                        		//ROS_INFO("TIME: %f", sw_current_time);
                        	} else if (current_axle < 10) {
                        		if (joystate.buttons[8] && dd_steering_actual_value_rad < 1.58825 && (tnow - last_dd_cmd > 0.5)) {
                        			dd_steering_actual_value_rad += dd_steering_increment_rad;
                        			ROS_INFO("Actual commanded angle: %f",dd_steering_actual_value_rad*180/M_PI);
                        			last_dd_cmd = tnow;
                        		} else if (joystate.buttons[7] && dd_steering_actual_value_rad > -1.58825 && (tnow - last_dd_cmd > 0.5)) {
                        			dd_steering_actual_value_rad -= dd_steering_increment_rad;
                        			ROS_INFO("Actual commanded angle: %f",dd_steering_actual_value_rad*180/M_PI);
                        			last_dd_cmd = tnow;
                        		}
                        		ddmsg.steering_rad[current_axle-6] = dd_steering_actual_value_rad;
                        	}
                        	directdrive_pub.publish(ddmsg);
                            //setdd(ddmsg,current_axle,joystate.axes[1]);

                        }
                        break;
                    case HDPCConst::MODE_ACKERMANN:
                        if (prev_control_mode != state.control_mode) {
                            ROS_INFO("Entered in mode ACKERMANN");
                            prev_control_mode = state.control_mode;
                            is_const_velocity=0;
                        }
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving Ackermann mode");
                        } else {
                            geometry_msgs::Twist msg;
                            // rotation speed in [-0.5,0.5] m/s
                            // Check for triggered const velocity button
                            if (joystate.buttons[6] && (tnow-last_const_velocity_cmd)>0.5){
                            	if (is_const_velocity) ROS_INFO("Leaving const velocity mode");
                            	else ROS_INFO("Entering const velocity mode");
                            	is_const_velocity = !is_const_velocity;
                            	last_const_velocity_cmd = tnow;
                            	current_const_velocity_m_s = initial_const_velocity_m_s;
                            }

                            if (is_const_velocity){
                            	if (joystate.buttons[8] && (tnow-last_velocity_incr_cmd)>0.5){
                            		if (current_const_velocity_m_s <= max_linear_velocity-const_velocity_increment_m_s)
                            			current_const_velocity_m_s += const_velocity_increment_m_s;
                            		last_velocity_incr_cmd = tnow;
                            	}
                            	if (joystate.buttons[7]&& (tnow-last_velocity_incr_cmd)>0.5){
                            	    if (current_const_velocity_m_s >= -max_linear_velocity+const_velocity_increment_m_s)
                            	    	current_const_velocity_m_s -= const_velocity_increment_m_s;
                            	    last_velocity_incr_cmd = tnow;
                            	}
                            	msg.linear.x = current_const_velocity_m_s;
                            }
                            else
                            	msg.linear.x = joystate.axes[1] * max_linear_velocity;

                            if (joystate.buttons[3]) {
                                elevation_change = true;
                                rotation_elevation -= 0.05;
                            } else if (joystate.buttons[4]) {
                                elevation_change = true;
                                rotation_elevation += 0.05;
                            } else if (joystate.buttons[2]) {
                                elevation_change = true;
                                rotation_elevation = M_PI/2;
                            }
                            rotation_elevation = remainder(rotation_elevation, M_PI);
                            if ((rotation_elevation > 0) && (rotation_elevation < elevation_boundary_rad)) {
                                rotation_elevation = elevation_boundary_rad + 0.05;
                            }
                            if ((rotation_elevation < 0) && (rotation_elevation > -elevation_boundary_rad)) {
                                rotation_elevation = -(elevation_boundary_rad + 0.05);
                            }
                            msg.angular.z = msg.linear.x / tan(rotation_elevation);
                            control_pub.publish(msg);
                            if (elevation_change && (fabs(msg.linear.x)<1e-3)) {
                                set_control_mode(HDPCConst::MODE_INIT_ACKERMANN, rotation_elevation);
                            }
                        }
                        break;
                    case HDPCConst::MODE_ROTATION:
                        if (prev_control_mode != state.control_mode) {
                            ROS_INFO("Entered in mode ROTATION");
                            prev_control_mode = state.control_mode;
                        }
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving Rotation mode");
                        } else {
                            geometry_msgs::Twist msg;
                            // rotation speed in [-1,1] rad/s
                            msg.angular.z = joystate.axes[0] * max_rotational_velocity; 
                            if (joystate.buttons[3]) {
                                elevation_change = true;
                                rotation_elevation += 0.05;
                                if (rotation_elevation > elevation_boundary_rad) {
                                    rotation_elevation = elevation_boundary_rad-0.05;
                                }
                            } else if (joystate.buttons[4]) {
                                elevation_change = true;
                                rotation_elevation -= 0.05;
                                if (rotation_elevation < -elevation_boundary_rad) {
                                    rotation_elevation = -elevation_boundary_rad+0.05;
                                }
                            } else if (joystate.buttons[2]) {
                                elevation_change = true;
                                rotation_elevation = 0.;
                            }
                            msg.linear.x = msg.angular.z * tan(rotation_elevation);
                            control_pub.publish(msg);
                            if (elevation_change && (fabs(msg.angular.z)<1e-3)) {
                                set_control_mode(HDPCConst::MODE_INIT_ROTATION, rotation_elevation);
                            }
                        }
                        break;
                    case HDPCConst::MODE_INIT_ACKERMANN:
                    case HDPCConst::MODE_INIT_ROTATION:
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving Current mode");
                        }
                        break;
                }
                // cleanup
                for (unsigned int i=0;i<joystate.buttons.size();i++) {
                    joystate.buttons[i] = 0;
                }

            }
        }


};

// #define SIMULATION

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hdpc_teleop");
    ros::NodeHandle nh("~");

    HDPCController api(nh);

    api.joyctrl();

    return 0;
}


