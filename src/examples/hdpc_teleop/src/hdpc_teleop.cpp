#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <ros/ros.h>
#include <joy/Joy.h>

#include <hdpc_drive/SetControlMode.h>
#include <hdpc_drive/Status.h>
#include <hdpc_drive/Ackermann.h>
#include <hdpc_drive/DirectDrive.h>
#include <hdpc_com/HDPCGeometry.h>
#include <hdpc_com/HDPCConst.h>

using namespace hdpc_com;
using namespace hdpc_drive;

class HDPCController
{
	protected:
		joy::Joy joystate;
		hdpc_drive::Status state;
		ros::Subscriber state_sub;
		ros::Subscriber joy_sub;
		ros::Publisher control_pub;
		ros::Publisher directdrive_pub;
		ros::ServiceClient setModeClt;

        hdpc_com::HDPCGeometry geom;
        hdpc_drive::DirectDrive ddmsg;
        double elevation_boundary_rad;
        double max_linear_velocity;
        double max_rotational_velocity;

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

			setModeClt = nh.serviceClient<hdpc_drive::SetControlMode>("/hdpc_drive/set_control_mode");

			// Subscribe to the state
			state_sub = nh.subscribe("/hdpc_drive/status",1,&HDPCController::stateCallback, this);
			// Publishing the control
			control_pub = nh.advertise<hdpc_drive::Ackermann>("/hdpc_drive/ackermann",1);
			directdrive_pub = nh.advertise<hdpc_drive::DirectDrive>("/hdpc_drive/direct",1);

			joy_sub = nh.subscribe("/joy",1,&HDPCController::joyCallback, this);

            ROS_INFO("HDPC Teleop initialised and ready to roll!");
		}
		~HDPCController() {
		}

		void stateCallback(const hdpc_drive::Status::ConstPtr& msg) {
			state = *msg;
		}

		void joyCallback(const joy::Joy::ConstPtr& msg) {
			assert(msg->buttons.size()>4);
			assert(msg->axes.size()>=2);
			joystate = *msg;
            gotjoy = true;
		}

        void set_control_mode(unsigned int mode) {
            hdpc_drive::SetControlMode req;
            req.request.request_mode = mode;
            if (!setModeClt.call(req)) {
                ROS_ERROR("Set control mode %d failed",mode);
            }
        }

        void setdd(hdpc_drive::DirectDrive & ddmsg, int axle, double value) {
            if (axle < 6) {
                ddmsg.velocities_rad_per_sec[axle] = value;
            } else if (axle < 10) {
                ddmsg.steering_rad[axle-6] = value;
            }
            directdrive_pub.publish(ddmsg);
        }

        void joyctrl() {
            ros::Rate looprate(10);
            int current_axle = 0;
            double rotation_elevation = 0.0;

            while (ros::ok()) {
                looprate.sleep();
                ros::spinOnce();
                if (!gotjoy) continue;
                switch (state.control_mode) {
                    case HDPCConst::MODE_STOPPED:
                        if (joystate.buttons[1]) {
                            set_control_mode(HDPCConst::MODE_ACKERMANN);
                            ROS_INFO("Entering Ackermann mode.");
                            ROS_INFO("Press 0 to exit. Velocity: Axis 0. Steering: Axis 1");
                        } else if (joystate.buttons[2]) {
                            rotation_elevation = 0.0;
                            set_control_mode(HDPCConst::MODE_ROTATION);
                            ROS_INFO("Entering Rotation mode.");
                            ROS_INFO("Press 0 to exit. Rot. Velocity: Axis 1.");
                            ROS_INFO("ICR Positon: [-/0/+] wiht button [3/2/4]");
                        } else if (joystate.buttons[3]) {
                            ddmsg = hdpc_drive::DirectDrive();
                            current_axle = 0;
                            set_control_mode(HDPCConst::MODE_DIRECT_DRIVE);
                            ROS_INFO("Entering Direct Drive mode.");
                            ROS_INFO("Press 0 to exit. Control: Axis 0. Select DoF with [3/4]");
                        }
                        break;
                    case HDPCConst::MODE_DIRECT_DRIVE:
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving DirectDrive mode");
                        } else if (joystate.buttons[3]) {
                            setdd(ddmsg,current_axle,0.0);
                            current_axle = (current_axle - 1) % 10;
                            ROS_INFO("DirectDrive: controlling DoF %d",current_axle);
                        } else if (joystate.buttons[4]) {
                            setdd(ddmsg,current_axle,0.0);
                            current_axle = (current_axle + 1) % 10;
                            ROS_INFO("DirectDrive: controlling DoF %d",current_axle);
                        } else {
                            setdd(ddmsg,current_axle,joystate.axes[0]);
                        }
                        break;
                    case HDPCConst::MODE_ACKERMANN:
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving Ackermann mode");
                        } else {
                            hdpc_drive::Ackermann msg;
                            // rotation speed in [-0.5,0.5] m/s
                            msg.velocity = joystate.axes[0] * max_linear_velocity; 
                            if (fabs(msg.velocity) < 0.05) {
                                // dead zone at the center
                                msg.velocity = 0.0;
                            }
                            msg.elevation_rad = M_PI/2 - joystate.axes[1] * (M_PI/2 - elevation_boundary_rad);
                            if (fabs(msg.elevation_rad-M_PI/2) < 0.1) {
                                // dead zone at the center
                                msg.elevation_rad = M_PI/2;
                            }
                            control_pub.publish(msg);
                        }
                        break;
                    case HDPCConst::MODE_ROTATION:
                        if (joystate.buttons[0]) {
                            // Stop
                            set_control_mode(HDPCConst::MODE_STOPPED);
                            ROS_INFO("Leaving Rotation mode");
                        } else {
                            hdpc_drive::Ackermann msg;
                            // rotation speed in [-1,1] rad/s
                            msg.velocity = joystate.axes[1] * max_rotational_velocity; 
                            if (fabs(msg.velocity) < 0.1) {
                                // dead zone at the center
                                msg.velocity = 0.0;
                            }
                            if (joystate.buttons[3]) {
                                rotation_elevation += 0.1 * elevation_boundary_rad;
                                if (rotation_elevation > elevation_boundary_rad) {
                                    rotation_elevation = elevation_boundary_rad;
                                }
                            } else if (joystate.buttons[4]) {
                                rotation_elevation -= 0.1 * elevation_boundary_rad;
                                if (rotation_elevation < -elevation_boundary_rad) {
                                    rotation_elevation = -elevation_boundary_rad;
                                }
                            } else if (joystate.buttons[2]) {
                                rotation_elevation = 0.;
                            }
                            msg.elevation_rad = rotation_elevation;
                            control_pub.publish(msg);
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
	ros::NodeHandle nh;

	HDPCController api(nh);

	api.joyctrl();

	return 0;
}

		
