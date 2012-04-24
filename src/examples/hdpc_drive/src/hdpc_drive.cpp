
#include <math.h>

#include <ros/ros.h>
#include <hdpc_drive/SetControlMode.h>
#include <hdpc_drive/Ackermann.h>
#include <hdpc_drive/DirectDrive.h>
#include <hdpc_drive/HDPCConst.h>
#include <hdpc_drive/Status.h>

#include <hdpc_com/ChangeStateMachine.h>
#include <hdpc_com/Commands.h>
#include <hdpc_com/Readings.h>
#include <hdpc_com/HDPCStateMachineEnums.h>
#include <hdpc_com/HDPCGeometry.h>

using namespace hdpc_drive;

class HDPCDrive {
    protected:
        hdpc_com::HDPCGeometry geom;

        // Transition value between rotation on the spot and ackermann
        double elevation_boundary_rad;
        double max_rotation_speed_rad_per_s;
        double max_linear_speed_m_per_s;
    protected:
        ros::Publisher command_pub;
        ros::Subscriber reading_sub;
        ros::ServiceClient state_machine_client;


        ros::ServiceServer control_mode_serv;
        ros::Subscriber direct_command_sub;
        ros::Subscriber ackermann_command_sub;
        ros::Publisher status_pub;

        unsigned int control_mode;
        static const unsigned int WATCHDOG_INIT = 2;
        unsigned int watchdog;

        void stop_rover() {
            if (control_mode == HDPCConst::MODE_STOPPED) {
                return;
            }

            hdpc_com::Commands cmd;
            cmd.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = false;
                cmd.command[i] = 0;
            }
            command_pub.publish(cmd);

            hdpc_com::ChangeStateMachine change_state;
            change_state.request.event = EVENT_STOP;
            state_machine_client.call(change_state);

            control_mode = HDPCConst::MODE_STOPPED;
            ROS_INFO("Rover entered STOPPED mode");
        }

        void vel_and_steering(float v_c, float elev, float x_w, float y_w, float *v_w, float *steering_w=NULL) {
            float r = tan(elev);
            float r2 = r*r;
            if (steering_w) {
                *steering_w = atan(x_w/(r-y_w));
            }
            if (v_w) {
                switch (control_mode) {
                    case HDPCConst::MODE_ROTATION:
                        // Rotation mode: v_c is the rover body rotation speed in
                        // rad/s
                        *v_w = v_c * r;
                        break;
                    case HDPCConst::MODE_ACKERMANN:
                        // Ackermann mode: v_c is the rover center velocity in m/s
                        *v_w = v_c * sqrt( 1 + (x_w*x_w+y_w*y_w)/r2 - 2*y_w/r );
                        break;
                    default:
                        ROS_ERROR("vel_and_steering called outside of Ackermann or Rotation mode");
                        *v_w = 0;
                        break;
                }
            }
        }

        void drive_rover(float velocity, float elevation_rad) {
            hdpc_com::Commands cmd;
            cmd.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = true;
            }
            elevation_rad = remainder(elevation_rad,M_PI);
            switch (control_mode) {
                case HDPCConst::MODE_STOPPED:
                case HDPCConst::MODE_DIRECT_DRIVE:
                    ROS_WARN("Ignored drive command in current mode");
                    return;
                    break;
                case HDPCConst::MODE_ACKERMANN:
                    if ((elevation_rad >= 0) && (elevation_rad < elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ACKERMANN mode");
                        elevation_rad = elevation_boundary_rad;
                    }
                    if ((elevation_rad < 0) && (elevation_rad > -elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ACKERMANN mode");
                        elevation_rad = -elevation_boundary_rad;
                    }
                    if (velocity > max_linear_speed_m_per_s) {
                        velocity = max_linear_speed_m_per_s;
                    }
                    if (velocity <-max_linear_speed_m_per_s) {
                        velocity = -max_linear_speed_m_per_s;
                    }
                    break;
                case HDPCConst::MODE_ROTATION:
                    if ((elevation_rad >= 0) && (elevation_rad >= elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ROTATION mode");
                        elevation_rad = elevation_boundary_rad;
                    }
                    if ((elevation_rad < 0) && (elevation_rad <= -elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ROTATION mode");
                        elevation_rad = -elevation_boundary_rad;
                    }
                    if (velocity > max_rotation_speed_rad_per_s) {
                        velocity = max_rotation_speed_rad_per_s;
                    }
                    if (velocity <-max_rotation_speed_rad_per_s) {
                        velocity = -max_rotation_speed_rad_per_s;
                    }
                    break;
                default:
                    return;
            }
            vel_and_steering(velocity, elevation_rad, geom.rover_center_to_front, geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_FRONT_LEFT], &cmd.command[HDPCConst::STEERING_FRONT_LEFT]);
            vel_and_steering(velocity, elevation_rad, geom.rover_center_to_front, -geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_FRONT_RIGHT], &cmd.command[HDPCConst::STEERING_FRONT_RIGHT]);
            vel_and_steering(velocity, elevation_rad, 0, geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_MIDDLE_LEFT]);
            vel_and_steering(velocity, elevation_rad, 0, -geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_MIDDLE_RIGHT]);
            vel_and_steering(velocity, elevation_rad, -geom.rover_center_to_rear, geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_REAR_LEFT], &cmd.command[HDPCConst::STEERING_REAR_LEFT]);
            vel_and_steering(velocity, elevation_rad, -geom.rover_center_to_rear, -geom.rover_width/2.,
                    &cmd.command[HDPCConst::DRIVE_REAR_RIGHT], &cmd.command[HDPCConst::STEERING_REAR_RIGHT]);
            command_pub.publish(cmd);
        }

        bool set_mode(SetControlMode::Request  &req, SetControlMode::Response &res )
        {
            hdpc_com::ChangeStateMachine change_state;
            change_state.request.event = EVENT_READ_STATE;
            if (!state_machine_client.call(change_state)) {
                ROS_WARN("Change state machine: request state failed");
                return false;
            }
            switch (change_state.response.state) {
                case SM_INIT:
                case SM_STOP:
                    if (req.request_mode == HDPCConst::MODE_STOPPED) {
                        break;
                    }
                    change_state.request.event = EVENT_START;
                    if (!state_machine_client.call(change_state)) {
                        ROS_WARN("Change state machine failed");
                        res.result = false;
                        res.result_mode = control_mode;
                        return false;
                    }
                    break;
                case SM_DRIVE:
                    if (req.request_mode != HDPCConst::MODE_STOPPED) {
                        break;
                    }
                    change_state.request.event = EVENT_STOP;
                    if (!state_machine_client.call(change_state)) {
                        ROS_WARN("Change state machine failed");
                        res.result = false;
                        res.result_mode = control_mode;
                        return false;
                    }
                    break;
                case SM_FAULT:
                default:
                    ROS_WARN("State machine is in faulty or unknown state. Trying calling RESET");
                    return false;
            }
            // If we reach this state we're good. Now we can have the logic 
            // of the transition
            res.result = true;
            switch (req.request_mode) {
                case HDPCConst::MODE_ACKERMANN:
                case HDPCConst::MODE_ROTATION:
                    if (control_mode != HDPCConst::MODE_STOPPED) {
                        ROS_WARN("Cannot switch to ACKERMANN or ROTATION from any other mode than STOPPED");
                        res.result = false;
                    } else {
                        control_mode = req.request_mode;
                    }
                    break;
                case HDPCConst::MODE_STOPPED:
                case HDPCConst::MODE_DIRECT_DRIVE:
                default:
                    // authorized from any mode
                    control_mode = req.request_mode;
                    break;
            }
            watchdog = WATCHDOG_INIT;
            res.result_mode = control_mode;
            return true;
        }

        void readingsCallback(const hdpc_com::Readings::ConstPtr & msg) {
            Status sts;
            sts.header.frame_id = "rover";
            sts.header.stamp = ros::Time::now();
            sts.control_mode = control_mode;
            sts.motors = *msg;
            status_pub.publish(sts);
            // TODO: create a visualization node
        }

        void directDriveCallback(const DirectDrive::ConstPtr& msg)
        {
            if (control_mode != HDPCConst::MODE_DIRECT_DRIVE) {
                ROS_WARN("Ignored direct drive command while not in DIRECT_DRIVE mode");
                return;
            }
            watchdog = WATCHDOG_INIT;
            hdpc_com::Commands cmd;
            cmd.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = true;
            }
            cmd.command[HDPCConst::DRIVE_FRONT_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_FRONT_LEFT];
            cmd.command[HDPCConst::DRIVE_FRONT_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_FRONT_RIGHT];
            cmd.command[HDPCConst::DRIVE_MIDDLE_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_MIDDLE_LEFT];
            cmd.command[HDPCConst::DRIVE_MIDDLE_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_MIDDLE_RIGHT];
            cmd.command[HDPCConst::DRIVE_REAR_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_REAR_LEFT];
            cmd.command[HDPCConst::DRIVE_REAR_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_REAR_RIGHT];
            cmd.command[HDPCConst::STEERING_FRONT_LEFT] = msg->steering_rad[DirectDrive::WHEEL_FRONT_LEFT];
            cmd.command[HDPCConst::STEERING_FRONT_RIGHT] = msg->steering_rad[DirectDrive::WHEEL_FRONT_RIGHT]; 
            cmd.command[HDPCConst::STEERING_REAR_LEFT] = msg->steering_rad[DirectDrive::WHEEL_REAR_LEFT]; 
            cmd.command[HDPCConst::STEERING_REAR_RIGHT] = msg->steering_rad[DirectDrive::WHEEL_REAR_RIGHT]; 
            command_pub.publish(cmd);
        }

        void ackermannCallback(const Ackermann::ConstPtr& msg)
        {
            watchdog = WATCHDOG_INIT;
            drive_rover(msg->velocity, msg->elevation_rad);
        }

    public:
        HDPCDrive(ros::NodeHandle & nh) : geom(nh) {
            command_pub = nh.advertise<hdpc_com::Commands>("/hdpc_com/commands",1);
            reading_sub = nh.subscribe("/hdpc_com/readings",1,&HDPCDrive::readingsCallback,this);
            state_machine_client = nh.serviceClient<hdpc_com::ChangeStateMachine>("/hdpc_com/change_state_machine");


            control_mode_serv = nh.advertiseService("set_control_mode",&HDPCDrive::set_mode,this);
            direct_command_sub = nh.subscribe("direct",1,&HDPCDrive::directDriveCallback,this);
            ackermann_command_sub = nh.subscribe("ackermann",1,&HDPCDrive::ackermannCallback,this);
            status_pub = nh.advertise<Status>("status",1);

            nh.param("max_rotation_speed_rad_per_s",max_rotation_speed_rad_per_s,1.0);
            nh.param("max_linear_speed_m_per_s",max_linear_speed_m_per_s,0.9);

            elevation_boundary_rad = atan(geom.rover_width/2);

            watchdog = 0;
            stop_rover();
        }

        bool wait_for_services() {
            state_machine_client.waitForExistence();
            return true;
        }
        
        void main_loop() {
            ros::Rate rate(1);
            while (ros::ok()) {
                ros::spinOnce();
                if (watchdog == 0) {
                    stop_rover();
                } else {
                    watchdog -= 1;
                }
                rate.sleep();
            }
        }


};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"hdpc_drive");
    ros::NodeHandle nh("~");

    HDPCDrive driver(nh);

    driver.wait_for_services();

    driver.main_loop();

    return 0;
}

