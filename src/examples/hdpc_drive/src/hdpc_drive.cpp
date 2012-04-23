

#include <ros/ros.h>
#include <hdpc_drive/SetControlMode.h>
#include <hdpc_drive/Ackermann.h>
#include <hdpc_drive/DirectDrive.h>

#include <hdpc_com/ChangeStateMachine.h>
#include <hdpc_com/Commands.h>
#include <hdpc_com/Readings.h>
#include <HDPCStateMachineEnums.h>

using namespace hdpc_drive;

class HDPCDrive {
    protected:
        float rover_center_to_front;
        float rover_center_to_rear;
        float rover_width;

        // Transition value between rotation on the spot and ackermann
        float elevation_boundary_rad;
    protected:
        ros::Publisher command_pub;
        ros::Subscriber reading_sub;
        ros::ServiceClient state_machine_serv;


        ros::ServiceServer control_mode_serv;
        ros::Subscriber direct_command_sub;
        ros::Subscriber ackermann_command_sub;
        ros::Publisher status_pub;

        unsigned int control_mode;
        static const unsigned int WATCHDOG_INIT = 2;
        unsigned int watchdog;

        void stop_rover() {
            if (control_mode == SetControlMode::MODE_STOPPED) {
                return;
            }

            hdpc_com::Commands cmd;
            cmd.header = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = false;
                cmd.command[i] = 0;
            }
            command_pub.publish(cmd);

            hdpc_com::ChangeStateMachine change_state;
            change_state.event = EVENT_STOP;
            state_machine_serv.call(change_state);

            control_mode = SetControlMode::MODE_STOPPED;
            ROS_INFO("Rover entered STOPPED mode");
        }

        void drive_rover(float velocity_ms, float elevation_rad) {
            hdpc_com::Commands cmd;
            cmd.header = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = true;
            }
            switch (control_mode) {
                case SetControlMode::MODE_STOPPED:
                case SetControlMode::MODE_DIRECT_DRIVE:
                    ROS_WARN("Ignored drive command in current mode");
                    return;
                    break;
                case SetControlMode::MODE_ACKERMANN:
                    if ((elevation_rad >= 0) && (elevation_rad < elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ACKERMANN mode");
                        elevation_rad = elevation_boundary_rad;
                    }
                    if ((elevation_rad < 0) && (elevation_rad > -elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ACKERMANN mode");
                        elevation_rad = -elevation_boundary_rad;
                    }
                    break;
                case SetControlMode::MODE_ROTATION:
                    if ((elevation_rad >= 0) && (elevation_rad >= elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ROTATION mode");
                        elevation_rad = elevation_boundary_rad;
                    }
                    if ((elevation_rad < 0) && (elevation_rad <= -elevation_boundary_rad)) {
                        ROS_WARN("Clipped elevation to the minimum possible elevation in ROTATION mode");
                        elevation_rad = -elevation_boundary_rad;
                    }
                    break;
                default:
                    return;
            }
            // blah blah compute the geometry
            cmd.command[Status::DRIVE_FRONT_LEFT] = 0;
            cmd.command[Status::DRIVE_FRONT_RIGHT] = 0;
            cmd.command[Status::DRIVE_MIDDLE_LEFT] = 0;
            cmd.command[Status::DRIVE_MIDDLE_RIGHT] = 0;
            cmd.command[Status::DRIVE_REAR_LEFT] = 0;
            cmd.command[Status::DRIVE_REAR_RIGHT] = 0;
            cmd.command[Status::STEERING_FRONT_LEFT] = 0;
            cmd.command[Status::STEERING_FRONT_RIGHT] = 0;
            cmd.command[Status::STEERING_REAR_LEFT] = 0;
            cmd.command[Status::STEERING_REAR_RIGHT] = 0;
            command_pub.publish(cmd);
        }

        bool set_mode(SetControlMode::Request  &req, SetControlMode::Response &res )
        {
            hdpc_com::ChangeStateMachine change_state;
            change_state.request.event = EVENT_READ_STATE;
            if (!state_machine_serv.call(change_state)) {
                ROS_WARN("Change state machine: request state failed");
                return false;
            }
            switch (change_state.response.state) {
                case SM_INIT:
                case SM_STOP:
                    if (req.request_mode == SetControlMode::MODE_STOPPED) {
                        break;
                    }
                    change_state.request.event = EVENT_START;
                    if (!state_machine_serv.call(change_state)) {
                        ROS_WARN("Change state machine failed");
                        res.result = false;
                        res.result_mode = control_mode;
                        return false;
                    }
                    break;
                case SM_DRIVE:
                    if (req.request_mode != SetControlMode::MODE_STOPPED) {
                        break;
                    }
                    change_state.request.event = EVENT_STOP;
                    if (!state_machine_serv.call(change_state)) {
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
            switch (req.control_mode) {
                case MODE_ACKERMANN:
                case MODE_ROTATION:
                    if (control_mode != MODE_STOPPED) {
                        ROS_WARN("Cannot switch to ACKERMANN or ROTATION from any other mode than STOPPED");
                        res.result = false;
                    } else {
                        control_mode = req.request_mode;
                    }
                    break;
                case MODE_STOPPED:
                case MODE_DIRECT_DRIVE:
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
            sts.control_mode = control_mode;
            sts.motos = *msg;
            status_pub.publish(sts);
        }

        void directDriveCallback(const DirectDrive::ConstPtr& msg)
        {
            if (control_mode != SetControlMode::MODE_DIRECT_DRIVE) {
                ROS_WARN("Ignored direct drive command while not in DIRECT_DRIVE mode");
                return;
            }
            watchdog = WATCHDOG_INIT;
            hdpc_com::Commands cmd;
            cmd.header = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                cmd.isActive[i] = true;
            }
            cmd.command[Status::DRIVE_FRONT_LEFT] = msg.velocities_rad_per_sec[WHEEL_FRONT_LEFT];
            cmd.command[Status::DRIVE_FRONT_RIGHT] = msg.velocities_rad_per_sec[WHEEL_FRONT_RIGHT];
            cmd.command[Status::DRIVE_MIDDLE_LEFT] = msg.velocities_rad_per_sec[WHEEL_MIDDLE_LEFT];
            cmd.command[Status::DRIVE_MIDDLE_RIGHT] = msg.velocities_rad_per_sec[WHEEL_MIDDLE_RIGHT];
            cmd.command[Status::DRIVE_REAR_LEFT] = msg.velocities_rad_per_sec[WHEEL_REAR_LEFT];
            cmd.command[Status::DRIVE_REAR_RIGHT] = msg.velocities_rad_per_sec[WHEEL_REAR_RIGHT];
            cmd.command[Status::STEERING_FRONT_LEFT] = msg.steering_rad[WHEEL_FRONT_LEFT];
            cmd.command[Status::STEERING_FRONT_RIGHT] = msg.steering_rad[WHEEL_FRONT_RIGHT]; 
            cmd.command[Status::STEERING_REAR_LEFT] = msg.steering_rad[WHEEL_REAR_LEFT]; 
            cmd.command[Status::STEERING_REAR_RIGHT] = msg.steering_rad[WHEEL_REAR_RIGHT]; 
            command_pub.publish(cmd);
        }

        void ackermannCallback(const Ackermann::ConstPtr& msg)
        {
            watchdog = WATCHDOG_INIT;
            drive_rover(msg.velocity_m_per_sec, msg.elevation_rad);
        }

    public:
        HDPCDrive(ros::NodeHandle & nh) {
            watchdog = 0;
            stop_rover();

            // create publishers, subscriber, services
            // read params (rover lengths)
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

    driver.main_loop();

    return 0;
}

