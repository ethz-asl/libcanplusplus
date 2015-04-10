
#include <math.h>

#include <ros/ros.h>
#include <hdpc_drive/SetControlMode.h>
#include <geometry_msgs/Twist.h>
#include <hdpc_drive/DirectDrive.h>
#include <hdpc_drive/Status.h>
#include <hdpc_drive/HDPCModes.h>

#include <hdpc_com/HDPCConst.h>
#include <hdpc_com/ChangeStateMachine.h>
#include <hdpc_com/Commands.h>
#include <hdpc_com/Readings.h>
#include <hdpc_com/HDPCStateMachineEnums.h>
#include <hdpc_com/HDPCGeometry.h>

using namespace hdpc_drive;
using namespace hdpc_com;

class HDPCDrive {
    protected:
        hdpc_com::HDPCGeometry geom;

        // Transition value between rotation on the spot and ackermann
        double max_rotation_speed_rad_per_s;
        double max_linear_speed_m_per_s;
        bool synchronise_steering;
        bool zero_on_init;
    protected:
        ros::Publisher command_pub;
        ros::Subscriber reading_sub;
        ros::ServiceClient state_machine_client;


        ros::ServiceServer control_mode_serv;
        ros::Subscriber direct_command_sub;
        ros::Subscriber ackermann_command_sub;
        ros::Publisher status_pub;

        unsigned int control_mode;
        double desired_elevation;
        static const unsigned int WATCHDOG_INIT = 100;
        unsigned int watchdog;
        hdpc_com::Readings motors;
        hdpc_com::Commands commands;

        static inline double SQR(double x) {return x*x;}
        static inline double SGN(double x) {return (x<0)?-1:1;}

        void stop_rover() {
            double max_wheel_offset = 0;
            for (unsigned int i=6;i<10;i++) {
                max_wheel_offset = std::max(fabs(motors.position[i]),max_wheel_offset);
            }

            // ROS_INFO("STOPPING: Max wheel offset: %02f",max_wheel_offset);
            commands.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<6;i++) {
                commands.isActive[i] = false;
                commands.velocity[i] = 0.0;
            }
            for (unsigned int i=6;i<10;i++) {
                commands.velocity[i] = 0.0;
            }

            if (zero_on_init && (max_wheel_offset > 5e-2)) {
                for (unsigned int i=6;i<10;i++) {
                    commands.isActive[i] = true;
                    commands.position[i] = 0.0;
                }
            } else if (control_mode != HDPCModes::MODE_STOPPED) {
                for (unsigned int i=6;i<10;i++) {
                    commands.isActive[i] = false;
                    commands.position[i] = motors.position[i];
                }
                hdpc_com::ChangeStateMachine change_state;
                change_state.request.event = EVENT_STOP;
                state_machine_client.call(change_state);

                control_mode = HDPCModes::MODE_STOPPED;
                if (watchdog == 0) {
                    ROS_INFO("HDPC Drive: Rover entered STOPPED mode on watchdog");
                } else {
                    ROS_INFO("HDPC Drive: Rover entered STOPPED mode");
                }
            }
        }


        void prepare_steering(double elevation_rad) {
            double desired[10];
            elevation_rad = remainder(elevation_rad,M_PI);
            double elevation_limit_1 = atan((geom.rover_width - geom.rover_wheel_width)/2);
            double elevation_limit_2 = atan((geom.rover_width + geom.rover_wheel_width)/2);
            if ((fabs(elevation_rad) > elevation_limit_1) && (fabs(elevation_rad) < elevation_limit_2)) {
                if (control_mode == HDPCModes::MODE_INIT_ROTATION) {
                    elevation_rad = elevation_limit_1*SGN(elevation_rad);
                } 
                if (control_mode == HDPCModes::MODE_INIT_ACKERMANN) {
                    elevation_rad = elevation_limit_2*SGN(elevation_rad);
                }
            }
            desired[HDPCConst::STEERING_FRONT_LEFT] = remainder(atan2(geom.rover_center_to_front,tan(elevation_rad) - geom.rover_width/2.),M_PI);
            desired[HDPCConst::STEERING_FRONT_RIGHT] = remainder(atan2(geom.rover_center_to_front,tan(elevation_rad) + geom.rover_width/2.),M_PI);
            desired[HDPCConst::STEERING_REAR_LEFT] = remainder(atan2(-geom.rover_center_to_rear,tan(elevation_rad) - geom.rover_width/2.),M_PI);
            desired[HDPCConst::STEERING_REAR_RIGHT] = remainder(atan2(-geom.rover_center_to_rear,tan(elevation_rad) + geom.rover_width/2.),M_PI);

            double max_wheel_offset = 0;
            for (unsigned int i=6;i<10;i++) {
                max_wheel_offset = std::max(fabs(remainder(motors.position[i]-desired[i],M_PI)),max_wheel_offset);
            }

            if (max_wheel_offset > 5e-2) {
                // ROS_INFO("INIT ROT: Max wheel offset: %02f",max_wheel_offset);
                commands.header.stamp = ros::Time::now();
                for (unsigned int i=0;i<6;i++) {
                    commands.isActive[i] = false;
                    commands.velocity[i] = 0.0;
                }
                for (unsigned int i=6;i<10;i++) {
                    commands.isActive[i] = true;
                    commands.position[i] = desired[i];
                    commands.velocity[i] = 0.0;
                }
            } else if (control_mode == HDPCModes::MODE_INIT_ROTATION) {
                control_mode = HDPCModes::MODE_ROTATION;
                // ROS_INFO("HDPC Drive: Rover entered ROTATION mode");
            } else  if (control_mode == HDPCModes::MODE_INIT_ACKERMANN) {
                control_mode = HDPCModes::MODE_ACKERMANN;
                // ROS_INFO("HDPC Drive: Rover entered ACKERMANN mode");
            }
        }


        void vel_and_steering(float v_c, float omega_c, unsigned int i_vel, signed int i_steer, float x_w, float y_w) {
            if ((fabs(omega_c) < 1e-3) && (fabs(v_c) < 1e-3)) {
                commands.velocity[i_vel] = 0.0;
                commands.isActive[i_vel] = false;
            } else {
                double phi_w = atan2(x_w*omega_c,(v_c - y_w*omega_c));
                commands.isActive[i_vel] = true;
                commands.velocity[i_vel] = sqrt(SQR(x_w * omega_c) + SQR(v_c - y_w * omega_c)) / geom.rover_wheel_radius;
                if (fabs(phi_w) > M_PI/2) {
                    // If the wheel had to turn more than 90 degrees, then
                    // the other side is used, and the velocity must be
                    // negated
                    phi_w = remainder(phi_w,M_PI);
                    commands.velocity[i_vel] *= -1;
                }
                if (i_steer>=0) {
                    commands.isActive[i_steer] = true;
                    commands.position[i_steer] = phi_w;
                    commands.velocity[i_steer] = 0.;
                }
            }
        }

        void drive_rover(float velocity, float omega) {
            commands.header.stamp = ros::Time::now();
            if (omega > max_rotation_speed_rad_per_s) {
                omega = max_rotation_speed_rad_per_s;
            }
            if (omega <-max_rotation_speed_rad_per_s) {
                omega = -max_rotation_speed_rad_per_s;
            }
            if (velocity > max_linear_speed_m_per_s) {
                velocity = max_linear_speed_m_per_s;
            }
            if (velocity <-max_linear_speed_m_per_s) {
                velocity = -max_linear_speed_m_per_s;
            }
            double r = velocity / omega;
            switch (control_mode) {
                case HDPCModes::MODE_INIT:
                case HDPCModes::MODE_DIRECT_DRIVE:
                case HDPCModes::MODE_INIT_ACKERMANN:
                case HDPCModes::MODE_INIT_ROTATION:
                    ROS_WARN("Ignored drive command in current mode");
                    return;
                    break;
                case HDPCModes::MODE_ACKERMANN:
                    if (fabs(velocity) > 1e-3) {
                        if ((r >= 0) && (r < (geom.rover_width + geom.rover_wheel_width)/2 - 1e-3)) {
                            ROS_DEBUG("Clipped rotation speed in ACKERMANN mode");
                            omega = 2*velocity / (geom.rover_width + geom.rover_wheel_width);
                        }
                        if ((r < 0) && (r > -(geom.rover_width + geom.rover_wheel_width)/2 + 1e-3)) {
                            ROS_DEBUG("Clipped rotation speed in ACKERMANN mode");
                            omega = -2*velocity / (geom.rover_width + geom.rover_wheel_width);
                        }
                    }
                    break;
                case HDPCModes::MODE_ROTATION:
                    if (fabs(omega) > 1e-3) {

                        if ((r >= 0) && (r > (geom.rover_width - geom.rover_wheel_width)/2 + 1e-3)) {
                            ROS_DEBUG("Clipped velocity speed in ROTATION mode");
                            velocity = omega * (geom.rover_width - geom.rover_wheel_width)/2;
                        }
                        if ((r < 0) && (r < -(geom.rover_width - geom.rover_wheel_width)/2 - 1e-3)) {
                            ROS_DEBUG("Clipped velocity speed in ROTATION mode");
                            velocity = -omega * (geom.rover_width - geom.rover_wheel_width)/2;
                        }
                    }
                    break;
                default:
                    return;
            }
            double max_t_steering = 0, delta_steering[10];
            unsigned int i;
            i = HDPCConst::STEERING_FRONT_LEFT;
            vel_and_steering(velocity, omega, HDPCConst::DRIVE_FRONT_LEFT, i, geom.rover_center_to_front, geom.rover_width/2.);
            delta_steering[i] = fabs(commands.position[i] - motors.position[i]);
            max_t_steering = std::max(max_t_steering,fabs(delta_steering[i])/geom.rover_max_steering_velocity);

            i = HDPCConst::STEERING_FRONT_RIGHT;
            vel_and_steering(velocity, omega, HDPCConst::DRIVE_FRONT_RIGHT, i, geom.rover_center_to_front, -geom.rover_width/2.);
            delta_steering[i] = fabs(commands.position[i] - motors.position[i]);
            max_t_steering = std::max(max_t_steering,fabs(delta_steering[i])/geom.rover_max_steering_velocity);

            vel_and_steering(velocity, omega, HDPCConst::DRIVE_MIDDLE_LEFT, -1, 0, geom.rover_width/2.);
            vel_and_steering(velocity, omega, HDPCConst::DRIVE_MIDDLE_RIGHT, -1, 0, -geom.rover_width/2.);

            i = HDPCConst::STEERING_REAR_LEFT;
            vel_and_steering(velocity, omega, HDPCConst::DRIVE_REAR_LEFT, i, -geom.rover_center_to_rear, geom.rover_width/2.);
            delta_steering[i] = fabs(commands.position[i] - motors.position[i]);
            max_t_steering = std::max(max_t_steering,fabs(delta_steering[i])/geom.rover_max_steering_velocity);
            
            i = HDPCConst::STEERING_REAR_RIGHT;
            vel_and_steering(velocity, omega, HDPCConst::DRIVE_REAR_RIGHT, i, -geom.rover_center_to_rear, -geom.rover_width/2.);
            delta_steering[i] = fabs(commands.position[i] - motors.position[i]);
            max_t_steering = std::max(max_t_steering,fabs(delta_steering[i])/geom.rover_max_steering_velocity);

            if (synchronise_steering) {
                commands.velocity[HDPCConst::STEERING_FRONT_LEFT] = delta_steering[HDPCConst::STEERING_FRONT_LEFT]/max_t_steering;
                commands.velocity[HDPCConst::STEERING_FRONT_RIGHT] = delta_steering[HDPCConst::STEERING_FRONT_RIGHT]/max_t_steering;
                commands.velocity[HDPCConst::STEERING_REAR_LEFT] = delta_steering[HDPCConst::STEERING_REAR_LEFT]/max_t_steering;
                commands.velocity[HDPCConst::STEERING_REAR_RIGHT] = delta_steering[HDPCConst::STEERING_REAR_RIGHT]/max_t_steering;
            }
            
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
                    if (req.request_mode == HDPCModes::MODE_STOPPED) {
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
                    // Normal. Nothing to do. Stop rover will stop the rover
                    // eventually if required
                    break;
                case SM_FAULT:
                	ROS_WARN("State machine is in FAULT state. Call RESET!");
                	break;
                default:
                    ROS_WARN("State machine is in faulty or unknown state. Trying calling RESET");
                    return false;
            }
            // If we reach this state we're good. Now we can have the logic 
            // of the transition
            res.result = true;
            if (control_mode != req.request_mode) {
                switch (req.request_mode) {
                    case HDPCModes::MODE_INIT_ROTATION:
                    case HDPCModes::MODE_ROTATION:
                        if ((control_mode != HDPCModes::MODE_STOPPED) 
                                && (control_mode != HDPCModes::MODE_ROTATION)) {
                            ROS_WARN("Cannot switch to ROTATION from any other mode than STOPPED");
                            res.result = false;
                        } else {
                            control_mode = HDPCModes::MODE_INIT_ROTATION;
                            desired_elevation = req.desired_elevation;
                            //ROS_INFO("Entering Init Rotation mode");
                        }
                        break;
                    case HDPCModes::MODE_INIT_ACKERMANN:
                    case HDPCModes::MODE_ACKERMANN:
                        if ((control_mode != HDPCModes::MODE_STOPPED) 
                                && (control_mode != HDPCModes::MODE_ACKERMANN)) {
                            ROS_WARN("Cannot switch to ACKERMANN or ROTATION from any other mode than STOPPED");
                            res.result = false;
                        } else {
                            control_mode = HDPCModes::MODE_INIT_ACKERMANN;
                            desired_elevation = req.desired_elevation;
                            //ROS_INFO("Entering Init Ackermann mode");
                        }
                        break;
                    case HDPCModes::MODE_DIRECT_DRIVE:
                        control_mode = req.request_mode;
                        ROS_INFO("Entering Direct Drive mode");
                        break;
                    case HDPCModes::MODE_INIT:
                    case HDPCModes::MODE_STOPPED:
                        control_mode = HDPCModes::MODE_INIT;
                        ROS_INFO("Entering INIT mode");
                        break;
                    default:
                        break;
                }
            } else {
                ROS_INFO("Ignoring transition request to the current mode");
            }
            watchdog = WATCHDOG_INIT;
            res.result_mode = control_mode;
            // ROS_INFO("Rover set mode successful: %d",control_mode);
            return true;
        }

        void readingsCallback(const hdpc_com::Readings::ConstPtr & msg) {
            Status sts;
            sts.header.frame_id = "rover";
            sts.header.stamp = ros::Time::now();
            sts.control_mode = control_mode;
            motors = sts.motors = *msg;
            status_pub.publish(sts);
        }

        void directDriveCallback(const DirectDrive::ConstPtr& msg)
        {
            if (control_mode != HDPCModes::MODE_DIRECT_DRIVE) {
                ROS_WARN("Ignored direct drive command while not in DIRECT_DRIVE mode");
                return;
            }
            watchdog = WATCHDOG_INIT;
            commands.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                commands.isActive[i] = true;
            }
            commands.velocity[HDPCConst::DRIVE_FRONT_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_FRONT_LEFT];
            commands.velocity[HDPCConst::DRIVE_FRONT_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_FRONT_RIGHT];
            commands.velocity[HDPCConst::DRIVE_MIDDLE_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_MIDDLE_LEFT];
            commands.velocity[HDPCConst::DRIVE_MIDDLE_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_MIDDLE_RIGHT];
            commands.velocity[HDPCConst::DRIVE_REAR_LEFT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_REAR_LEFT];
            commands.velocity[HDPCConst::DRIVE_REAR_RIGHT] = msg->velocities_rad_per_sec[DirectDrive::WHEEL_REAR_RIGHT];
            commands.velocity[HDPCConst::STEERING_FRONT_LEFT] = 0.0;
            commands.velocity[HDPCConst::STEERING_FRONT_RIGHT] = 0.0;
            commands.velocity[HDPCConst::STEERING_REAR_LEFT] = 0.0;
            commands.velocity[HDPCConst::STEERING_REAR_RIGHT] = 0.0;
            commands.position[HDPCConst::STEERING_FRONT_LEFT] = msg->steering_rad[DirectDrive::WHEEL_FRONT_LEFT];
            commands.position[HDPCConst::STEERING_FRONT_RIGHT] = msg->steering_rad[DirectDrive::WHEEL_FRONT_RIGHT]; 
            commands.position[HDPCConst::STEERING_REAR_LEFT] = msg->steering_rad[DirectDrive::WHEEL_REAR_LEFT]; 
            commands.position[HDPCConst::STEERING_REAR_RIGHT] = msg->steering_rad[DirectDrive::WHEEL_REAR_RIGHT]; 
        }

        void ackermannCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            if ((control_mode != HDPCModes::MODE_ACKERMANN) && 
                    (control_mode != HDPCModes::MODE_ROTATION)) {
                ROS_WARN("Ignored Ackermann command while not in Ackermann/Rotation mode");
                return;
            }
            watchdog = WATCHDOG_INIT;
            drive_rover(msg->linear.x, msg->angular.z);
        }

    public:
        HDPCDrive(ros::NodeHandle & nh) : geom(nh) {
            command_pub = nh.advertise<hdpc_com::Commands>("/hdpc_com/commands",1);
            reading_sub = nh.subscribe("/hdpc_com/readings",1,&HDPCDrive::readingsCallback,this);
            state_machine_client = nh.serviceClient<hdpc_com::ChangeStateMachine>("/hdpc_com/changeState");


            control_mode_serv = nh.advertiseService("set_control_mode",&HDPCDrive::set_mode,this);
            direct_command_sub = nh.subscribe("direct",1,&HDPCDrive::directDriveCallback,this);
            ackermann_command_sub = nh.subscribe("ackermann",1,&HDPCDrive::ackermannCallback,this);
            status_pub = nh.advertise<Status>("status",1);

            // TODO: change default value to something that makes sense
            nh.param("max_rotation_speed_rad_per_s",max_rotation_speed_rad_per_s,1.0);
            nh.param("max_linear_speed_m_per_s",max_linear_speed_m_per_s,0.9);
            nh.param("synchronise_steering",synchronise_steering,false);
            nh.param("zero_on_init",zero_on_init,false);

            watchdog = 0;
            desired_elevation = M_PI/2;
            control_mode = HDPCModes::MODE_INIT;
            commands.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                commands.isActive[i] = false;
                commands.velocity[i] = 0;
                commands.position[i] = 0.0;
            }
        }

        bool wait_for_services() {
            state_machine_client.waitForExistence();
            ROS_INFO("State machine service is ready, waiting for init");
            hdpc_com::ChangeStateMachine change_state;
            ros::Rate rate(1);
            do {
                rate.sleep();
                change_state.request.event = EVENT_READ_STATE;
                if (!state_machine_client.call(change_state)) {
                    ROS_WARN("Change state machine: request state failed");
                    return false;
                }
            } while (ros::ok() && (change_state.response.state != SM_STOP));
            ROS_INFO("HDPC Drive: ROVER is ready");

            change_state.request.event = EVENT_START;
            if (!state_machine_client.call(change_state)) {
                ROS_WARN("Change state machine: starting failed");
                return false;
            }

            watchdog = 0;
            control_mode = HDPCModes::MODE_INIT;
            commands.header.stamp = ros::Time::now();
            for (unsigned int i=0;i<10;i++) {
                commands.isActive[i] = false;
                commands.velocity[i] = 0;
                commands.position[i] = 0.0;
            }
            
            return true;
        }
        
        void main_loop() {
            ros::Rate rate(40); //50
            hdpc_com::ChangeStateMachine change_state;

            while (ros::ok()) {
                ros::spinOnce();

                // Check for Fault State
                change_state.request.event = EVENT_READ_STATE;
                if (!state_machine_client.call(change_state)) {
                	ROS_WARN("Request of state machine failed!");
                }

                switch (change_state.response.state){
                	case SM_INIT:
                	case SM_STOP:
                	case SM_DRIVE:
                		break;
                	case SM_FAULT:
                		if (control_mode!=HDPCModes::MODE_STOPPED){
                			control_mode = HDPCModes::MODE_STOPPED;
                			ROS_INFO("HDPC_COM in FAULT state, going to STOPPED MODE");
                		}
                		break;
                }


                switch (control_mode) {
                    case HDPCModes::MODE_INIT:
                        stop_rover();
                        command_pub.publish(commands);
                        break;
                    case HDPCModes::MODE_INIT_ACKERMANN:
                    case HDPCModes::MODE_INIT_ROTATION:
                        prepare_steering(desired_elevation);
                        command_pub.publish(commands);
                        break;
                    case HDPCModes::MODE_ROTATION:
                    case HDPCModes::MODE_ACKERMANN:
                    case HDPCModes::MODE_DIRECT_DRIVE:
                        if (watchdog == 0) {
                            control_mode = HDPCModes::MODE_INIT;
                        } else {
                            watchdog -= 1;
                            command_pub.publish(commands);
                        }
                        break;
                    case HDPCModes::MODE_STOPPED:
                    default:
                        break;
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

