#ifndef HDPC_GEOMETRY_H
#define HDPC_GEOMETRY_H
#include <ros/ros.h>

namespace hdpc_com {
    struct HDPCGeometry {
        double rover_height;
        double rover_width;
        double rover_center_to_front;
        double rover_center_to_rear;
        double rover_rocker_to_front_axle;
        double rover_rocker_to_boggie_x;
        double rover_wheel_radius;
        double rover_rocker_to_boggie_z;
        double rover_boggie_to_axle_z;
        double rover_boggie_to_rear_axle;
        double rover_boggie_to_middle_axle;


        HDPCGeometry(ros::NodeHandle & nh) {
            nh.param("/rover_height",rover_height,0.577);
            nh.param("/rover_width",rover_width,0.820);
            nh.param("/rover_center_to_front",rover_center_to_front,1.500/2.);
            nh.param("/rover_center_to_rear",rover_center_to_rear,rover_center_to_front);
            nh.param("/rover_rocker_to_front_axle",rover_rocker_to_front_axle,rover_center_to_front);
            nh.param("/rover_rocker_to_boggie_x",rover_rocker_to_boggie_x,0.375);
            nh.param("/rover_wheel_radius",rover_wheel_radius,0.250/2.);
            nh.param("/rover_rocker_to_boggie_z",rover_rocker_to_boggie_z,rover_height - 0.317);
            nh.param("/rover_boggie_to_axle_z",rover_boggie_to_axle_z,0.317-rover_wheel_radius);
            nh.param("/rover_boggie_to_rear_axle",rover_boggie_to_rear_axle,0.375);
            nh.param("/rover_boggie_to_middle_axle",rover_boggie_to_middle_axle,0.375);
        }
    };
};

#endif // HDPC_GEOMETRY_H
