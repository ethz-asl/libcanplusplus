

#include <math.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


#include <hdpc_com/HDPCGeometry.h>
#include <hdpc_com/HDPCConst.h>
#include <hdpc_drive/Status.h>

using namespace hdpc_drive;
using namespace hdpc_com;

class HDPCVisualize {
    protected:
        hdpc_com::HDPCGeometry geom;
        int verbose;

    protected:
        ros::Subscriber status_sub;
        ros::Publisher marker_pub;
        tf::TransformBroadcaster br;
       
        void sendtf(double x, double y, double z, double roll, double pitch, double yaw, 
                const ros::Time & t, const std::string & parent, const std::string & offspring) {
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin( tf::Vector3(x, y, z) );
            q.setRPY(roll,pitch,yaw); transform.setRotation( q );
            br.sendTransform(tf::StampedTransform(transform, t, parent, offspring));
            if (verbose) {
                ROS_INFO("Published %s -> %s",parent.c_str(),offspring.c_str());
            }
        }

        void sendtf(double x, double y, double z, 
                const ros::Time & t, const std::string & parent, const std::string & offspring) {
            sendtf(x,y,z,0,0,0,t,parent,offspring);
        }



        void statusCallback(const hdpc_drive::Status::ConstPtr & msg) {
            sendtf(0,0,geom.rover_height,msg->header.stamp,"world","body");

            sendtf(0,geom.rover_width/2,0,
                    0,msg->motors.analog[HDPCConst::DRIVE_MIDDLE_LEFT],0, 
                    msg->header.stamp,"body","LeftRocker");
            sendtf(geom.rover_rocker_to_front_axle,0,-(geom.rover_rocker_to_boggie_z+geom.rover_boggie_to_axle_z),
                    msg->header.stamp,"LeftRocker","FLSteering");
            sendtf(-geom.rover_rocker_to_boggie_x,0,-geom.rover_rocker_to_boggie_z,
                    0,msg->motors.analog[HDPCConst::DRIVE_REAR_LEFT],0, 
                    msg->header.stamp,"LeftRocker","LeftBoggie");
            sendtf(geom.rover_boggie_to_middle_axle,0,-geom.rover_boggie_to_axle_z,
                    msg->header.stamp,"LeftBoggie","CLAxle");
            sendtf(-geom.rover_boggie_to_rear_axle,0,-geom.rover_boggie_to_axle_z,
                    msg->header.stamp,"LeftBoggie","RLSteering");
            sendtf(0,0,0,0,0,msg->motors.position[HDPCConst::STEERING_REAR_LEFT],
                    msg->header.stamp,"RLSteering","RLAxle");
            sendtf(0,0,0,0,0,msg->motors.position[HDPCConst::STEERING_FRONT_LEFT],
                    msg->header.stamp,"FLSteering","FLAxle");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_REAR_LEFT],0,
                    msg->header.stamp,"RLAxle","RLWheel");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_MIDDLE_LEFT],0,
                    msg->header.stamp,"CLAxle","CLWheel");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_FRONT_LEFT],0,
                    msg->header.stamp,"FLAxle","FLWheel");


            sendtf(0,geom.rover_width/2,0,
                    0,msg->motors.analog[HDPCConst::DRIVE_MIDDLE_RIGHT],0, 
                    msg->header.stamp,"body","RightRocker");
            sendtf(geom.rover_rocker_to_front_axle,0,-(geom.rover_rocker_to_boggie_z+geom.rover_boggie_to_axle_z),
                    msg->header.stamp,"RightRocker","FRSteering");
            sendtf(-geom.rover_rocker_to_boggie_x,0,-geom.rover_rocker_to_boggie_z,
                    0,msg->motors.analog[HDPCConst::DRIVE_REAR_RIGHT],0, 
                    msg->header.stamp,"RightRocker","RightBoggie");
            sendtf(geom.rover_boggie_to_middle_axle,0,-geom.rover_boggie_to_axle_z,
                    msg->header.stamp,"RightBoggie","CRAxle");
            sendtf(-geom.rover_boggie_to_rear_axle,0,-geom.rover_boggie_to_axle_z,
                    msg->header.stamp,"RightBoggie","RRSteering");
            sendtf(0,0,0,0,0,msg->motors.position[HDPCConst::STEERING_REAR_RIGHT],
                    msg->header.stamp,"RRSteering","RRAxle");
            sendtf(0,0,0,0,0,msg->motors.position[HDPCConst::STEERING_FRONT_RIGHT],
                    msg->header.stamp,"FRSteering","FRAxle");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_REAR_RIGHT],0,
                    msg->header.stamp,"RRAxle","RRWheel");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_MIDDLE_RIGHT],0,
                    msg->header.stamp,"CRAxle","CRWheel");
            sendtf(0,0,0,0,msg->motors.position[HDPCConst::DRIVE_FRONT_RIGHT],0,
                    msg->header.stamp,"FRAxle","FRWheel");

            unsigned int objectId = 0;
            visualization_msgs::MarkerArray ma;
            visualization_msgs::Marker mk;
            tf::Quaternion q;

            mk.header.stamp=msg->header.stamp;
            mk.ns = "rover";

            mk.header.frame_id="body";
            mk.id = objectId++;
            mk.type = visualization_msgs::Marker::CUBE;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 1.0; 
            mk.scale.y = geom.rover_width * 0.9; 
            mk.scale.z = 0.1; 
            mk.color.r = 0;
            mk.color.g = 0.2;
            mk.color.b = 1;
            mk.color.a = 0.5;
            ma.markers.push_back(mk);

            mk.header.frame_id="LeftRocker";
            mk.id = objectId++;
            mk.type = visualization_msgs::Marker::LINE_STRIP;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 0.05; 
            mk.scale.y = 0.0;
            mk.scale.z = 0.0; 
            mk.color.r = 0.7;
            mk.color.g = 0.7;
            mk.color.b = 0.7;
            mk.color.a = 0.5;
            mk.points.resize(3);
            mk.points[0].x = geom.rover_rocker_to_boggie_x;
            mk.points[0].y = 0.0;
            mk.points[0].z = -geom.rover_rocker_to_boggie_z;
            mk.points[1].x = 0.0;
            mk.points[1].y = 0.0;
            mk.points[1].z = 0.0;
            mk.points[2].x = geom.rover_rocker_to_front_axle;
            mk.points[2].y = 0.0;
            mk.points[2].z = -(geom.rover_rocker_to_boggie_z-geom.rover_boggie_to_axle_z);
            ma.markers.push_back(mk);

            mk.header.frame_id="RightRocker";
            mk.id = objectId++;
            ma.markers.push_back(mk);

            mk.header.frame_id="LeftBoggie";
            mk.id = objectId++;
            mk.type = visualization_msgs::Marker::LINE_STRIP;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 0.05; 
            mk.scale.y = 0.0;
            mk.scale.z = 0.0; 
            mk.color.r = 0.5;
            mk.color.g = 0.5;
            mk.color.b = 0.9;
            mk.color.a = 0.5;
            mk.points.resize(4);
            mk.points[0].x = -geom.rover_boggie_to_rear_axle;
            mk.points[0].y = 0.0;
            mk.points[0].z = -geom.rover_boggie_to_axle_z;
            mk.points[1].x = -geom.rover_boggie_to_rear_axle;
            mk.points[1].y = 0.0;
            mk.points[1].z = 0.0;
            mk.points[2].x = geom.rover_boggie_to_middle_axle;
            mk.points[2].y = 0.0;
            mk.points[2].z = 0.0;
            mk.points[3].x = geom.rover_boggie_to_middle_axle;
            mk.points[3].y = 0.0;
            mk.points[3].z = -geom.rover_boggie_to_axle_z;
            ma.markers.push_back(mk);

            mk.header.frame_id="RightBoggie";
            mk.id = objectId++;
            ma.markers.push_back(mk);



            mk.type = visualization_msgs::Marker::CYLINDER;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = geom.rover_wheel_radius * 2.; 
            mk.scale.y = geom.rover_wheel_radius * 2.; 
            mk.scale.z = 0.01; 
            mk.color.r = 1.0;
            mk.color.g = 1.0;
            mk.color.b = 0;
            mk.color.a = 0.5;
            q.setRPY(M_PI/2,0,0);
            tf::quaternionTFToMsg(q,mk.pose.orientation);
            mk.header.frame_id="FLWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);
            mk.header.frame_id="FRWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);
            mk.header.frame_id="CLWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);
            mk.header.frame_id="CRWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);
            mk.header.frame_id="RLWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);
            mk.header.frame_id="RRWheel";
            mk.id = objectId++;
            ma.markers.push_back(mk);

            marker_pub.publish(ma);

        }

    public:

        HDPCVisualize(ros::NodeHandle & nh) : geom(nh) {
            status_sub = nh.subscribe("/hdpc_drive/status",1,&HDPCVisualize::statusCallback,this);
            marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers",1);
            nh.param("verbose",verbose,0);
        }
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"hdpc_visualize");
    ros::NodeHandle nh("~");

    HDPCVisualize driver(nh);

    ros::spin();

    return 0;
}




