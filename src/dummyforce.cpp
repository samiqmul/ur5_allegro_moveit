#include "kdl_control_tools/WrenchArray.h"
#include <kdl_control_tools/progress_logger.h>


#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#include "std_msgs/String.h"
#include <sstream>
#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

tf2::Vector3 thumb_vect;
tf2::Vector3 finger1_point;
tf2::Vector3 finger2_vect;
tf2::Vector3 finger3_point;
ros::Publisher cart_forces;
vector<tf2::Vector3> target_value_sensor ;
void createForceVec(geometry_msgs::Vector3 & from,  geometry_msgs::Vector3 & to,   geometry_msgs::Vector3 & force_vec){
  // ROS_INFO_STREAM("Inside create force vec");
    force_vec.x = to.x - from.x;
    force_vec.y = to.y - from.y;
    force_vec.z = to.z - from.z;
    // normalize vector and scale
  //   double magnitude = sqrt(
  //     force_vec.x*force_vec.x +
  //     force_vec.y*force_vec.y +
  //     force_vec.z*force_vec.z);
  //   force_vec.x /= magnitude;
  //   force_vec.y /= magnitude;
  //   force_vec.z /= magnitude;
  //   force_vec.x *= 2.1;
  //   force_vec.y *= 2.1;
  //   force_vec.z *= 2.1;
  }

void publishforces()
{

    vector<geometry_msgs::Vector3> force_vec;
    geometry_msgs::Vector3 tmp_vec;
    geometry_msgs::Vector3 target_value_sensors[4];



    tf2::Vector3 center_points (0.118,0.390,1.017);


    geometry_msgs::Vector3 center_point = toMsg (center_points);

   geometry_msgs::Pose center_pose;
   center_pose.position.x = center_points[0];
   center_pose.position.y = center_points[1] ;
   center_pose.position.z = center_points[2] ;
   ros::spinOnce();


    // create a force vector from each fingertip to the center
    for(int fi=0; fi<4; fi++){
      target_value_sensors[fi] = toMsg(target_value_sensor[fi]);
      createForceVec(target_value_sensors[fi], center_point, tmp_vec);
      // ROS_INFO_STREAM("After createforce");
      force_vec.push_back(tmp_vec); // concat to force_vec
}
    kdl_control_tools::WrenchArray msg_forces;
    msg_forces.wrenches.resize(4);

    for(int fi=0; fi < 4; fi++){
    // add to wrench vector to publish
    // ROS_INFO_STREAM("Inside msg_forces.wrenches");

         msg_forces.wrenches[fi].force  = force_vec[fi] ;


      }
      // ROS_INFO_STREAM("After for msg_forces.wrenches");

      cart_forces.publish(msg_forces);
      ROS_INFO_STREAM("After cart_forces.publish"<<msg_forces);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "dummyforce");
  ros::NodeHandle nh;

  cart_forces = nh.advertise<kdl_control_tools::WrenchArray> ("publish_forces",    1);

  tf2::Vector3 finger1_vect(0.120,0.380,1.027);
  tf2::Vector3 finger2_vect(0.130,0.390,1.037);
  tf2::Vector3 finger3_vect(0.110,0.400,1.047);
  tf2::Vector3 thumb_vect(0.125,0.370,1.057);



  target_value_sensor.push_back ( finger1_vect);
  target_value_sensor.push_back ( finger2_vect);
  target_value_sensor.push_back ( finger3_vect);
  target_value_sensor.push_back ( thumb_vect);

  while (nh.ok()){
publishforces();
}

return 0;

}
