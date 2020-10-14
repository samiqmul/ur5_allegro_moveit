#include <signal.h>
#include <memory>
#include <kdl_control_tools/progress_logger.h>


#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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


#include "kdl_control_tools/WrenchArray.h"

using namespace std;
using namespace tf2;

ros::Publisher pub_force;


tf2::Vector3 finger0_vect;
tf2::Vector3 finger1_vect;
tf2::Vector3 finger2_vect;
tf2::Vector3 finger3_vect;

// tf naming params
string kinect2_frame_;
string hand_tf_name_;
string world_frame_;
double  safe_distance = 0.03;
double thumb_y_;
double  diff1;
double  diff3;
double z_mid_boundarybox ;

  tf2::Vector3 finger_vect;
  vector<tf2::Vector3> finger_array;  //finger_vect[3] is thumb
  vector<geometry_msgs::Vector3> opto_vect;
  tf2::Vector3 flag ;


void opto0Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
  // ROS_INFO_STREAM("Inside opto0_callback: " );
      opto_vect.resize(4);
  opto_vect[0] = msg->wrench.force;

  if (opto_vect[0].x >1.5 || opto_vect[0].y>1.5 || opto_vect[0].z >1.5)
  {
  flag[0] =1;
  }
  // else
  // {
  //   flag[0] =0;
  // }

ROS_INFO_STREAM("flag[0]: " << flag[0]);

    }
void opto2Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {

  opto_vect.resize(4);
// ROS_INFO_STREAM("Inside opto2_callback: " );
    opto_vect.resize(4);
  opto_vect[2] = msg->wrench.force;

  // ROS_INFO_STREAM("opto_vect[2]: "<< opto_vect[2].x  << "  -  "<<  opto_vect[2].y  << "  -    "<< opto_vect[2].z );

      if (opto_vect[2].x >1.5 || opto_vect[2].y>1.5 || opto_vect[2].z >1.5)
      {
      flag[2] =1;
      }
      // else
      // {
      //   flag[2] =0;
      // }

  ROS_INFO_STREAM("flag[2]: " << flag[2]);

    }
void opto1Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
  // ROS_INFO_STREAM("Inside opto1_callback: " );
    opto_vect.resize(4);
  opto_vect[1] = msg->wrench.force;

  // ROS_INFO_STREAM("opto_vect[1]: "<< opto_vect[1].x  << "  -  "<<  opto_vect[1].y  << "  -    "<< opto_vect[1].z );
  if (opto_vect[1].x >1.5 || opto_vect[1].y>1.5 || opto_vect[1].z >1.5)
  {
  flag[1] =1;
  }
  // else
  // {
  //   flag[1] =0;
  // }
  ROS_INFO_STREAM("flag[1]: " << flag[1]);
    }
void opto3Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg )
    {

    opto_vect.resize(4);
  // ROS_INFO_STREAM("Inside opto3_callback: " );
    opto_vect[3] = msg->wrench.force;

    if (opto_vect[3].x >1.5 || opto_vect[3].y>1.5 || opto_vect[3].z >1.5)
    {
    flag[3] =1;
    }
    // else
    // {
    //   flag[3] =0;
    // }
  ROS_INFO_STREAM("flag[3]: " << flag[3]);

}


void forcecallback(const kdl_control_tools::WrenchArray::ConstPtr &msg_forces){
  // ROS_INFO_STREAM("function forcecallback "  );

    vector<geometry_msgs::Vector3> force_vec;
  vector<geometry_msgs::Vector3> wrench_vec(4);

  // create a wrench vec for each finger
  // vector<vector<double> > wrench_vec(4);
  for(int fi=0; fi < 4; fi++){

    wrench_vec.resize(4);

    // add to wrench vector to publish
    wrench_vec[fi] = msg_forces->wrenches[fi].force;
      force_vec.push_back(wrench_vec[fi]);
    // wrench_vec[fi].y = msg_forces->wrenches[fi].force.y;
    // wrench_vec[fi].z = msg_forces->wrenches[fi].force.z;
    // ROS_INFO_STREAM("Inside wrench_vec "  << wrench_vec[fi] );

    ROS_INFO_STREAM("Inside forcecallback "  << wrench_vec[fi].x << wrench_vec[fi].y <<wrench_vec[fi].z );
  }


    // for(int fi=0; fi < 4; fi++){
    //
    //   if (flag[fi] ==0)
    //   {force_vec[fi] =force_vec[fi];}
    //
    //   else
    //
    //   {force_vec[fi].x =0;
    //     force_vec[fi].y =0;
    //     force_vec[fi].z =0;
    //
    //   }}


    kdl_control_tools::WrenchArray pub_forces;
    pub_forces.wrenches.resize(4);
    for(int fi=0; fi < 4; fi++){
    pub_forces.wrenches[fi].force  = force_vec[fi] ;}

for(int fi=0; fi < 4; fi++){
    if (flag[fi] ==1)
    {pub_forces.wrenches[fi].force.x =0;
      pub_forces.wrenches[fi].force.y =0;
      pub_forces.wrenches[fi].force.z =0;

    }
    else
    {pub_forces.wrenches[fi].force =pub_forces.wrenches[fi].force;}}

      pub_force.publish(pub_forces);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "cartesian_closegrip");
  ros::NodeHandle nh;

    // attempt to get ros parametersopto_vect
    // if(!getParams(nh)) return -1;

    ROS_INFO_STREAM("After getparam");



    //Subscribe to forces of optoforce
    ros::Subscriber opto_0 = nh.subscribe("/optoforce_wrench_0", 1, opto0Callback);
    ros::Subscriber opto_1 = nh.subscribe("/optoforce_wrench_1", 1, opto1Callback);
    ros::Subscriber opto_2 = nh.subscribe("/optoforce_wrench_2", 1, opto2Callback);
    ros::Subscriber opto_3 = nh.subscribe("/optoforce_wrench_3", 1, opto3Callback);

    ros::Subscriber cart_forces = nh.subscribe("publish_forces", 1, forcecallback); // allegro_hand_kdl/cartesian_find_points.cpp


    // ros::Rate loop_rate(5);
    // loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO_STREAM("After subscribe");

  pub_force = nh.advertise<kdl_control_tools::WrenchArray>  ("cartesian_forces",    1);

   ROS_INFO_STREAM("After advertise");

   // timerCallback();
   // ros::Timer timer = nh.createTimer(ros::Duration(0.005), timerCallback);
   ROS_INFO_STREAM("After timer");





  ros::spin();

return 0;

}
