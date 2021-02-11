#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <std_srvs/Empty.h>


#include <allegro_hand_kdl/pose_control_client.h>
using namespace std;
using namespace allegro_hand_kdl;
shared_ptr<JointPoseClient> joint_client;
//check git atom

using namespace tf2;
// std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double x_current = 0;
double y_current = 0;
double z_current = 0;
double a_current = 0;
double b_current = 0;
double c_current = 0;
double w_current = 0;
double aa_current = 0;
double bb_current = 0;
double cc_current = 0;

tf2::Vector3 initial_pose ;
tf2::Vector3 next_pose ;

ros::Publisher arrow_pub;
ros::Publisher envelop_intensity;
ros::Publisher torque_values;

float object_length;

//Horizontal
float orientationX =0.65941;
float orientationY =  -0.29462;
float orientationZ = -0.63983;
float orientationW = 0.26268;

// Oriented
float orientationOX = 0.707515;
float orientationOY = -0.319556;
float orientationOZ = -0.58807;
float orientationOW = 0.226891;

std_msgs::Float64 intensity_value;

// float orientationX =0.67953;
// float orientationY = -0.24508;
// float orientationZ =-0.65737;
// float orientationW =0.21457;

ros::ServiceClient clientgrip_request;
std_srvs::Empty srv;
std::string grip = "";

void cloud_cb (geometry_msgs::Pose& desPose) {

  visualization_msgs::Marker arrow;

  // arrow.header.frame_id = "/kinect2_rgb_optical_frame";
  arrow.header.frame_id = "/world";
  arrow.header.stamp = ros::Time::now();

  arrow.ns = "example";
  arrow.id = 1;

  arrow.type = visualization_msgs::Marker::SPHERE;
  arrow.action = visualization_msgs::Marker::ADD;

   arrow.pose.position.x = desPose.position.x;
   arrow.pose.position.y = desPose.position.y;
   arrow.pose.position.z = desPose.position.z;


 arrow.pose.orientation.x = -0.9914449;
 arrow.pose.orientation.y = 0.0;
 arrow.pose.orientation.z = 0.0;
 arrow.pose.orientation.w = 0.1305262;

 arrow.scale.x= 0.03;
 arrow.scale.y= 0.03;
 arrow.scale.z = 0.03;

 arrow.color.g = 0.0f;
 arrow.color.a = 1.0;
 arrow.color.r = 0.0f;
 arrow.color.b = 1.0f;

 arrow.lifetime = ros::Duration();

 arrow_pub.publish(arrow);

 // ROS_INFO_STREAM("arrow_pub POINTS: " << arrow);
}




void poseCallback (const geometry_msgs::Pose::ConstPtr &msg ) {
  /*
    //from query: https://answers.ros.org/question/223599/how-to-subscribe-to-a-topic-while-publishing-to-another/: geometry_msgs::Pose current_pose = msg->pose;
    //from tutorial: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29: ROS_INFO("I heard: [%s]", msg->data.c_str());
  //from query : https://answers.ros.org/question/281213/read-variable-from-posestamped-c/  :-
  ROS_INFO_STREAM("Received pose: " << msg);
  */
  x_current = msg->position.x;
  y_current = msg->position.y;
  z_current = msg->position.z;
  w_current = msg->orientation.w;
  a_current = msg->orientation.x;
  b_current = msg->orientation.y;
  c_current = msg->orientation.z;

   // ROS_INFO_STREAM("Function A orientation: " << a_current);

}


void boxCallback(const geometry_msgs::Point::ConstPtr& bmsg ) {
  //void boxCallback(const geometry_msgs::PoseStamped::ConstPtr &bmsg )

   //ROS_INFO_NAMED("tutorial", "INSIDE BOXCALLBACK");

  aa_current = bmsg->x ;
  bb_current =  bmsg->y;
  cc_current =  bmsg->z;

 // ROS_INFO_STREAM("Function AA Pose: " << aa_current);

return;
}

void nextpointCallback(const geometry_msgs::Point::ConstPtr& bmsg ) {

  next_pose[0] = bmsg->x ;
  next_pose[1] =  bmsg->y;
  next_pose[2] =  bmsg->z;

}

void lengthCallback(const std_msgs::Float32::ConstPtr& length) //length of the object
{

  object_length= (length->data);  //in meters
}

void opengripper()
{
  grip = "inloop";
  intensity_value.data =0;
  envelop_intensity.publish(intensity_value);
  joint_client->setTargetName("next");
  joint_client->move();
}

void closegripper ()
{
  grip = "inloop";
  clientgrip_request.waitForExistence();
  clientgrip_request.call(srv);
}

void envelop()
{
    grip = "inloop";
  intensity_value.data =1.7;
  envelop_intensity.publish(intensity_value);

}


void torque_close()
{
  float desired_torques[16]= {0.011171566878036592, -0.06565917703674992, 0.043267385495564157, 0.006338279346367652, 0.012411914363948586, -0.014722661841737615, 0.044705850478346076, 0.015110713000407687, 0.009759421213981764, -0.13483119390580756, -0.005983175336968394, 0.0038796565316640786, -0.19093140327472863, -0.01673824315101389, 0.0020234336393371265, 0.016489904354828434};
  // Create a JointState msg for torques
  sensor_msgs::JointState msg_out;
  msg_out.header.stamp = ros::Time::now();
  msg_out.position.resize(16);
  msg_out.velocity.resize(16);
  msg_out.effort.resize(16);
  for (int j=0; j < 16; j++){
  //   // scale with constant
    msg_out.effort[j] = desired_torques[j];    // names of joints
    msg_out.name.push_back("joint_"+to_string(j));
  }  // send!

  torque_values.publish(msg_out);

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_ur5");

  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // waiting for rviz and planners to initialize
 sleep(8.0);

 ros::Subscriber sub2 = node_handle.subscribe("/d/controlmidbox_box", 1, boxCallback);
 ros::Subscriber sub3 = node_handle.subscribe("/rank/next_point", 1, nextpointCallback);    // from probablistic/bayesian.cpp
 ros::Subscriber sub = node_handle.subscribe("/d/controloutput_pose", 1, poseCallback);
 // ros::Subscriber sub = node_handle.subscribe(" /find_object/object_pose", 1, poseCallback);
 ros::Subscriber sub4 = node_handle.subscribe("/d/object_length", 1, lengthCallback);

 arrow_pub = node_handle.advertise<visualization_msgs::Marker> ("arrow",1);
 envelop_intensity = node_handle.advertise<std_msgs::Float64>("/envelop_intensity", 3);
 torque_values = node_handle.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd",1);

 clientgrip_request = node_handle.serviceClient<std_srvs::Empty>("/grip_request");
 joint_client =  make_shared<JointPoseClient>();


 ROS_INFO_NAMED("tutorial", "Subscribed to output_pose");

  // static const std::string PLANNING_GROUP = "manipulator";
  static const std::string PLANNING_GROUP = "ur5_arm";


  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_group.setMaxVelocityScalingFactor(0.025);
  //move_group_interface::MoveGroup group("manipulator");
    // See ompl_planning.yaml for a complete list
  // group.setPlannerId("SBLkConfigDefault");


  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      // We can print the name of the reference frame for this robot.
      ROS_INFO_NAMED("tutorial", "UR5 Reference frame: %s", move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("tutorial", "UR5 End effector link: %s", move_group.getEndEffectorLink().c_str());


      move_group.allowReplanning(true);
      move_group.setPlanningTime(20);
      // move_group.setPlannerId("RRTConnectkConfigDefault");
      move_group.setPlannerId("KPIECE");

        geometry_msgs::Pose target_pose1;

  geometry_msgs::Pose home_pose = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose;
  ROS_INFO_STREAM("Home: " << home_pose);
  /*KINECT :    RED	X   BLUE  Z      GREEN 	Y
    WORLD  :    RED X   BLUE	Z      GREEN	Y   (-90)  */



  // target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(a_current, b_current,c_current);
  // target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.1, 0.1,0.1);

  // // In Small Table
  // target_pose1.position.x =  aa_current+ 0.09  ;
  // target_pose1.position.y =  0.39 + bb_current ;
  // target_pose1.position.z =   1.92 -cc_current  ; //added 20cm more



  // // WorKing but Oriented:
  // - Translation: [0.254, 0.475, 1.301]
  // - Rotation: in Quaternion [0.618, -0.378, -0.598, 0.344]
  //             in RPY (radian) [1.627, 0.499, -1.572]
  //             in RPY (degree) [93.201, 28.584, -90.073]


  //workiNG:
              // target_pose1.orientation.x = 0.613445;
              //  target_pose1.orientation.y = -0.38199;
              //  target_pose1.orientation.z =-0.597485;
              //  target_pose1.orientation.w = 0.347535;
               //

                  //WOLRD
  target_pose1.orientation.x =orientationX;
  target_pose1.orientation.y = orientationY;
  target_pose1.orientation.z =orientationZ;
  target_pose1.orientation.w =orientationW;
  target_pose1.position.x =  aa_current - 0.055; //decrease 5cm because of allegro length
  // target_pose1.position.y =    bb_current -0.05 ; //offset mid
  target_pose1.position.y =    0.350  ; //offset mid
  target_pose1.position.z =   cc_current + 0.1 ; //10 cm higher
  initial_pose[0]= target_pose1.position.x  ;
  initial_pose[1]= target_pose1.position.y   ;
  initial_pose[2]= target_pose1.position.z ;


   ros::spinOnce();
   cloud_cb(target_pose1);
   //move_group.setGoalTolerance(0.2);  //to	account	for	inaccuracies	in	the	control
   move_group.setPoseTarget(target_pose1);

    ROS_INFO_STREAM("Target Pose: " << target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO_STREAM("Going higher pos initially ");
    std::string answer = "";


    while(answer != "torque"){
      torque_close();
      std::cout << "Change torque values?(torque/n)\n";
      std::cin >> answer;
    }

    while(answer != "y"){

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      std::cout << "Safe to execute?(y/n)\n";
      std::cin >> answer;
    }


    move_group.execute(my_plan);

  //~~~#################################################################

  target_pose1.position.x = initial_pose[0]  ;
  target_pose1.position.y = initial_pose[1]  ;
  target_pose1.position.z = initial_pose[2] - 0.12  ; //coming back to orginal position

  ros::spinOnce();
  cloud_cb(target_pose1);
  move_group.setPoseTarget(target_pose1);
  move_group.setMaxVelocityScalingFactor(0.025);
  ROS_INFO_STREAM("Target Pose: " << target_pose1);



  moveit::planning_interface::MoveGroupInterface::Plan my_planC;
         std::string answerC = "";
         while(answerC != "y"){

           bool success = (move_group.plan(my_planC) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

           std::cout << "Safe to lower?(y/n)\n";
           std::cin >> answerC;
         }

    move_group.execute(my_planC);
  //~~~#################################################################
  //https://answers.ros.org/question/223090/start-or-stop-the-ros-node-in-another-node/


  // ////////////////////////////////  LOOP  START  ///////////////////////////////////////////////////////////////
  // ////////////////////////////////  LOOP  START  ///////////////////////////////////////////////////////////////
  // geometry_msgs::Pose initState = target_pose1;
  // std::vector<geometry_msgs::Pose> waypoints;
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 5;
  // const double eef_step = 0.01;
  //
  //   std::string target = "";
  //     while(target != "fin"){
  //   std::cout << "Move to Next stage?(fin)\n";
  //   std::cin >> target;
  //
  //   }
  //
  // std::string loop = "";
  //   while(loop != "fin"){
  //
  //
  // initState.position.y += (next_pose[1] - initState.position.y ) ;
  // waypoints.push_back(initState);
  // ros::spinOnce();
  // cloud_cb(initState);
  // ROS_INFO_STREAM("initState: " << initState);
  // // ROS_INFO_STREAM("waypoints: " << waypoints.point.x);
  //   move_group.setMaxVelocityScalingFactor(0.025);
  // move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // my_planC.trajectory_ = trajectory;
  // std::string answerC = "";
  // while(answerC != "y"){
  //   std::cout << "Move sideways?(y/n)\n";
  //   std::cin >> answerC;
  // }
  //
  // move_group.execute(my_planC);
  //
  // waypoints.pop_back();
  // std::cout << "Is loop finsihed?(fin)\n";
  // std::cin >> loop;
  //
  // }
  // ////////////////////////////////  LOOP end  ///////////////////////////////////////////////////////////////
  // ////////////////////////////////  LOOP end  ///////////////////////////////////////////////////////////////

  ////////////////////////////////  Velocities START  ///////////////////////////////////////////////////////////////
  ////////////////////////////////   Velocities  START  ///////////////////////////////////////////////////////////////







  geometry_msgs::Pose initState = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose;;
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group.setPlanningTime(10);
  const double jump_threshold = 5;
  const double eef_step = 0.01;

    std::string target = "";
      while(target != "fin"){
    std::cout << "Move to Next stage?(fin)\n";
    std::cin >> target;

    }

  std::string loop = "";
    while(loop != "home"){


  initState.position.y += (next_pose[1] - initState.position.y ) ;
  initState.position.z += (next_pose[2] - initState.position.z ) ;

  waypoints.push_back(initState);
  ros::spinOnce();
  cloud_cb(initState);
  ROS_INFO_STREAM("initState: " << initState);


  //https://groups.google.com/forum/#!msg/moveit-users/x5FwalM5ruk/Rb9BReCAfKQJ
  move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg,false);
  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "ur5_arm");
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt);
  rt.getRobotTrajectoryMsg(trajectory_msg);


  my_planC.trajectory_ = trajectory_msg;
  std::string answerC = "";
  // while(answerC != "fin"){
  while(answerC != "y"){
    std::cout << "Move sideways?(y/n)\n";
    std::cin >> answerC;
  }
    sleep(1.0);
  move_group.execute(my_planC);
  waypoints.pop_back();
  // std::cout << "Open close the hand?(y/n)\n";
  // std::cin >> answerC;
// }


  while(grip != "fin"){
    intensity_value.data =0;
    envelop_intensity.publish(intensity_value);
    while(grip != "y"){
      std::cout << "OPEN the Grasp?(y/n)\n";
      std::cin >> grip;                        }
    opengripper();
   while(grip != "y"){
     std::cout << "Close the Grasp?(y/n)\n";
     std::cin >> grip;                         }
   closegripper();
    while(grip != "y"){
      std::cout << "Envelop Intensity?(y/n)\n";
      std::cin >> grip;                        }
    envelop();
    std::cout << "Finished opening/closing of Grasp?(fin/n)\n";
    std::cin >> grip;  }
    opengripper();
    grip = "again_loop";

  std::cout << "Is loop finsihed?(home)\n";
  std::cin >> loop;

  }
  ////////////////////////////////   Velocities end  ///////////////////////////////////////////////////////////////
  ////////////////////////////////  Velocities end  ///////////////////////////////////////////////////////////////


 target_pose1 = home_pose;

  move_group.setPoseTarget(target_pose1);
  cloud_cb(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;

    std::string answerH = "";
    while(answerH != "y"){

      bool success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      std::cout << "Back to Home(y/n)\n";
      std::cin >> answerH;
    }
  move_group.setMaxVelocityScalingFactor(0.025);
    move_group.execute(home_plan);


   ros::waitForShutdown();

        return 0;
        }
