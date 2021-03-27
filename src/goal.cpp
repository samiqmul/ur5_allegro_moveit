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
// ros::Publisher torque_values;

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
ros::ServiceClient torquegrip_request;
ros::ServiceClient movenext_request;
ros::ServiceClient torquenvelop_request;

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
  intensity_value.data =1;
  envelop_intensity.publish(intensity_value);

  // vector<geometry_msgs::Vector3> force_vec;
  // vector<geometry_msgs::Vector3> wrench_vec(4);
  // for(int fi=0; fi < 4; fi++){
  //   wrench_vec.resize(4);
  //   wrench_vec[fi] = msg_forces->wrenches[fi].force;
  //   force_vec.push_back(wrench_vec[fi]);
  //
  // pub_forcesenv.wrenches.resize(4);
  // for(int fi=0; fi < 4; fi++){
  // pub_forcesenv.wrenches[fi].force  = force_vec[fi] ;}
  // pub_force.publish(pub_forcesenv);

}


void torque_close()
{
  grip = "inloop";
  torquegrip_request.waitForExistence();
  torquegrip_request.call(srv);
}

void torque_envelop()
{
  grip = "inloop";
  torquenvelop_request.waitForExistence();
  torquenvelop_request.call(srv);
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
 envelop_intensity = node_handle.advertise<std_msgs::Float64>("/envelop_intensity", 3); //envelop_force.cpp
 // torque_values = node_handle.advertise<sensor_msgs::JointState>("/manipulation_torque",1);
 // cartenv_forces = node_handle.advertise<std_msgs::Float64>("publishenv_forces",    1); //to cartesisan close grip


 clientgrip_request = node_handle.serviceClient<std_srvs::Empty>("/grip_request"); //to gradual_grip node
 torquegrip_request = node_handle.serviceClient<std_srvs::Empty>("/torquegrip_request"); //to gradual_grip node
 // torquenvelop_request = node_handle.serviceClient<std_srvs::Empty>("/torquenvelop_request"); //to gradual_grip node

 // movenext_request = node_handle.serviceClient<std_srvs::Empty>("/movenext_request");


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
      // move_group.setPlannerId("KPIECE");

        geometry_msgs::Pose target_pose1;

  geometry_msgs::Pose home_pose = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose;
  ROS_INFO_STREAM("Home: " << home_pose);
  /*KINECT :    RED	X   BLUE  Z      GREEN 	Y
    WORLD  :    RED X   BLUE	Z      GREEN	Y   (-90)  */



  // target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(a_current, b_current,c_current);
  // target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.1, 0.1,0.1);

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
  target_pose1.position.x =  aa_current - 0.06; //decrease 5cm because of allegro length
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

    while(answer != "y"){
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Safe to execute?(y/n)\n";
      std::cin >> answer;
    }
    move_group.execute(my_plan);

  //~~~#################################################################
  //~~~####################   ROTATE   ##################################
    std::map<std::string, double> joints;
    joints["shoulder_pan_joint"] = 2.51;
    joints["shoulder_lift_joint"] = -2.51;
    joints["elbow_joint"] = -2;
    joints["wrist_1_joint"] = -0.05;
    joints["wrist_2_joint"] = 1.45;
    joints["wrist_3_joint"] = 0.17;
    ros::spinOnce();
    cloud_cb(target_pose1);
    move_group.setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan my_planO;
    std::string answerO = "";
    while(answerO != "y"){
      bool success = (move_group.plan(my_planO) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Safe for orientation?(y/n)\n";
      std::cin >> answerO;
    }
    move_group.execute(my_planO);



  //~~~#################################################################
    //~~~###############     LOWER     ################################
  //~~~#################################################################
  // target_pose1.orientation.x =0.701;
  // target_pose1.orientation.y = -0.320;
  // target_pose1.orientation.z =-0.588;
  // target_pose1.orientation.w = 0.247;
  target_pose1.position.x = initial_pose[0]  ;
  target_pose1.position.y = initial_pose[1]  ;
  target_pose1.position.z = initial_pose[2] - 0.08  ; // 0.12 foam, 0.05 bottle coming back to orginal position

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
  //~~~####################   ROTATE   ##################################
  // std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = 2.39;
  joints["shoulder_lift_joint"] = -2.57;
  joints["elbow_joint"] = -1.82;
  joints["wrist_1_joint"] = -0.21;
  joints["wrist_2_joint"] = 1.49;
  joints["wrist_3_joint"] = 0.06;
  ros::spinOnce();
  cloud_cb(target_pose1);
  move_group.setJointValueTarget(joints);
  moveit::planning_interface::MoveGroupInterface::Plan my_planOA;
  std::string answerOA = "";
  while(answerOA != "y"){
    bool success = (move_group.plan(my_planOA) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "Safe for orientation?(y/n)\n";
    std::cin >> answerOA;
  }
  move_group.execute(my_planOA);



  //~~~#################################################################
  //~~~####################   TRANSLATE  ##################################

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
  while(answerC != "y"){
    std::cout << "Move sideways?(y/n)\n";
    std::cin >> answerC;
  }
    sleep(1.0);
  move_group.execute(my_planC);
  waypoints.pop_back();


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
    torque_close();
    while(grip != "y"){
      std::cout << "Envelop Intensity?(y/n)\n";
      std::cin >> grip;                        }
    envelop();
    // torque_envelop();
    std::cout << "Finished opening/closing of Grasp?(fin/n)\n";
    std::cin >> grip;  }
    opengripper();
    grip = "again_loop";

      //
      // for (int j=0; j<25 ; j++)
      // {
      // intensity_value.data =0;
      // envelop_intensity.publish(intensity_value);
      // opengripper();
      // sleep(1.0);
      // torque_close();
      // sleep(1.0);
      // envelop();
      // movenext_request.call(srv);
      //send empty request to movepoint to ask for next point
      //movepoint receives metric score and sends to bopt.
      //bopt sends next point to movepoint,
      //movepoint asks robot to move next point
      // opengripper();
      // sleep(1.0);
      // robot moves sideways}


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
