
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

/**
* This node will listen to allegro hand JointState,
* ur5 JointState and publish the merged JointState.
**/

using namespace std;

ros::Publisher pub_; // publish merged JointStates

sensor_msgs::JointState js_ur5_;
sensor_msgs::JointState js_allegro_;

bool overwrite_ = false;

void publishMerged(){

  if(js_ur5_.name.size()==0 || js_allegro_.name.size()==0)
    return;

  // create merged msg
  sensor_msgs::JointState js_merged;

  js_merged.header.stamp = ros::Time::now();

  // add joints from ur5
  int size_ur5 = js_ur5_.name.size();
  for (int j=0; j<size_ur5; j++){
    js_merged.name.push_back(js_ur5_.name[j]);
    js_merged.position.push_back(js_ur5_.position[j]);
    js_merged.velocity.push_back(js_ur5_.velocity[j]);
    js_merged.effort.push_back(js_ur5_.effort[j]);
  }
  // add joints from allegro
  int size_allegro = js_allegro_.name.size();
  for (int j=0; j<size_allegro; j++){
    
    if(overwrite_){
      js_merged.position[j] = js_allegro_.position[j];
      js_merged.velocity[j] = js_allegro_.velocity[j];
      js_merged.effort[j] = js_allegro_.effort[j];

    }else{
      js_merged.name.push_back(js_allegro_.name[j]);
      js_merged.position.push_back(js_allegro_.position[j]);
      js_merged.velocity.push_back(js_allegro_.velocity[j]);
      js_merged.effort.push_back(js_allegro_.effort[j]);
    }
  }

  pub_.publish(js_merged);

}

void ur5Callback(const sensor_msgs::JointState::ConstPtr &msg_in) {

  js_ur5_ = *msg_in;

  // ensure that all vectors are the same size
  if(js_ur5_.velocity.size() < js_ur5_.name.size())
    js_ur5_.velocity.resize(js_ur5_.name.size());

  if(js_ur5_.effort.size() < js_ur5_.name.size())
    js_ur5_.effort.resize(js_ur5_.name.size());

  publishMerged();
}


void allegroCallback(const sensor_msgs::JointState::ConstPtr &msg_in) {

  js_allegro_ = *msg_in;

  publishMerged();
}


int main(int argc, char **argv){

  ros::init(argc, argv, "joint_state_merger");
  ROS_INFO("Joint State Merger");

  ros::NodeHandle nh;

  // get overwrite param
  ros::param::get("~overwrite", overwrite_);

  ros::Subscriber sub_ur5 = nh.subscribe<sensor_msgs::JointState>( "ur5_js", 1, &ur5Callback);
  ros::Subscriber sub_allegro = nh.subscribe<sensor_msgs::JointState>( "allegro_js", 1, &allegroCallback);

  pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::spin();

  return 0;

}
