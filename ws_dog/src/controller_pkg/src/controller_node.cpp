#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <nav_msgs/Path.h>

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

#define VEL_CONTROL 0
class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber desired_state, current_state, cmd_vel,current_goal;
  ros::Publisher prop_speeds;
  ros::Timer timer;


  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  // Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  // std::vector<std::vector<double>> xd;
  double xd;
  double yd;

  double yawd;           // desired yaw angle
  double hz;             // frequency of the main control loop
  double forward_vel;
  double angle; 
  double x_current;
  double y_current;
  //current goal
  // Eigen::Vector3d x_goal;
  double x_goal;
  double y_goal;
  double start;
  double now;
  double timeperiod;

public:
  controllerNode():hz(1000.0){
      
      desired_state = nh.subscribe("move_base/TrajectoryPlannerROS/global_plan", 1, &controllerNode::onDesiredState, this);
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
      cmd_vel = nh.subscribe("cmd_vel", 10, &controllerNode::vel_controller, this);
      current_goal = nh.subscribe("current_goal", 10, &controllerNode::goal_controller, this);
      start=ros::Time::now().toSec();    
  }


  void onDesiredState(const nav_msgs::Path& des_state){
      // for(int i=0;i<10;i++){
      //   std::vector<double> temp;
      //   temp.push_back(des_state.poses[i].pose.position.x);
      //   temp.push_back(des_state.poses[i].pose.position.y);
      //   temp.push_back(des_state.poses[i].pose.position.z);
      //   xd.push_back(temp);
      //   ROS_INFO_STREAM("Point x:"<<temp[0]<<"y:"<<temp[1]<<"z:"<<temp[2]<<"\n");
      //   ROS_INFO_STREAM("Point x:"<<xd[i][0]<<"y:"<<xd[i][1]<<"z:"<<xd[i][2]<<"\n");
      // } 
      // xd<<des_state.poses[5].pose.position.x,des_state.poses[5].pose.position.y,des_state.poses[5].pose.position.z;
      xd = des_state.poses[5].pose.position.x;
      yd = des_state.poses[5].pose.position.y;
      // ROS_INFO_STREAM("Point x:"<<xd[1][0]<<"y:"<<xd[1][1]<<"z:"<<xd[1][2]<<"\n");
      // tf::Quaternion q;
      // tf::quaternionMsgToTF(des_state.transforms[0].rotation , q);
      // yawd = tf::getYaw(q);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    x_current = cur_state.pose.pose.position.x;
    y_current = cur_state.pose.pose.position.y;

    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();

    // Rotate omega
    omega = R.transpose()*omega;
  }

  // use the topic "/cmd_vel" to control
  void vel_controller(const geometry_msgs::Twist& vel){
      

    if( vel.linear.y > 0){
      forward_vel = 1;
      angle = 4;
    }
    else if (vel.linear.y < 0)
    {
      forward_vel = 1;
      angle = -4;
    }
    else{
      forward_vel = 1;
      angle =0;
    }

    
  }

  void goal_controller(const geometry_msgs::PoseStamped& goal){
    // x_goal << goal.pose.position.x, goal.pose.position.y, goal.pose.position.z;
    x_goal = goal.pose.position.x;
    y_goal = goal.pose.position.y;
  }
  
  void controlLoop(const ros::TimerEvent& t){
    
    double ex,ey;
    now = ros::Time::now().toSec();
    timeperiod=now-start;
    ex = xd-x_current;
    ey = yd-y_current;
    // ROS_INFO_STREAM("Point x:"<<xd<<"y:"<<yd<<"\n");
    if(timeperiod<40 && timeperiod>140){
      forward_vel = 1;
      if(ex<=0.02 && ex>=-0.02)
        angle=0;
      else 
        angle =ex*50;
    }
    // change the direction
    else if(timeperiod<=140 && timeperiod >=40){
      if(ey<=0.02 && ey>=-0.02)
        angle=0;
      else 
        angle =ey*50;
    }
    
    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = forward_vel; // Forward Velocity (max Value: 1m/s)
    msg.angular_velocities[1] = angle; // Turning angle (max Value 4 - does not relate to a unit)
    msg.angular_velocities[2] = 1; // Amplitude
    msg.angular_velocities[3] = 0; // not used
    prop_speeds.publish(msg);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
