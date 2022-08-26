#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI M_PI
#define kp 2.58
#define kd 0.47
#define ks 0


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber desired_state, current_state, des_pos;
  ros::Publisher prop_speeds;
  ros::Timer timer;


  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  double position_error = 0;
  double previous__position_error = 0;
  double angle_error = 0;
  double previous__angle_error = 0;
  double sumdistance = 0;
  double sumangle = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  int i = 0;

public:
  controllerNode():hz(2){
      
      desired_state = nh.subscribe("desired_state", 1, &controllerNode::onDesiredState, this);
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      //des_pos = nh.subscribe("nav_path",10,&controllerNode::onPath, this);

      prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      
      // Position
      geometry_msgs::Vector3 t = des_state.transforms[0].translation;
      xd << t.x, t.y, t.z;
      // ROS_INFO_NAMED("onDesiredState", "POS: %f %f %f", t.x, t.y, t.z);

      // Velocities
      geometry_msgs::Vector3 v = des_state.velocities[0].linear;
      vd << v.x, v.y, v.z;
      // ROS_INFO_NAMED("onDesiredState", "VEL: %f %f %f", v.x, v.y, v.z);

      // Accelerations
      geometry_msgs::Vector3 a = des_state.accelerations[0].linear;
      ad << a.x, a.y, a.z;
      
      tf::Quaternion q;
      tf::quaternionMsgToTF(des_state.transforms[0].rotation , q);
      yawd = tf::getYaw(q);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(cur_state.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    R = q.toRotationMatrix();

    // Rotate omega
    omega = R.transpose()*omega;
  }

 


  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);

    if(i<=80)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=180)
    {
      msg.angular_velocities[0] = 0; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 4;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=280)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=335)
    {
      msg.angular_velocities[0] = 0; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 4;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=380)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=440)
    {
      msg.angular_velocities[0] = 0; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = -4;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=470)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=595)
    {
      msg.angular_velocities[0] = 0; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = -4;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=635)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else if (i<=735)
    {
      msg.angular_velocities[0] = 1; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 3; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }
    else
    {
      msg.angular_velocities[0] = 0; // Forward Velocity (max Value: 1m/s)
      msg.angular_velocities[1] = 0;; // Turning angle (max Value 4 - does not relate to a unit)
      msg.angular_velocities[2] = 1; // Amplitude
      msg.angular_velocities[3] = 0; // not used
    }

    prop_speeds.publish(msg);
    i++;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node_test");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
