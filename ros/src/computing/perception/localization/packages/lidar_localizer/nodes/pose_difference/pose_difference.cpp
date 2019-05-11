#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static pose current_gnss_pose, current_ndt_pose, current_diff_pose;
static tf::Quaternion diff_q;

static ros::Publisher pose_difference_pub;
static geometry_msgs::PoseStamped pose_difference_msg;

//ros::Time current_time;

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
    tf::Matrix3x3 gnss_m(gnss_q);

    current_gnss_pose.x = input->pose.position.x;
    current_gnss_pose.y = input->pose.position.y;
    current_gnss_pose.z = input->pose.position.z;
    gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

    // Updated in initialpose_callback or gnss_callback
    current_diff_pose.x = current_gnss_pose.x-current_ndt_pose.x;
    current_diff_pose.y = current_gnss_pose.y-current_ndt_pose.y;
    current_diff_pose.z = current_gnss_pose.z-current_ndt_pose.z;
    current_diff_pose.roll = calcDiffForRadian(current_gnss_pose.roll, current_ndt_pose.roll);
    current_diff_pose.pitch = calcDiffForRadian(current_gnss_pose.pitch, current_ndt_pose.pitch);
    current_diff_pose.yaw = calcDiffForRadian(current_gnss_pose.yaw, current_ndt_pose.yaw);

    diff_q.setRPY(current_diff_pose.roll, current_diff_pose.pitch, current_diff_pose.yaw);
    tf::Vector3 v(current_diff_pose.x, current_diff_pose.y, current_diff_pose.z);
    tf::Transform transform(diff_q, v);
    pose_difference_msg.header.frame_id = "/map";
    pose_difference_msg.header.stamp = input->header.stamp;
    pose_difference_msg.pose.position.x = current_diff_pose.x;
    pose_difference_msg.pose.position.y = current_diff_pose.y;
    pose_difference_msg.pose.position.z = current_diff_pose.z;
    pose_difference_msg.pose.orientation.x = diff_q.x();
    pose_difference_msg.pose.orientation.y = diff_q.y();
    pose_difference_msg.pose.orientation.z = diff_q.z();
    pose_difference_msg.pose.orientation.w = diff_q.w();

    pose_difference_pub.publish(pose_difference_msg);    
}

static void ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    tf::Quaternion ndt_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
    tf::Matrix3x3 ndt_m(ndt_q);

    current_ndt_pose.x = input->pose.position.x;
    current_ndt_pose.y = input->pose.position.y;
    current_ndt_pose.z = input->pose.position.z;
    ndt_m.getRPY(current_ndt_pose.roll, current_ndt_pose.pitch, current_ndt_pose.yaw);

    // Updated in initialpose_callback or gnss_callback
    current_diff_pose.x = current_gnss_pose.x-current_ndt_pose.x;
    current_diff_pose.y = current_gnss_pose.y-current_ndt_pose.y;
    current_diff_pose.z = current_gnss_pose.z-current_ndt_pose.z;
    current_diff_pose.roll = calcDiffForRadian(current_gnss_pose.roll, current_ndt_pose.roll);
    current_diff_pose.pitch = calcDiffForRadian(current_gnss_pose.pitch, current_ndt_pose.pitch);
    current_diff_pose.yaw = calcDiffForRadian(current_gnss_pose.yaw, current_ndt_pose.yaw);

    diff_q.setRPY(current_diff_pose.roll, current_diff_pose.pitch, current_diff_pose.yaw);
    tf::Vector3 v(current_diff_pose.x, current_diff_pose.y, current_diff_pose.z);
    tf::Transform transform(diff_q, v);
    pose_difference_msg.header.frame_id = "/map";
    pose_difference_msg.header.stamp = input->header.stamp;
    pose_difference_msg.pose.position.x = current_diff_pose.x;
    pose_difference_msg.pose.position.y = current_diff_pose.y;
    pose_difference_msg.pose.position.z = current_diff_pose.z;
    pose_difference_msg.pose.orientation.x = diff_q.x();
    pose_difference_msg.pose.orientation.y = diff_q.y();
    pose_difference_msg.pose.orientation.z = diff_q.z();
    pose_difference_msg.pose.orientation.w = diff_q.w();

    pose_difference_pub.publish(pose_difference_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_difference");

    ros::NodeHandle nh;

    //ros::Rate loop_rate(0.5);


    // Subscribers

    ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
    ros::Subscriber ndt_pose_sub = nh.subscribe("ndt_pose", 10, ndt_pose_callback);

    // // Updated in initialpose_callback or gnss_callback
    // current_diff_pose.x = current_gnss_pose.x-current_ndt_pose.x;
    // current_diff_pose.y = current_gnss_pose.y-current_ndt_pose.y;
    // current_diff_pose.z = current_gnss_pose.z-current_ndt_pose.z;
    // current_diff_pose.roll = current_gnss_pose.roll-current_ndt_pose.roll;
    // current_diff_pose.pitch = current_gnss_pose.pitch-current_ndt_pose.pitch;
    // current_diff_pose.yaw = current_gnss_pose.yaw-current_ndt_pose.yaw;

    // diff_q.setRPY(current_diff_pose.roll, current_diff_pose.pitch, current_diff_pose.yaw);
    // tf::Vector3 v(current_diff_pose.x, current_diff_pose.y, current_diff_pose.z);
    // tf::Transform transform(diff_q, v);
    // pose_difference_msg.header.frame_id = "/map";
    // pose_difference_msg.header.stamp = ros::Time(0);
    // pose_difference_msg.pose.position.x = current_diff_pose.x;
    // pose_difference_msg.pose.position.y = current_diff_pose.y;
    // pose_difference_msg.pose.position.z = current_diff_pose.z;
    // pose_difference_msg.pose.orientation.x = diff_q.x();
    // pose_difference_msg.pose.orientation.y = diff_q.y();
    // pose_difference_msg.pose.orientation.z = diff_q.z();
    // pose_difference_msg.pose.orientation.w = diff_q.w();

    //pose_difference_pub.publish(pose_difference_msg);

    // Publishers
    pose_difference_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_output", 10);

    //loop_rate.sleep();
    ros::spin();

    return 0;
}