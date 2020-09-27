
#ifndef __EKF_NODE__
#define __EKF_NODE__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ekf.hpp"

// messages
#include <sensor_msgs/JointState.h>

class EKFNode {
 private:
    BFL::ColumnVector z;
    BFL::ColumnVector u;

    double rate = 1e-3;

    ros::NodeHandle nh;

    ros::Subscriber jnt_state_sub_;
    ros::Publisher jnt_state_pub_;

    EKF ekf;

 public:
    EKFNode(ros::NodeHandle& nh, double rate);

    void sensor_callback();
    void jnt_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void update();
};

#endif
