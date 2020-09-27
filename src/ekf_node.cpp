#include "ekf_node.hpp"

#include "ekf_models.hpp"

// constructor
EKFNode::EKFNode(ros::NodeHandle& nh, double r) : nh(nh), z(14), u(8), rate(r) {
    // advertise our estimation
    u = 0.0;
    z = 0.0;
    u(8) = rate;
    this->jnt_state_pub_ = nh.advertise<sensor_msgs::JointState>("posterior", 10);
    this->jnt_state_sub_ =
        nh.subscribe("kinova_joint_states", 1000, &EKFNode::jnt_state_callback, this);
}

// torque callback
void EKFNode::sensor_callback() { /* place holder for torque callback */
}

// This calls the EKF with the latest measurements
void EKFNode::jnt_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    z(1) = msg->position[0];
    z(2) = msg->position[1];
    z(3) = msg->position[2];
    z(4) = msg->position[3];
    z(5) = msg->position[4];
    z(6) = msg->position[5];
    z(7) = msg->position[6];

    z(8) = msg->velocity[0];
    z(9) = msg->velocity[1];
    z(10) = msg->velocity[2];
    z(11) = msg->velocity[3];
    z(12) = msg->velocity[4];
    z(13) = msg->velocity[5];
    z(14) = msg->velocity[6];

    if (!ekf.is_initialized()) {
        ekf.initialize();
    }
}

void EKFNode::update() {
    if (ekf.is_initialized()) {
        ekf.update(ros::Time::now(), z, u);
        State posterior = ekf.get_posterior();
        sensor_msgs::JointState msg;
        msg.position.resize(7);
        msg.velocity.resize(7);
        msg.effort.resize(7);

        // TO be filled
        for (int i = 0; i < 7; ++i) {
            msg.header.stamp = ros::Time::now();
            msg.position[i] = posterior.q[i];
            msg.velocity[i] = posterior.qd[i];
            msg.effort[i] = posterior.qdd[i];
        }

        this->jnt_state_pub_.publish(msg);
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "EKF");
    ros::NodeHandle nh;

    // create filter class
    EKFNode ekf_node(nh, 1.0 / 1000.0);
    ros::Rate r(1000.0);

    while (nh.ok()) {
        ekf_node.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
