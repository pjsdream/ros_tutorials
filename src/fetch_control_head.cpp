#include <algorithm>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <topic_tools/MuxSelect.h>


double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fetch_control_head");

    ros::Duration dt(1. / 30);

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

    ros::NodeHandle nh("~");
    ros::NodeHandle pnh(nh, "head");

    int deadman_, axis_pan_, axis_tilt_;
    double max_vel_pan_, max_vel_tilt_;
    double max_acc_pan_, max_acc_tilt_;
    double min_pos_pan_, max_pos_pan_, min_pos_tilt_, max_pos_tilt_;
    std::string head_pan_joint_, head_tilt_joint_;
    double actual_pos_pan_ = 0, actual_pos_tilt_ = 0;  // actual positions
    double desired_pan_, desired_tilt_;  // desired velocities
    double last_pan_, last_tilt_;

    pnh.param("button_deadman", deadman_, 8);
    pnh.param("axis_pan", axis_pan_, 0);
    pnh.param("axis_tilt", axis_tilt_, 3);

    pnh.param("max_vel_pan", max_vel_pan_, 1.5);
    pnh.param("max_vel_tilt", max_vel_tilt_, 1.5);
    pnh.param("max_acc_pan", max_acc_pan_, 3.0);
    pnh.param("max_acc_tilt", max_acc_tilt_, 3.0);
    pnh.param("min_pos_pan", min_pos_pan_, -1.57);
    pnh.param("max_pos_pan", max_pos_pan_, 1.57);
    pnh.param("min_pos_tilt", min_pos_tilt_, -0.76);
    pnh.param("max_pos_tilt", max_pos_tilt_, 1.45);

    head_pan_joint_ = "head_pan_joint";
    head_tilt_joint_ = "head_tilt_joint";

    std::string action_name = "/head_controller/follow_joint_trajectory";
    boost::shared_ptr<client_t> client_;
    client_.reset(new client_t(action_name, true));
    if (!client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("%s may not be connected.", action_name.c_str());
    }

    // set to desired angles
    desired_pan_ = 0;
    desired_tilt_ = 0.7;

    ros::Rate rate(30);

    while (true)
    {
        // Fill in message (future dated with fixed time step)
        double step = 0.125;
        double pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, step);
        double pan_travel = step * (pan_vel + last_pan_) / 2.0;
        double pan = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
        double tilt_vel = integrate(desired_tilt_, last_tilt_, max_acc_tilt_, step);
        double tilt_travel = step * (tilt_vel + last_tilt_) / 2.0;
        double tilt = std::max(min_pos_tilt_, std::min(max_pos_tilt_, actual_pos_tilt_ + tilt_travel));
        // Publish message
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back(head_pan_joint_);
        goal.trajectory.joint_names.push_back(head_tilt_joint_);
        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(pan);
        p.positions.push_back(tilt);
        p.velocities.push_back(pan_vel);
        p.velocities.push_back(tilt_vel);
        p.time_from_start = ros::Duration(step);
        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = ros::Duration(0.0);
        client_->sendGoal(goal);
        // Update based on actual timestep
        pan_vel = integrate(desired_pan_, last_pan_, max_acc_pan_, dt.toSec());
        pan_travel = dt.toSec() * (pan_vel + last_pan_) / 2.0;
        actual_pos_pan_ = std::max(min_pos_pan_, std::min(max_pos_pan_, actual_pos_pan_ + pan_travel));
        last_pan_ = pan_vel;
        tilt_vel = integrate(desired_tilt_, last_tilt_, max_acc_tilt_, dt.toSec());
        tilt_travel = dt.toSec() * (tilt_vel + last_tilt_) / 2.0;
        actual_pos_tilt_ = std::max(min_pos_tilt_, std::min(max_pos_tilt_, actual_pos_tilt_ + tilt_travel));
        last_tilt_ = tilt_vel;

        rate.sleep();
    }

    return 0;
}
