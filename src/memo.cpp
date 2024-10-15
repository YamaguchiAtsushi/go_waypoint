#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>  // Boolメッセージ型を使用
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

class WaypointFollower {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber waypoint_sub_;  // ウェイポイントをサブスクライブするためのSubscriber
    ros::Publisher twist_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher goal_reached_pub_;

    geometry_msgs::Twist twist_;
    geometry_msgs::PoseStamped current_goal_;  // 現在のゴール
    bool goal_reached_;
    double robot_x_, robot_y_, yaw_, odom_robot_x_, odom_robot_y_;

public:
    WaypointFollower() : goal_reached_(false) {
        odom_sub_ = nh_.subscribe("/ypspur_ros/odom", 1000, &WaypointFollower::odomCallback, this);
        waypoint_sub_ = nh_.subscribe("next_waypoint", 1, &WaypointFollower::waypointCallback, this);  // ウェイポイントのサブスクライブ
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1000);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("goal_reached", 10);
    }

    // オドメトリのコールバック関数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_robot_x_ = msg->pose.pose.position.x;
        odom_robot_y_ = msg->pose.pose.position.y;
    }

    // ウェイポイントのコールバック関数
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_goal_ = *msg;
        goal_reached_ = false;  // 新しいウェイポイントが到着したらgoal_reachedをリセット
        ROS_INFO("新しいウェイポイントを受信: x=%f, y=%f, z=%f", current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z);
    }

    // ロボットがウェイポイントに到達したかどうかをチェックする関数
    bool isGoalReached() {
        double distance = std::sqrt(std::pow(current_goal_.pose.position.x - odom_robot_x_, 2) +
                                    std::pow(current_goal_.pose.position.y - odom_robot_y_, 2));

        if (distance < 0.1) {  // 例えば、0.1m以内ならゴール到達とみなす
            goal_reached_ = true;
        }
        return goal_reached_;
    }

    // ウェイポイントに向かってロボットを移動させる関数
    void moveToGoal() {
        if (!goal_reached_) {
            double dx = current_goal_.pose.position.x - odom_robot_x_;
            double dy = current_goal_.pose.position.y - odom_robot_y_;

            // シンプルな比例制御で目標地点に向かう
            twist_.linear.x = 0.5 * std::sqrt(dx * dx + dy * dy);
            twist_.angular.z = 2.0 * std::atan2(dy, dx);

            twist_pub_.publish(twist_);
        } else {
            twist_.linear.x = 0.0;
            twist_.angular.z = 0.0;
            twist_pub_.publish(twtwistist_);

            std_msgs::Bool goal_reached_msg;
            goal_reached_msg.data = true;
            goal_reached_pub_.publish(goal_reached_msg);  // ゴール到達を通知
        }
    }

    // マーカーのパブリッシュ関数
    void publishGoalMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "current_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = current_goal_.pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub_.publish(marker);
    }

    void run() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            if (!goal_reached_) {
                moveToGoal();
                publishGoalMarker();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_follower");

    WaypointFollower wf;
    wf.run();

    return 0;
}
