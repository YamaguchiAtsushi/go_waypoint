#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include <vector>
#include <cmath>

#define PI 3.141592653589793
#define CIRCLE_POINTS 100  // 円を構成する点の数

class GoWaypoint {
private:
    ros::NodeHandle nh_;  // NodeHandleをプライベートメンバとして追加
    ros::Publisher marker_pub_;
    ros::Subscriber odom_sub_;
    ros::Publisher twist_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber amcl_sub_;
    ros::Publisher goal_reached_pub_;
    ros::Subscriber goal_reached_sub_; 
    ros::Subscriber waypoint_sub_; 
    ros::Subscriber speed_sub_;


    double robot_x_, robot_y_;
    double robot_odom_x_, robot_odom_y_;
    geometry_msgs::Quaternion robot_r_;
    geometry_msgs::Twist twist_;
    ros::Rate loop_rate_;
    ros::Time start_time_;
    std::vector<geometry_msgs::PoseStamped> waypoints_; // ウェイポイントのベクター
    geometry_msgs::PoseStamped goal_; // 現在の目標位置
    std_msgs::Bool goal_reached_msg;

    bool goal_reached_; 
    // bool goal_reached_callback_;
    int near_position_flag = 0;
    double robot_speed_;

    
    int i = 0; // ウェイポイントインデックス

public:
    GoWaypoint() : nh_(), loop_rate_(100) {
        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 1000, &GoWaypoint::odomCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
        scan_sub_ = nh_.subscribe("/scan", 10, &GoWaypoint::scanCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &GoWaypoint::amclPoseCallback, this);
        // marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("goal_reached", 10);
        goal_reached_sub_ = nh_.subscribe("goal_reached", 1000, &GoWaypoint::goalReachedCallback, this);
        waypoint_sub_ = nh_.subscribe("waypoint", 1000, &GoWaypoint::waypointCallback, this);
        speed_sub_ = nh_.subscribe("speed", 1000, &GoWaypoint::speedCallback, this);

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        robot_speed_ = 1.0;

        start_time_ = ros::Time::now();

        // ウェイポイントの初期化
        // ここにウェイポイントを追加するコードを実装する必要があります
    }

    void spin() {
        while (ros::ok()) {
            ros::Time now = ros::Time::now();   
            if (now - start_time_ < ros::Duration(5.0)) {
                continue;
            }

            ros::spinOnce();

            // ウェイポイントをRvizに表示
            publishWaypointsMarker();

            // ROS_INFO("GOAL POSI, (%lf, %lf)", goal_.pose.position.x, goal_.pose.position.y);
            // 目標位置に移動
            go_position(goal_);

            // 目標位置に近づいているか確認
            if (near_position(goal_)) {
                if(goal_reached_ == true){
                    continue;
                }
                twist_.linear.x = 0.1;
                twist_.angular.z = 0.0;
                // goal_reached をパブリッシュ
                // std_msgs::Bool goal_reached_;
                // std_msgs::Bool goal_reached_msg;
                // goal_reached_msg.data = true;
                // goal_reached_pub_.publish(goal_reached_msg);
                
                // std::cout << "goal_reached" << std::endl;
                // std::cout << "goal_reached" << goal_reached_msg.data << std::endl;


                // if(near_position_flag == 0){
                //     publishGoalFlag();
                // }
                // if(goal_reached_sub_ == false)
                // near_position_flag = 1;


                // 次のウェイポイントに進む
                // if (i < waypoints_.size() - 1) {
                //     goal_ = waypoints_[++i];
                // }
            }
            else{
                goal_reached_msg.data = goal_reached_;
                // goal_reached_msg.data = false;
                near_position_flag = 0;

            }

            visualize_waypoint();

            twist_pub_.publish(twist_);
            loop_rate_.sleep();
        }
    }

private:
    void visualize_waypoint(){
        visualization_msgs::Marker circle_marker;
        circle_marker.header.frame_id = "map";  // 基準座標系
        circle_marker.header.stamp = ros::Time::now();
        circle_marker.ns = "circle";
        circle_marker.id = 0;
        circle_marker.type = visualization_msgs::Marker::LINE_STRIP;
        circle_marker.action = visualization_msgs::Marker::ADD;

        // 色設定 (青, 透明度 80%)
        circle_marker.color.r = 0.0;
        circle_marker.color.g = 0.0;
        circle_marker.color.b = 1.0;
        circle_marker.color.a = 0.8;

        // 線の太さ
        circle_marker.scale.x = 0.05;

        // 円の中心位置
        double center_x = goal_.pose.position.x;
        double center_y = goal_.pose.position.y;
        double radius = 0.2;  // 半径1m

        // 円を構成する点を追加
        for (int i = 0; i <= CIRCLE_POINTS; i++) {
            double angle = 2.0 * PI * i / CIRCLE_POINTS;
            geometry_msgs::Point p;
            p.x = center_x + radius * cos(angle);
            p.y = center_y + radius * sin(angle);
            p.z = 0.0;  // 平面上に配置
            circle_marker.points.push_back(p);
        }

        marker_pub_.publish(circle_marker);
    }

    void speedCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        robot_speed_ = msg->data;
        ROS_INFO("Received speed: %.2f", robot_speed_);
    }

    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        goal_reached_ = msg->data; // サブスクライブした値でgoal_reached_を更新
        // ROS_INFO("goal_reached_ = %s", goal_reached_ ? "true" : "false");
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = msg->pose.pose.orientation;
        // ROS_INFO("Current estimated pose: [robot_x: %f, robot_y: %f, theta: %f]", robot_x_, robot_y_, tf::getYaw(msg->pose.pose.orientation));
    }

    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("Received waypoint: [x: %f, y: %f]", msg->pose.position.x, msg->pose.position.y);

        // サブスクライブしたウェイポイントを処理
        // 例えば次の目標に設定する
        goal_ = *msg;
        std::cout << "goal_" << goal_ << std::endl;
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        robot_odom_x_ = msg->pose.pose.position.x;
        robot_odom_y_ = msg->pose.pose.position.y;
        // std::cout << "robot_odom_x_:" << robot_odom_x_ << "robot_odom_y_;" << robot_odom_y_ << std::endl;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_) {
        // スキャンデータのコールバック処理
    }

    void publishGoalFlag(){
        goal_reached_msg.data = true;
        goal_reached_pub_.publish(goal_reached_msg);
    }

    void go_position(const geometry_msgs::PoseStamped& goal) {
        double k_v = 3.0;
        double k_w = 1.6;

        double v = 1.0;
        double w = 0.0;

        // double theta = atan2(goal.pose.position.y - robot_odom_y_, goal.pose.position.x - robot_odom_x_);
        double theta = atan2(goal.pose.position.y - robot_y_, goal.pose.position.x - robot_x_);

        while (theta <= -M_PI || M_PI <= theta) {
            if (theta <= -M_PI)
                theta += 2 * M_PI;
            else
                theta -= 2 * M_PI;
        }

        double roll, pitch, yaw;
        geometryQuatToRPY(robot_r_, roll, pitch, yaw);

        while (yaw <= -M_PI || M_PI <= yaw) {
            if (yaw <= -M_PI)
                yaw += 2 * M_PI;
            else
                yaw -= 2 * M_PI;
        }

        theta -= yaw;

        while (theta <= -M_PI || M_PI <= theta) {
            if (theta <= -M_PI)
                theta += 2 * M_PI;
            else
                theta -= 2 * M_PI;
        }

        w = k_w * theta;

        if (theta <= M_PI / 2 && theta >= -M_PI / 2) {
            v = 1.0;
            // v = k_v * ((goal_.pose.position.x - robot_x_) * (goal_.pose.position.x - robot_x_) + (goal_.pose.position.y - robot_y_) * (goal_.pose.position.y - robot_y_));

        } else {
            v = 1.0;
            // v = -k_v * ((goal_.pose.position.x - robot_x_) * (goal_.pose.position.x - robot_x_) + (goal_.pose.position.y - robot_y_) * (goal_.pose.position.y - robot_y_));

        }

        // twist_.linear.x = v;
        twist_.linear.x = v * robot_speed_;//vをPIDするとvの値が1を超えるため,robot_speed_をかけても速度はあまり変化しない
        twist_.angular.z = w * robot_speed_;
    }

    bool near_position(const geometry_msgs::PoseStamped& goal_) {
        double difx = robot_x_ - goal_.pose.position.x;
        double dify = robot_y_ - goal_.pose.position.y;
        // double difx = robot_odom_x_ - goal_.pose.position.x;
        // double dify = robot_odom_y_ - goal_.pose.position.y;
        return (sqrt(difx * difx + dify * dify) < 0.2);
    }

    void publishWaypointsMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto& waypoint : waypoints_) {
            geometry_msgs::Point p;
            p.x = waypoint.pose.position.x;
            p.y = waypoint.pose.position.y;
            p.z = waypoint.pose.position.z;
            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    void geometryQuatToRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw) {
        tf::Quaternion tf_quat;
        quaternionMsgToTF(quat, tf_quat);
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "go_waypoint");

    GoWaypoint goWaypoint; // NodeHandleを内部で初期化するため引数なし
    goWaypoint.spin();

    return 0;
}
