#ifndef APP_NODE_H
#define APP_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <QThread>
#include <QString>
#include <string>
#include <map>


class AppNode : public QThread 
{
    Q_OBJECT
public:
    AppNode(int argc, char **argv);
    virtual ~AppNode();
    bool init();
    bool init (const std::string &master_url, const std::string &host_url);

    void move(char _key, float _speed_linear, float _speed_angular);

    void run() override;

signals:
    void rosShutdown();
    void speed_x(double x);
    void speed_y(double y);

private:
    int _argc;
    char **_argv;

    ros::Publisher m_pub_cmd;
    ros::Subscriber m_sub_cmd;
    ros::Subscriber m_sub_pos;
    ros::Subscriber m_laser_sub;
    ros::Subscriber m_map_sub;

    QString odom_topic;
    QString pose_topic; 
    QString laser_topic;
    QString map_Topic;
    QString navGoal_topic;



    int m_frameRate = 40;
    int m_threadNum = 4;
    void TopicSubPub();

    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
};

#endif