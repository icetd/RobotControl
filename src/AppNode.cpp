#include "AppNode.h"
#include <QSettings>


AppNode::AppNode(int argc, char **argv):_argc(argc), _argv(argv)
{
    QSettings* configs = new QSettings("/opt/RobotControl/config/config.ini", QSettings::IniFormat);   
    
    cmd_topic = configs->value("topic/topic_speed").toString();
    odom_topic = configs->value("topic/topic_odom").toString();
    pose_topic = configs->value("topic/topic_pose").toString();
    goal_topic = configs->value("topic/topic_goal").toString();
}
AppNode::~AppNode()
{   
    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    wait();
}

bool AppNode::init()
{
    ros::init(_argc, _argv, "AppNode", ros::init_options::AnonymousName);
    if (!ros::master::check()) {
        return false;
    }
    
    ros::start();
    TopicSubPub();
    start();
    return true;
}

bool AppNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, "AppNode", ros::init_options::AnonymousName);
    if (!ros::master::check()) 
        return false;
    
    ros::start();
    TopicSubPub();
    start();
 
    return true;
}

void AppNode::run()
{
    ros::Rate loop_rate(m_frameRate);
    
    /*use 4 threads*/
    ros::AsyncSpinner spinner(m_threadNum); 
    spinner.start();

    while(ros::ok()) {

        loop_rate.sleep();
    }
    
    emit rosShutdown();
}

void AppNode::TopicSubPub()
{
    ros::NodeHandle nh;
    m_pub_cmd = nh.advertise<geometry_msgs::Twist>(cmd_topic.toStdString(), 10);
    m_pub_goal = nh.advertise<geometry_msgs::PoseStamped>(goal_topic.toStdString(), 1000);
    m_sub_cmd = nh.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(), 200, &AppNode::speedCallback, this);
    m_sub_pose = nh.subscribe(pose_topic.toStdString(), 1000, &AppNode::poseCallback, this);
}

void AppNode::move(char key, float speed_linear, float speed_angular)
{
    std::map<char, std::vector<float>> moveBases {
        {'u', { 1, 0, 0, 1}}, {'i', { 1, 0, 0, 0}}, {'o', { 1, 0, 0,-1}},
        {'j', { 0, 0, 0, 1}}, {'k', { 0, 0, 0, 0}}, {'l', { 0, 0, 0,-1}},
        {'m', {-1, 0, 0,-1}}, {',', {-1, 0, 0, 0}}, {'.', {-1, 0, 0, 1}},
    };

    float direction_x = moveBases[key][0];
    float direction_y = moveBases[key][1];
    float direction_z = moveBases[key][2];
    float direction_yaw = moveBases[key][3];

    geometry_msgs::Twist twist;

    twist.linear.x = direction_x * speed_linear;
    twist.linear.y = direction_y * speed_linear;
    twist.linear.z = direction_z * speed_linear;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = direction_yaw * speed_angular;

    m_pub_cmd.publish(twist);
    ros::spinOnce();
}

void AppNode::speedCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.linear.y);
}

void AppNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
    emit signal_position(pos.header.frame_id.data(), pos.pose.pose.position.x, pos.pose.pose.position.y, pos.pose.pose.position.z, pos.pose.pose.orientation.w);
}

void AppNode::set_goal(QString frame, double x, double y, double z, double w)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = frame.toStdString();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.w = w;

    m_pub_goal.publish(goal);
    ros::spinOnce();
}