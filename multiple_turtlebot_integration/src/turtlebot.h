
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <string>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>


class DefaultTurtleBot {
public:
public:
    DefaultTurtleBot(const std::string& namespace_prefix, const ros::NodeHandle& nh,  geometry_msgs::Transform map_Offset);
    // DefaultTurtleBot cloneWithNamespace(const std::string& name) const;
    void Send_cmd_tb1(geometry_msgs::Twist intructions);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg);
    void RGBCallback(const sensor_msgs::Image::ConstPtr& Msg);
    void ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg);
    void guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

    DefaultTurtleBot(const DefaultTurtleBot&) = delete;
    DefaultTurtleBot& operator=(const DefaultTurtleBot&) = delete;

    nav_msgs::Odometry GetCurrent_Odom();
    sensor_msgs::Image GetCurrentupdated_RGB();
    sensor_msgs::LaserScan Getupdated_Lida();
    sensor_msgs::Image Getupdated_imageDepth();

    /**
     * @brief Function to incorporate transform into odom reading
     * @param[in] odom Odometry of TurtleBot
     * @return Converted odometry
    */
    nav_msgs::Odometry odomToMap(nav_msgs::Odometry odom);

    /** Transform of map to odom*/
    geometry_msgs::Transform map;
    

    // void callback(const SomeMessageTypeConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber some_subscriber_;
    // ros::NodeHandle nh_;
    std::string namespace_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Subscriber sub4_;

    std::mutex odom_locker;
    std::mutex odom_locker2;
    std::mutex RGB_locker;
    std::mutex Lida_locker;
    std::mutex ImageDepth_locker;

    //variables for callbacks
    nav_msgs::Odometry Current_Odom;
    nav_msgs::Odometry guider_Odom;
    sensor_msgs::Image updated_RGB;
    sensor_msgs::LaserScan updated_Lida;
    sensor_msgs::Image updated_imageDepth;


};