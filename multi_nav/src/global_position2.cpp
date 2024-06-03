#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

class GetGlobalPosition2
{
public:
    GetGlobalPosition2()
    {
        ros::NodeHandle nh;
        nh.getParam("/global_position2/tb0", tb3_0);
        nh.getParam("/global_position2/tb1", tb3_1);

        pose_marker_pub_1 = nh.advertise<visualization_msgs::Marker>("/" + tb3_0 + "/marker/position", 10);
        pose_marker_pub_2 = nh.advertise<visualization_msgs::Marker>("/" + tb3_1 + "/marker/position", 10);

        position_marker_1();
        position_marker_2();
    }

    void getPosition()
    {
        tf::TransformListener tf_listener;
        ros::Rate rate(10);

        while (ros::ok())
        {
            tf::StampedTransform transform;
            bool success;

            success = getTransform(tf_listener, tb3_0 + "/base_footprint", transform);
            if (success)
            {
                geometry_msgs::Point point;
                point.x = transform.getOrigin().x();
                point.y = transform.getOrigin().y();
                point.z = 0.0;
                pose_points_1.points.push_back(point);
                pose_marker_pub_1.publish(pose_points_1);
            }

            success = getTransform(tf_listener, tb3_1 + "/base_footprint", transform);
            if (success)
            {
                geometry_msgs::Point point;
                point.x = transform.getOrigin().x();
                point.y = transform.getOrigin().y();
                point.z = 0.0;
                pose_points_2.points.push_back(point);
                pose_marker_pub_2.publish(pose_points_2);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    std::string tb3_0, tb3_1;
    ros::Publisher pose_marker_pub_1, pose_marker_pub_2;
    visualization_msgs::Marker pose_points_1, pose_points_2;

    void position_marker_1()
    {
        pose_points_1.header.frame_id = "map";
        pose_points_1.header.stamp = ros::Time::now();
        pose_points_1.ns = "points";
        pose_points_1.id = 0;
        pose_points_1.type = visualization_msgs::Marker::POINTS;
        pose_points_1.action = visualization_msgs::Marker::ADD;
        pose_points_1.pose.orientation.w = 1.0;
        pose_points_1.scale.x = 0.3;
        pose_points_1.scale.y = 0.3;
        pose_points_1.color = getColor(1.0, 0.0, 0.0, 1.0);
        pose_points_1.lifetime = ros::Duration();
    }

    void position_marker_2()
    {
        pose_points_2.header.frame_id = "map";
        pose_points_2.header.stamp = ros::Time::now();
        pose_points_2.ns = "points";
        pose_points_2.id = 0;
        pose_points_2.type = visualization_msgs::Marker::POINTS;
        pose_points_2.action = visualization_msgs::Marker::ADD;
        pose_points_2.pose.orientation.w = 1.0;
        pose_points_2.scale.x = 0.3;
        pose_points_2.scale.y = 0.3;
        pose_points_2.color = getColor(0.0, 1.0, 0.0, 1.0);
        pose_points_2.lifetime = ros::Duration();
    }

    std_msgs::ColorRGBA getColor(float r, float g, float b, float a)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    bool getTransform(tf::TransformListener &listener, const std::string &target_frame, tf::StampedTransform &transform)
    {
        try
        {
            listener.waitForTransform("/map", target_frame, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/map", target_frame, ros::Time(0), transform);
            return true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_position2");

    GetGlobalPosition2 global_position;
    global_position.getPosition();

    return 0;
}