#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>

class GetGlobalPath3
{
public:
    GetGlobalPath3()
    {
        ros::NodeHandle nh;
        nh.getParam("/global_path3/tb0", tb3_0);
        nh.getParam("/global_path3/tb1", tb3_1);
        nh.getParam("/global_path3/tb2", tb3_2);

        pub_1 = nh.advertise<visualization_msgs::Marker>("/" + tb3_0 + "/marker/gpp", 10);
        pub_2 = nh.advertise<visualization_msgs::Marker>("/" + tb3_1 + "/marker/gpp", 10);
        pub_3 = nh.advertise<visualization_msgs::Marker>("/" + tb3_2 + "/marker/gpp", 10);
    }

    void ggpCallback1(const nav_msgs::Path::ConstPtr &data)
    {
        poses_x1_.clear();
        poses_y1_.clear();

        for (const auto &pose : data->poses)
        {
            poses_x1_.push_back(pose.pose.position.x);
            poses_y1_.push_back(pose.pose.position.y);
        }

        if (poses_x1_.empty())
        {
            ROS_INFO("Waiting for plan data");
        }
    }

    void ggpCallback2(const nav_msgs::Path::ConstPtr &data)
    {
        poses_x2_.clear();
        poses_y2_.clear();

        for (const auto &pose : data->poses)
        {
            poses_x2_.push_back(pose.pose.position.x);
            poses_y2_.push_back(pose.pose.position.y);
        }

        if (poses_x2_.empty())
        {
            ROS_INFO("Waiting for plan data");
        }
    }

    void ggpCallback3(const nav_msgs::Path::ConstPtr &data)
    {
        poses_x3_.clear();
        poses_y3_.clear();

        for (const auto &pose : data->poses)
        {
            poses_x3_.push_back(pose.pose.position.x);
            poses_y3_.push_back(pose.pose.position.y);
        }

        if (poses_x3_.empty())
        {
            ROS_INFO("Waiting for plan data");
        }
    }

    void pathSub1()
    {
        ros::NodeHandle nh;
        sub_1 = nh.subscribe("/" + tb3_0 + "/move_base/NavfnROS/plan", 1, &GetGlobalPath3::ggpCallback1, this);
    }

    void pathSub2()
    {
        ros::NodeHandle nh;
        sub_2 = nh.subscribe("/" + tb3_1 + "/move_base/NavfnROS/plan", 1, &GetGlobalPath3::ggpCallback2, this);
    }

    void pathSub3()
    {
        ros::NodeHandle nh;
        sub_3 = nh.subscribe("/" + tb3_2 + "/move_base/NavfnROS/plan", 1, &GetGlobalPath3::ggpCallback3, this);
    }

    void getPath1()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            pathSub1();
            posMarker1();
            if (!poses_x1_.empty())
            {
                for (size_t i = 0; i < poses_x1_.size(); ++i)
                {
                    geometry_msgs::Point point;
                    point.x = poses_x1_[i];
                    point.y = poses_y1_[i];
                    point.z = 0;
                    pose_points1_.points.push_back(point);
                }
                pub_1.publish(pose_points1_);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void getPath2()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            pathSub2();
            posMarker2();
            if (!poses_x2_.empty())
            {
                for (size_t i = 0; i < poses_x2_.size(); ++i)
                {
                    geometry_msgs::Point point;
                    point.x = poses_x2_[i];
                    point.y = poses_y2_[i];
                    point.z = 0;
                    pose_points2_.points.push_back(point);
                }
                pub_2.publish(pose_points2_);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void getPath3()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            pathSub3();
            posMarker3();
            if (!poses_x3_.empty())
            {
                for (size_t i = 0; i < poses_x3_.size(); ++i)
                {
                    geometry_msgs::Point point;
                    point.x = poses_x3_[i];
                    point.y = poses_y3_[i];
                    point.z = 0;
                    pose_points3_.points.push_back(point);
                }
                pub_3.publish(pose_points3_);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void posMarker1()
    {
        pose_points1_.header.frame_id = "map";
        pose_points1_.header.stamp = ros::Time::now();
        pose_points1_.ns = "points";
        pose_points1_.id = 0;
        pose_points1_.type = visualization_msgs::Marker::POINTS;
        pose_points1_.action = visualization_msgs::Marker::ADD;
        pose_points1_.pose.orientation.w = 1.0;
        pose_points1_.scale.x = 0.2;
        pose_points1_.scale.y = 0.2;

        pose_points1_.color = createColor(1, 0, 1, 1);

        pose_points1_.lifetime = ros::Duration();
    }

    void posMarker2()
    {
        pose_points2_.header.frame_id = "map";
        pose_points2_.header.stamp = ros::Time::now();
        pose_points2_.ns = "points";
        pose_points2_.id = 0;
        pose_points2_.type = visualization_msgs::Marker::POINTS;
        pose_points2_.action = visualization_msgs::Marker::ADD;
        pose_points2_.pose.orientation.w = 1.0;
        pose_points2_.scale.x = 0.2;
        pose_points2_.scale.y = 0.2;

        pose_points2_.color = createColor(1, 1, 0, 1);

        pose_points2_.lifetime = ros::Duration();
    }

    void posMarker3()
    {
        pose_points3_.header.frame_id = "map";
        pose_points3_.header.stamp = ros::Time::now();
        pose_points3_.ns = "points";
        pose_points3_.id = 0;
        pose_points3_.type = visualization_msgs::Marker::POINTS;
        pose_points3_.action = visualization_msgs::Marker::ADD;
        pose_points3_.pose.orientation.w = 1.0;
        pose_points3_.scale.x = 0.2;
        pose_points3_.scale.y = 0.2;

        pose_points3_.color = createColor(0, 1, 1, 1);

        pose_points3_.lifetime = ros::Duration();
    }

private:
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    std::string tb3_0, tb3_1, tb3_2;
    ros::Publisher pub_1, pub_2, pub_3;
    ros::Subscriber sub_1, sub_2, sub_3;
    visualization_msgs::Marker pose_points1_, pose_points2_, pose_points3_;
    std::vector<double> poses_x1_, poses_x2_, poses_x3_;
    std::vector<double> poses_y1_, poses_y2_, poses_y3_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_path3");
    GetGlobalPath3 obj;
    obj.getPath1();
    obj.getPath2();
    obj.getPath3();
    return 0;
}