#ifndef TF2_DRIVE_ROS_MSGS_H
#define TF2_DRIVE_ROS_MSGS_H

#include <tf2/convert.h>
#include <drive_ros_msgs/Obstacle.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace tf2
{

/***************/
/** OBSTACLE  **/
/***************/

/**
* method to extract timestamp from object
*/
template <>
inline
const ros::Time& getTimestamp(const drive_ros_msgs::Obstacle& o) {return o.header.stamp;}


/**
* method to extract frame id from object
*/
template <>
inline
const std::string& getFrameId(const drive_ros_msgs::Obstacle& o) {return o.header.frame_id;}

/**
* Transforms drive_ros_msgs::Obstacle data from one frame to another
*/
template <>
inline
void doTransform(const drive_ros_msgs::Obstacle &obs_in, drive_ros_msgs::Obstacle &obs_out, const geometry_msgs::TransformStamped& t_in)
{
    obs_out.header = t_in.header;

    // setup transform
    Eigen::Translation3f t(t_in.transform.translation.x,
                           t_in.transform.translation.y,
                           t_in.transform.translation.z);

    Eigen::Quaternion<float>  r(t_in.transform.rotation.w,
                                t_in.transform.rotation.x,
                                t_in.transform.rotation.y,
                                t_in.transform.rotation.z);
    Eigen::Affine3f trans(t);
    Eigen::Affine3f rot(r);

    // transform centroid
    {
        Eigen::Vector3f c = trans * rot * Eigen::Vector3f(obs_in.centroid_pose.pose.position.x,
                                                          obs_in.centroid_pose.pose.position.y,
                                                          obs_in.centroid_pose.pose.position.z);
        obs_out.centroid_pose.pose.position.x = c.x();
        obs_out.centroid_pose.pose.position.y = c.y();
        obs_out.centroid_pose.pose.position.z = c.z();
    }

    // transform polygon points
    for(auto pt: obs_in.polygon.points)
    {
        Eigen::Vector3f p = trans * rot * Eigen::Vector3f(pt.x, pt.y, pt.z);
        geometry_msgs::Point32 p_out;
        p_out.x = p.x();
        p_out.y = p.y();
        p_out.z = p.z();
        obs_out.polygon.points.push_back(p_out);
    }

    // transform orientation
    {
        Eigen::Quaternion<float> orientation = r * Eigen::Quaternion<float>(obs_in.centroid_pose.pose.orientation.w,
                                                                            obs_in.centroid_pose.pose.orientation.x,
                                                                            obs_in.centroid_pose.pose.orientation.y,
                                                                            obs_in.centroid_pose.pose.orientation.z) * r.inverse();
        obs_out.centroid_pose.pose.orientation.w = orientation.w();
        obs_out.centroid_pose.pose.orientation.x = orientation.x();
        obs_out.centroid_pose.pose.orientation.y = orientation.y();
        obs_out.centroid_pose.pose.orientation.z = orientation.z();
    }

    // save all other values
    obs_out.trust = obs_in.trust;
    obs_out.obstacle_type = obs_in.obstacle_type;

    // dimensions are independent of transformation
    // (obstacle has the same dimensions in all coordinate frames)
    obs_out.height = obs_in.height;
    obs_out.width  = obs_in.width;
    obs_out.length = obs_in.length;


    // TODO TRANSFORM COVARIANCES AND TWIST!
}


inline
drive_ros_msgs::Obstacle toMsg(const drive_ros_msgs::Obstacle &in)
{
  return in;
}

inline
void fromMsg(const drive_ros_msgs::Obstacle &msg, drive_ros_msgs::Obstacle &out)
{
  out = msg;
}


}



#endif // TF2_DRIVE_ROS_MSGS_H
