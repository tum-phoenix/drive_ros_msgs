#ifndef TF2_OBSTACLE_H
#define TF2_OBSTACLE_H

#include <drive_ros_msgs/tf2_PoseWithCovariance.h>
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

    // save all other values
    obs_out.trust = obs_in.trust;
    obs_out.obstacle_type = obs_in.obstacle_type;

    // dimensions are independent of transformation
    // (obstacle has the same dimensions in all coordinate frames)
    obs_out.height = obs_in.height;
    obs_out.width  = obs_in.width;
    obs_out.length = obs_in.length;

    // transform centroid pose
    doTransform(obs_in.centroid_pose, obs_out.centroid_pose, t_in);


    // TODO: twist!
    obs_out.centroid_twist = obs_in.centroid_twist;
}

} // namespace



#endif // TF2_OBSTACLE_H
