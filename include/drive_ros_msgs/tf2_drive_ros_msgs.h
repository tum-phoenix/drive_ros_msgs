#ifndef TF2_DRIVE_ROS_MSGS_H
#define TF2_DRIVE_ROS_MSGS_H


#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <drive_ros_msgs/Obstacle.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>


////////////////////////
/// IMPORTANT:
/// Some transforms can may be available in standard ROS packages in newer versions!

namespace tf2
{


/*************/
/** Vector3 **/
/*************/


/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Vector3 toMsg(const tf2::Vector3& in)
{
  geometry_msgs::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/***********/
/** Point **/
/***********/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Point& toMsg(const tf2::Vector3& in, geometry_msgs::Point& out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Point& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

/****************/
/** Quaternion **/
/****************/

/** \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Quaternion object.
 * \return The Quaternion converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Quaternion toMsg(const tf2::Quaternion& in)
{
  geometry_msgs::Quaternion out;
  out.w = in.getW();
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Quaternion message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Quaternion message type.
 * \param out The Quaternion converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}

/***************/
/** Transform **/
/***************/

/** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Transform object.
 * \return The Transform converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Transform toMsg(const tf2::Transform& in)
{
  geometry_msgs::Transform out;
  out.translation = toMsg(in.getOrigin());
  out.rotation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a Transform message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Transform message type.
 * \param out The Transform converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  tf2::Vector3 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  // w at the end in the constructor
  tf2::Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}

/************/
/** Pose   **/
/************/

/** \brief Convert a tf2 Transform type to an equivalent geometry_msgs Pose message.
 * \param in A tf2 Transform object.
 * \param out The Transform converted to a geometry_msgs Pose message type.
 */
inline
geometry_msgs::Pose& toMsg(const tf2::Transform& in, geometry_msgs::Pose& out)
{
  toMsg(in.getOrigin(), out.position);
  out.orientation = toMsg(in.getRotation());
  return out;
}

/***************************/
/** Pose with covariance  **/
/***************************/


/** \brief Transform the covariance matrix of a PoseWithCovarianceStamped message to a new frame.
* \param t_in The covariance matrix to transform.
* \param transform The timestamped transform to apply, as a TransformStamped message.
* \return The transformed covariance matrix.
*/
inline
geometry_msgs::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& cov_in, const tf2::Transform& transform)
{
  /**
   * To transform a covariance matrix:
   *
   * [R 0] COVARIANCE [R' 0 ]
   * [0 R]            [0  R']
   *
   * Where:
   * 	R is the rotation matrix (3x3).
   * 	R' is the transpose of the rotation matrix.
   * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
   */

  // get rotation matrix transpose
  const tf2::Matrix3x3  R_transpose = transform.getBasis().transpose();

  // convert the covariance matrix into four 3x3 blocks
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2],
                  cov_in[6], cov_in[7], cov_in[8],
                  cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5],
                  cov_in[9], cov_in[10], cov_in[11],
                  cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20],
                  cov_in[24], cov_in[25], cov_in[26],
                  cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23],
                  cov_in[27], cov_in[28], cov_in[29],
                  cov_in[33], cov_in[34], cov_in[35]);

  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = transform.getBasis()*cov_11*R_transpose;
  const tf2::Matrix3x3 result_12 = transform.getBasis()*cov_12*R_transpose;
  const tf2::Matrix3x3 result_21 = transform.getBasis()*cov_21*R_transpose;
  const tf2::Matrix3x3 result_22 = transform.getBasis()*cov_22*R_transpose;

  // form the output
  geometry_msgs::PoseWithCovariance::_covariance_type output;
  output[0] = result_11[0][0];
  output[1] = result_11[0][1];
  output[2] = result_11[0][2];
  output[6] = result_11[1][0];
  output[7] = result_11[1][1];
  output[8] = result_11[1][2];
  output[12] = result_11[2][0];
  output[13] = result_11[2][1];
  output[14] = result_11[2][2];

  output[3] = result_12[0][0];
  output[4] = result_12[0][1];
  output[5] = result_12[0][2];
  output[9] = result_12[1][0];
  output[10] = result_12[1][1];
  output[11] = result_12[1][2];
  output[15] = result_12[2][0];
  output[16] = result_12[2][1];
  output[17] = result_12[2][2];

  output[18] = result_21[0][0];
  output[19] = result_21[0][1];
  output[20] = result_21[0][2];
  output[24] = result_21[1][0];
  output[25] = result_21[1][1];
  output[26] = result_21[1][2];
  output[30] = result_21[2][0];
  output[31] = result_21[2][1];
  output[32] = result_21[2][2];

  output[21] = result_22[0][0];
  output[22] = result_22[0][1];
  output[23] = result_22[0][2];
  output[27] = result_22[1][0];
  output[28] = result_22[1][1];
  output[29] = result_22[1][2];
  output[33] = result_22[2][0];
  output[34] = result_22[2][1];
  output[35] = result_22[2][2];

  return output;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs PoseWithCovariance type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped PoseWithCovariance message.
* \param t_out The transformed pose, as a timestamped PoseWithCovariance message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseWithCovariance& t_in, geometry_msgs::PoseWithCovariance& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.pose.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.pose.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out.pose);

  t_out.covariance = transformCovariance(t_in.covariance, t);
}


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



#endif // TF2_DRIVE_ROS_MSGS_H
