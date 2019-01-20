#ifndef TF2_POSEWITHCOVARIANCE_H
#define TF2_POSEWITHCOVARIANCE_H

#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//////////////////////////////////////////////////////////////
// Remove this header when following PR is released:        //
// https://github.com/ros/geometry2/pull/300                //
//////////////////////////////////////////////////////////////

namespace tf2
{

/***************************/
/** Pose with covariance  **/
/***************************/

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

} // namespace


#endif // TF2_POSEWITHCOVARIANCE_H
