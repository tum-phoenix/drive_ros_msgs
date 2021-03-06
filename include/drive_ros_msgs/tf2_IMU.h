#ifndef TF2_IMU_H
#define TF2_IMU_H

#include <tf2/convert.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>


//////////////////////////////////////////////////////////////
// Remove this header when following PR is released:        //
// https://github.com/ros/geometry_experimental/pull/78     //
//////////////////////////////////////////////////////////////


namespace tf2
{

/**********/
/** IMU  **/
/**********/

/**
* method to extract timestamp from object
*/
  template <>
  inline
  const ros::Time& getTimestamp(const sensor_msgs::Imu& p) {return p.header.stamp;}

/**
* method to extract frame id from object
*/
  template <>
  inline
  const std::string& getFrameId(const sensor_msgs::Imu &p) {return p.header.frame_id;}


/**
* Transforms a covariance array from one frame to another
*/
  inline
  void transformCovariance(const boost::array<double, 9>& in, boost::array<double, 9>& out, Eigen::Quaternion<double> r){

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());
    cov_out = r * cov_in * r.inverse();

  }

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
  template <>
  inline
  void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped& t_in)
  {

    imu_out.header = t_in.header;


    Eigen::Vector3d trans(t_in.transform.translation.x,
                          t_in.transform.translation.y,
                          t_in.transform.translation.z);

    Eigen::Quaternion<double> r(t_in.transform.rotation.w,
                                t_in.transform.rotation.x,
                                t_in.transform.rotation.y,
                                t_in.transform.rotation.z);

    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d vel = t * Eigen::Vector3d(imu_in.angular_velocity.x,
                                              imu_in.angular_velocity.y,
                                              imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = vel.x();
    imu_out.angular_velocity.y = vel.y();
    imu_out.angular_velocity.z = vel.z();

    transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);


    /* assumptions being made:
     * - translation is fixed over time (rigid body)
     * - Euler acceleration is neglected (otherwise we would have to break
     *   the function structure in order to derive the angular velocity)
     */


    Eigen::Vector3d accel = t * Eigen::Vector3d(imu_in.linear_acceleration.x,
                                                imu_in.linear_acceleration.y,
                                                imu_in.linear_acceleration.z)
                            + vel.cross(vel.cross(trans));


    imu_out.linear_acceleration.x = accel.x();
    imu_out.linear_acceleration.y = accel.y();
    imu_out.linear_acceleration.z = accel.z();

    transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

    Eigen::Quaternion<double> orientation = r * Eigen::Quaternion<double>(imu_in.orientation.w,
                                                                          imu_in.orientation.x,
                                                                          imu_in.orientation.y,
                                                                          imu_in.orientation.z) * r.inverse();

    imu_out.orientation.w = orientation.w();
    imu_out.orientation.x = orientation.x();
    imu_out.orientation.y = orientation.y();
    imu_out.orientation.z = orientation.z();

    transformCovariance(imu_in.orientation_covariance, imu_out.orientation_covariance, r);

  }

  inline
  sensor_msgs::Imu toMsg(const sensor_msgs::Imu &in)
  {
    return in;
  }

  inline
  void fromMsg(const sensor_msgs::Imu &msg, sensor_msgs::Imu &out)
  {
    out = msg;
  }

/*********************/
/** Magnetic Field  **/
/*********************/

/**
* method to extract timestamp from object
*/
  template <>
  inline
  const ros::Time& getTimestamp(const sensor_msgs::MagneticField& p) {return p.header.stamp;}

/**
* method to extract frame id from object
*/
  template <>
  inline
  const std::string& getFrameId(const sensor_msgs::MagneticField &p) {return p.header.frame_id;}

/**
* Transforms sensor_msgs::MagneticField data from one frame to another
*/
  template <>
  inline
  void doTransform(const sensor_msgs::MagneticField &mag_in, sensor_msgs::MagneticField &mag_out, const geometry_msgs::TransformStamped& t_in)
  {

    mag_out.header = t_in.header;

    // Discard translation, only use orientation for Magnetic Field transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d mag = t * Eigen::Vector3d(
        mag_in.magnetic_field.x, mag_in.magnetic_field.y, mag_in.magnetic_field.z);

    mag_out.magnetic_field.x = mag.x();
    mag_out.magnetic_field.y = mag.y();
    mag_out.magnetic_field.z = mag.z();

    transformCovariance(mag_in.magnetic_field_covariance, mag_out.magnetic_field_covariance, r);

  }

  inline
  sensor_msgs::MagneticField toMsg(const sensor_msgs::MagneticField &in)
  {
    return in;
  }

  inline
  void fromMsg(const sensor_msgs::MagneticField &msg, sensor_msgs::MagneticField &out)
  {
    out = msg;
  }

} // namespace


#endif // TF2_IMU_H
