/*!
 * @File:         SE3fToTFTransform.cpp
 *
 * @Brief:
 *
 * @Date:         20/07/2026
 *
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

tf2::Transform SE3fToTFTransform(const Sophus::SE3f &transformation_SE3f_in)
{
    /* Extract the translation component */
    const Eigen::Vector3f translation_Eigen =
        transformation_SE3f_in.translation();

    /* Extract and normalise the orientation component */
    Eigen::Quaternionf orientation_Eigen =
        transformation_SE3f_in.unit_quaternion();

    orientation_Eigen.normalize();

    /* Convert the translation into its tf2 representation */
    const tf2::Vector3 translation_tf2(
        static_cast<tf2Scalar>(translation_Eigen.x()),
        static_cast<tf2Scalar>(translation_Eigen.y()),
        static_cast<tf2Scalar>(translation_Eigen.z()));

    /* Convert the orientation into its tf2 representation */
    tf2::Quaternion orientation_tf2(
        static_cast<tf2Scalar>(orientation_Eigen.x()),
        static_cast<tf2Scalar>(orientation_Eigen.y()),
        static_cast<tf2Scalar>(orientation_Eigen.z()),
        static_cast<tf2Scalar>(orientation_Eigen.w()));

    orientation_tf2.normalize();

    /* Construct and return the complete tf2 transformation */
    return tf2::Transform(orientation_tf2, translation_tf2);
}