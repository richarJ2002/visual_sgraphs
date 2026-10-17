/*!
 * @file            fitRotationFromNormals.cc
 *
 * @brief           Implements fitRotationFromNormals(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "semantic/semantic_verify/semantic_verify.h"

#include "geometric/Plane/Plane.h"
#include "loop_closing/loop_closing.h"
#include "map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "types/types.h"
#include "semantic/room/objects/RoomClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <set>

#include "semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Horn-style SVD rotation fit on signed normal correspondences
 *                  (same closed-form pattern as
 *                  Utils::computeMapTransform_Horn's covariance/SVD step,
 *                  applied to plane normals instead of point positions --
 *                  Horn's function itself is not called; it is point-based and
 *                  unsuitable here).
 */
SemanticVerifyStatus
    fitRotationFromNormals(const std::vector<Eigen::Vector3d> &normalsA_in,
                           const std::vector<Eigen::Vector3d> &normalsB_in,
                           RotationFit                        &rotation_out)
{
    RotationFit result;
    if (normalsA_in.size() != normalsB_in.size() || normalsA_in.size() < 3U)
    {
        rotation_out = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t index = 0U; index < normalsA_in.size(); ++index)
    {
        bool isFiniteVector2{};
        if (isFiniteVector(normalsA_in[index], isFiniteVector2) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isFiniteVector returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool isFiniteVector3{};
        if (!(!isFiniteVector2) &&
            isFiniteVector(normalsB_in[index], isFiniteVector3) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isFiniteVector returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!isFiniteVector2 || !isFiniteVector3)
        {
            rotation_out = result;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        covariance += normalsA_in[index] * normalsB_in[index].transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                                Eigen::ComputeFullU |
                                                    Eigen::ComputeFullV);

    Eigen::Matrix3d signCorrection = Eigen::Matrix3d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
    {
        signCorrection(2, 2) = -1.0;
    }

    result.rotation =
        svd.matrixV() * signCorrection * svd.matrixU().transpose();
    result.valid = result.rotation.allFinite();
    rotation_out = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
