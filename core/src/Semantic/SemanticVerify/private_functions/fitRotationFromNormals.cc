

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! Horn-style SVD rotation fit on signed normal correspondences (same
 * closed-form pattern as Utils::computeMapTransform_Horn's covariance/SVD
 * step, applied to plane normals instead of point positions -- Horn's
 * function itself is not called; it is point-based and unsuitable here). */
RotationFit
    fitRotationFromNormals(const std::vector<Eigen::Vector3d> &normalsA_in,
                           const std::vector<Eigen::Vector3d> &normalsB_in)
{
    RotationFit result;
    if (normalsA_in.size() != normalsB_in.size() || normalsA_in.size() < 3U)
    {
        return result;
    }

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t index = 0U; index < normalsA_in.size(); ++index)
    {
        if (!isFiniteVector(normalsA_in[index]) ||
            !isFiniteVector(normalsB_in[index]))
        {
            return result;
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
    return result;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
