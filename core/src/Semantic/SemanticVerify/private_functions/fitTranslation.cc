

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

/*! Solves N_B t = b (translation from offsets) via SVD, and reports
 * rank(N_B)/cond(N_B) for the observability gates. */
SemanticVerifyStatus
    fitTranslation(const Eigen::Matrix3d              &rotation_in,
                   const std::vector<Eigen::Vector3d> &normalsA_in,
                   const std::vector<double>          &offsetsA_in,
                   const std::vector<Eigen::Vector3d> &normalsB_in,
                   const std::vector<double>          &offsetsB_in,
                   TranslationFit                     &translation_out)
{
    TranslationFit    result;
    const std::size_t count = normalsA_in.size();
    if (count == 0U || offsetsA_in.size() != count ||
        normalsB_in.size() != count || offsetsB_in.size() != count)
    {
        translation_out = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    Eigen::MatrixXd N_B(static_cast<Eigen::Index>(count), 3);
    Eigen::VectorXd b(static_cast<Eigen::Index>(count));
    for (std::size_t index = 0U; index < count; ++index)
    {
        bool isFiniteVector2{};
        if (isFiniteVector(normalsB_in[index], isFiniteVector2) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // isFiniteVector cannot fail; continue as before.
        }
        if (!isFiniteVector2 || !std::isfinite(offsetsA_in[index]) ||
            !std::isfinite(offsetsB_in[index]))
        {
            translation_out = result;
            return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
        }
        N_B.row(static_cast<Eigen::Index>(index)) =
            normalsB_in[index].transpose();
        /* n_B^T t = sigma*d_A - d_B; sigma is always +1 here because both
         * sides are independently canonicalised via
         * Room::getWallNormalTowardRoom_World before this function ever
         * sees them (sign search is therefore a no-op). */
        b(static_cast<Eigen::Index>(index)) =
            offsetsA_in[index] - offsetsB_in[index];
    }
    static_cast<void>(
        rotation_in); // rotation already baked into normalsA_in via caller

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(N_B,
                                                Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
    const Eigen::VectorXd singularValues = svd.singularValues();

    constexpr double RANK_TOLERANCE = 1e-9;
    std::size_t      rank           = 0U;
    for (Eigen::Index index = 0; index < singularValues.size(); ++index)
    {
        if (singularValues(index) > RANK_TOLERANCE)
        {
            ++rank;
        }
    }
    result.rank = rank;
    result.conditionNumber =
        (singularValues.size() > 0 &&
         singularValues(singularValues.size() - 1) > RANK_TOLERANCE)
            ? singularValues(0) / singularValues(singularValues.size() - 1)
            : std::numeric_limits<double>::infinity();

    if (rank < 3U)
    {
        translation_out = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    result.translation = svd.solve(b);
    result.valid       = result.translation.allFinite();
    translation_out    = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
