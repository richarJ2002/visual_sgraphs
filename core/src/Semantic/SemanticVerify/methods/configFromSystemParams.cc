

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

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyConfig SemanticVerify::configFromSystemParams()
{
    const auto &loadedVerification =
        types::SystemParams::getParams()->verification;
    const auto &loadedFactor = types::SystemParams::getParams()->factor;

    SemanticVerifyConfig config;
    config.maxNormalAngle_deg =
        static_cast<double>(loadedVerification.maxNormalAngle_deg);
    config.maxOffset_m = static_cast<double>(loadedVerification.maxOffset_m);
    config.maxSupportDist_m =
        static_cast<double>(loadedVerification.maxSupportDist_m);
    config.minInlierRatio =
        static_cast<double>(loadedVerification.minInlierRatio);
    config.maxConditionNumber =
        static_cast<double>(loadedVerification.maxConditionNumber);
    config.ambiguityMarginInliers  = loadedVerification.ambiguityMarginInliers;
    config.maxWallsPerRoom         = loadedVerification.maxWallsPerRoom;
    config.maxHypotheses           = loadedVerification.maxHypotheses;
    config.maxSupportSamplePerWall = loadedVerification.maxSupportSamplePerWall;
    config.minAbsCosNormalAngle =
        static_cast<double>(loadedVerification.minAbsCosNormalAngle);
    config.sigmaTheta_rad = static_cast<double>(loadedFactor.sigmaTheta_rad);
    config.sigmaOffset_m  = static_cast<double>(loadedFactor.sigmaOffset_m);
    config.huberDelta     = static_cast<double>(loadedFactor.huberDelta);
    config.optimizerIterations = loadedFactor.optimizerIterations;
    return config;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
