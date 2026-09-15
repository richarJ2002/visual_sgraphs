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
#include "Types/SystemParams.h"

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
namespace
{

bool isFiniteVector(const Eigen::Vector3d &value_in)
{
    return value_in.allFinite();
}

/** One admitted (indexA, indexB) candidate correspondence -- not yet a
 * hypothesis, just a pairing considered for 3-subset enumeration. */
struct CandidatePair
{
    std::size_t indexA{0U};
    std::size_t indexB{0U};
};

struct RotationFit
{
    bool            valid{false};
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

/** Horn-style SVD rotation fit on signed normal correspondences (same
 * closed-form pattern as Utils::computeMapTransform_Horn's covariance/SVD
 * step, applied to plane normals per Section 9.3/11.1 instead of point
 * positions -- Horn's function itself is not called; it is point-based and
 * the plan reserves it as a Phase 0 legacy-characterization target only). */
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

struct TranslationFit
{
    bool            valid{false};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    std::size_t     rank{0U};
    double          conditionNumber{std::numeric_limits<double>::infinity()};
};

/** Solves N_B t = b (Section 9.3 "Translation from offsets") via SVD, and
 * reports rank(N_B)/cond(N_B) for the observability gates of Section 11.2. */
TranslationFit fitTranslation(const Eigen::Matrix3d              &rotation_in,
                              const std::vector<Eigen::Vector3d> &normalsA_in,
                              const std::vector<double>          &offsetsA_in,
                              const std::vector<Eigen::Vector3d> &normalsB_in,
                              const std::vector<double>          &offsetsB_in)
{
    TranslationFit    result;
    const std::size_t count = normalsA_in.size();
    if (count == 0U || offsetsA_in.size() != count ||
        normalsB_in.size() != count || offsetsB_in.size() != count)
    {
        return result;
    }

    Eigen::MatrixXd N_B(static_cast<Eigen::Index>(count), 3);
    Eigen::VectorXd b(static_cast<Eigen::Index>(count));
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (!isFiniteVector(normalsB_in[index]) ||
            !std::isfinite(offsetsA_in[index]) ||
            !std::isfinite(offsetsB_in[index]))
        {
            return result;
        }
        N_B.row(static_cast<Eigen::Index>(index)) =
            normalsB_in[index].transpose();
        /* n_B^T t = sigma*d_A - d_B; sigma is always +1 here because both
         * sides are independently canonicalised via
         * Room::getWallNormalTowardRoom_World before this function ever
         * sees them (Section 9.3's sign search is therefore a no-op). */
        b(static_cast<Eigen::Index>(index)) =
            offsetsA_in[index] - offsetsB_in[index];
    }
    static_cast<void>(
        rotation_in); // rotation already baked into normalsA_in via caller

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(N_B,
                                                Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
    const Eigen::VectorXd singularValues = svd.singularValues();

    constexpr double kRankTolerance = 1e-9;
    std::size_t      rank           = 0U;
    for (Eigen::Index index = 0; index < singularValues.size(); ++index)
    {
        if (singularValues(index) > kRankTolerance)
        {
            ++rank;
        }
    }
    result.rank = rank;
    result.conditionNumber =
        (singularValues.size() > 0 &&
         singularValues(singularValues.size() - 1) > kRankTolerance)
            ? singularValues(0) / singularValues(singularValues.size() - 1)
            : std::numeric_limits<double>::infinity();

    if (rank < 3U)
    {
        return result;
    }

    result.translation = svd.solve(b);
    result.valid       = result.translation.allFinite();
    return result;
}

double angleBetween_rad(const Eigen::Vector3d &first_in,
                        const Eigen::Vector3d &second_in)
{
    const double dot =
        std::clamp(first_in.normalized().dot(second_in.normalized()),
                   -1.0,
                   1.0);
    return std::acos(dot);
}

const VerifyWallObservation *
    findByWallId(const std::vector<VerifyWallObservation> &walls_in,
                 const int                                 wallId_in)
{
    const auto it =
        std::find_if(walls_in.begin(),
                     walls_in.end(),
                     [wallId_in](const VerifyWallObservation &wall_in)
                     { return wall_in.wallId == wallId_in; });
    return it == walls_in.end() ? nullptr : &(*it);
}

/** Symmetric point-to-plane support-cloud distance (Section 9.3 inlier
 * classification): sampled points from wall A, transformed by the
 * hypothesis, checked against wall B's plane; and the reverse. Returns the
 * larger (worse) of the two mean distances; 0.0 (vacuously passing) when
 * neither side has a usable sample, since not every synthetic/unit-test
 * observation populates a support cloud. */
double symmetricSupportDistance(const VerifyWallObservation &wallA_in,
                                const VerifyWallObservation &wallB_in,
                                const Eigen::Matrix3d       &rotation_in,
                                const Eigen::Vector3d       &translation_in)
{
    double      sum   = 0.0;
    std::size_t count = 0U;
    for (const Eigen::Vector3d &pointA : wallA_in.supportSample_World)
    {
        const Eigen::Vector3d pointB = rotation_in * pointA + translation_in;
        sum += std::abs(wallB_in.normal_World.dot(pointB) + wallB_in.d);
        ++count;
    }
    const Eigen::Matrix3d rotationInverse = rotation_in.transpose();
    const Eigen::Vector3d translationInverse =
        -rotationInverse * translation_in;
    for (const Eigen::Vector3d &pointB : wallB_in.supportSample_World)
    {
        const Eigen::Vector3d pointA =
            rotationInverse * pointB + translationInverse;
        sum += std::abs(wallA_in.normal_World.dot(pointA) + wallA_in.d);
        ++count;
    }
    return count == 0U ? 0.0 : sum / static_cast<double>(count);
}

} // namespace

SemanticVerifyConfig SemanticVerify::configFromSystemParams()
{
    const auto &loadedVerification = types::SystemParams::GetParams()->verification;
    const auto &loadedFactor       = types::SystemParams::GetParams()->factor;

    SemanticVerifyConfig config;
    config.maxNormalAngle_deg =
        static_cast<double>(loadedVerification.max_normal_angle_deg);
    config.maxOffset_m = static_cast<double>(loadedVerification.max_offset_m);
    config.maxSupportDist_m =
        static_cast<double>(loadedVerification.max_support_dist_m);
    config.minInlierRatio =
        static_cast<double>(loadedVerification.min_inlier_ratio);
    config.maxConditionNumber =
        static_cast<double>(loadedVerification.max_condition_number);
    config.ambiguityMarginInliers = loadedVerification.ambiguity_margin_inliers;
    config.maxWallsPerRoom        = loadedVerification.max_walls_per_room;
    config.maxHypotheses          = loadedVerification.max_hypotheses;
    config.maxSupportSamplePerWall =
        loadedVerification.max_support_sample_per_wall;
    config.minAbsCosNormalAngle =
        static_cast<double>(loadedVerification.min_abs_cos_normal_angle);
    config.sigmaTheta_rad = static_cast<double>(loadedFactor.sigma_theta_rad);
    config.sigmaOffset_m  = static_cast<double>(loadedFactor.sigma_offset_m);
    config.huberDelta     = static_cast<double>(loadedFactor.huber_delta);
    config.optimizerIterations = loadedFactor.optimizer_iterations;
    return config;
}

std::vector<VerifyWallObservation> SemanticVerify::collectWallObservations(
    const Room                 *p_room_in,
    const SemanticVerifyConfig &config_in)
{
    std::vector<VerifyWallObservation> observations;
    if (p_room_in == nullptr)
    {
        return observations;
    }

    const Eigen::Vector3d roomCentroid_World = p_room_in->getCentroid();
    if (!isFiniteVector(roomCentroid_World))
    {
        return observations;
    }

    for (geometric::Plane *p_wall : p_room_in->getWalls())
    {
        if (observations.size() == config_in.maxWallsPerRoom)
        {
            break;
        }
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        /* Re-derive the SAME oriented (n,d) pair Room::
         * getWallNormalTowardRoom_World computes internally, but keep d
         * paired with the (possibly sign-flipped) normal -- the existing
         * getter returns only the oriented normal, not a paired oriented d,
         * and n^T x + d = 0 requires both to flip together. */
        Eigen::Vector4d coeffs     = p_wall->getGlobalEquation().coeffs();
        const double    normalNorm = coeffs.head<3>().norm();
        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }
        coeffs /= normalNorm;
        if (!coeffs.allFinite())
        {
            continue;
        }
        const double signedDistance =
            coeffs.head<3>().dot(roomCentroid_World) + coeffs(3);
        if (!std::isfinite(signedDistance))
        {
            continue;
        }
        if (signedDistance < 0.0)
        {
            coeffs = -coeffs;
        }

        VerifyWallObservation observation;
        observation.wallId         = p_wall->getId();
        observation.normal_World   = coeffs.head<3>();
        observation.d              = coeffs(3);
        observation.centroid_World = p_wall->getCentroid();
        if (!isFiniteVector(observation.centroid_World))
        {
            continue;
        }

        const geometric::Plane::GeometrySnapshot snapshot = p_wall->getGeometrySnapshot();
        if (snapshot.supportCloud && !snapshot.supportCloud->empty())
        {
            const std::size_t total  = snapshot.supportCloud->size();
            const std::size_t stride = std::max<std::size_t>(
                1U,
                total / config_in.maxSupportSamplePerWall);
            for (std::size_t index = 0U;
                 index < total && observation.supportSample_World.size() <
                                      config_in.maxSupportSamplePerWall;
                 index += stride)
            {
                const auto &point = snapshot.supportCloud->points[index];
                if (!pcl::isFinite(point))
                {
                    continue;
                }
                observation.supportSample_World.emplace_back(
                    static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));
            }
        }

        observations.push_back(std::move(observation));
    }

    return observations;
}

SemanticVerifyResult
    SemanticVerify::verify(const std::vector<VerifyWallObservation> &wallsA_in,
                           const std::vector<VerifyWallObservation> &wallsB_in,
                           const SemanticVerifyConfig               &config_in)
{
    SemanticVerifyResult result;
    result.candidateWallPairCount = wallsA_in.size() * wallsB_in.size();

    /* Section 9.3: "minimal sample: 3 planes ... for full SE(3)". Fewer than
     * 3 walls on either side can never form a full-rank hypothesis. */
    if (wallsA_in.size() < 3U || wallsB_in.size() < 3U)
    {
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::TOO_FEW_WALLS;
        return result;
    }

    /* Full cross-product candidate set. No pre-filtering by raw normal
     * similarity here: the two rooms are in different, as-yet-unrelated map
     * frames, so a candidate pair's normals cannot be compared directly
     * before a hypothesis rotation exists (see the correspondence-safety
     * finding in the WP13 Phase 4 plan). Robustness to wrong candidate pairs
     * comes from the 3-subset hypothesis + inlier-count step below. */
    std::vector<CandidatePair> candidatePairs;
    candidatePairs.reserve(wallsA_in.size() * wallsB_in.size());
    for (std::size_t indexA = 0U; indexA < wallsA_in.size(); ++indexA)
    {
        for (std::size_t indexB = 0U; indexB < wallsB_in.size(); ++indexB)
        {
            candidatePairs.push_back({indexA, indexB});
        }
    }

    struct Hypothesis
    {
        Eigen::Matrix3d             rotation{Eigen::Matrix3d::Identity()};
        Eigen::Vector3d             translation{Eigen::Vector3d::Zero()};
        std::size_t                 rank{0U};
        double                      conditionNumber{0.0};
        std::vector<WallInlierPair> inliers;
    };

    const double maxNormalAngle_rad =
        config_in.maxNormalAngle_deg * M_PI / 180.0;

    /* All hypotheses passing the rank/condition-number gates. Deduplicated
     * by inlier-set signature after enumeration: distinct minimal (3-wall)
     * samples routinely rediscover the exact same correct solution on
     * well-conditioned, noise-free data, and that must not look like two
     * competing hypotheses to the ambiguity-rejection step below. */
    std::vector<Hypothesis> allHypotheses;
    std::size_t             hypothesesEvaluated = 0U;
    const std::size_t       pairCount           = candidatePairs.size();

    for (std::size_t i = 0U;
         i < pairCount && hypothesesEvaluated < config_in.maxHypotheses;
         ++i)
    {
        for (std::size_t j = i + 1U;
             j < pairCount && hypothesesEvaluated < config_in.maxHypotheses;
             ++j)
        {
            for (std::size_t k = j + 1U;
                 k < pairCount && hypothesesEvaluated < config_in.maxHypotheses;
                 ++k)
            {
                const CandidatePair &pair0 = candidatePairs[i];
                const CandidatePair &pair1 = candidatePairs[j];
                const CandidatePair &pair2 = candidatePairs[k];

                /* One-to-one constraint: 3 distinct A-walls, 3 distinct
                 * B-walls (an injective partial matching of size 3). */
                if (pair0.indexA == pair1.indexA ||
                    pair0.indexA == pair2.indexA ||
                    pair1.indexA == pair2.indexA)
                {
                    continue;
                }
                if (pair0.indexB == pair1.indexB ||
                    pair0.indexB == pair2.indexB ||
                    pair1.indexB == pair2.indexB)
                {
                    continue;
                }

                ++hypothesesEvaluated;

                const std::vector<Eigen::Vector3d> normalsA = {
                    wallsA_in[pair0.indexA].normal_World,
                    wallsA_in[pair1.indexA].normal_World,
                    wallsA_in[pair2.indexA].normal_World};
                const std::vector<Eigen::Vector3d> normalsB = {
                    wallsB_in[pair0.indexB].normal_World,
                    wallsB_in[pair1.indexB].normal_World,
                    wallsB_in[pair2.indexB].normal_World};

                const RotationFit rotationFit =
                    fitRotationFromNormals(normalsA, normalsB);
                if (!rotationFit.valid)
                {
                    continue;
                }

                std::vector<Eigen::Vector3d> rotatedNormalsA;
                rotatedNormalsA.reserve(3U);
                for (const Eigen::Vector3d &normal : normalsA)
                {
                    rotatedNormalsA.push_back(rotationFit.rotation * normal);
                }
                const std::vector<double> offsetsA = {
                    wallsA_in[pair0.indexA].d,
                    wallsA_in[pair1.indexA].d,
                    wallsA_in[pair2.indexA].d};
                const std::vector<double> offsetsB = {
                    wallsB_in[pair0.indexB].d,
                    wallsB_in[pair1.indexB].d,
                    wallsB_in[pair2.indexB].d};

                const TranslationFit translationFit =
                    fitTranslation(rotationFit.rotation,
                                   rotatedNormalsA,
                                   offsetsA,
                                   normalsB,
                                   offsetsB);
                if (!translationFit.valid || translationFit.rank < 3U)
                {
                    continue;
                }
                if (!std::isfinite(translationFit.conditionNumber) ||
                    translationFit.conditionNumber >
                        config_in.maxConditionNumber)
                {
                    continue;
                }

                /* Inlier classification over the full candidate set. */
                struct PassingPair
                {
                    std::size_t indexA{0U};
                    std::size_t indexB{0U};
                    double      normalAngle_rad{0.0};
                    double      offset_m{0.0};
                    double      supportDist_m{0.0};
                    double      combined{0.0};
                };
                std::vector<PassingPair> passing;
                for (const CandidatePair &candidate : candidatePairs)
                {
                    const VerifyWallObservation &wallA =
                        wallsA_in[candidate.indexA];
                    const VerifyWallObservation &wallB =
                        wallsB_in[candidate.indexB];
                    const Eigen::Vector3d predictedNormal =
                        rotationFit.rotation * wallA.normal_World;
                    const double predictedOffset =
                        wallA.d -
                        predictedNormal.dot(translationFit.translation);
                    const double normalAngle_rad =
                        angleBetween_rad(predictedNormal, wallB.normal_World);
                    const double offsetResidual_m =
                        std::abs(predictedOffset - wallB.d);

                    if (normalAngle_rad > maxNormalAngle_rad)
                    {
                        continue;
                    }
                    if (offsetResidual_m > config_in.maxOffset_m)
                    {
                        continue;
                    }
                    /* Explicit |cos(theta)| gate (Section 19.5), distinct
                     * from the angle gate above. */
                    if (std::abs(std::cos(normalAngle_rad)) <=
                        config_in.minAbsCosNormalAngle)
                    {
                        continue;
                    }
                    const double supportDist_m =
                        symmetricSupportDistance(wallA,
                                                 wallB,
                                                 rotationFit.rotation,
                                                 translationFit.translation);
                    if (supportDist_m > config_in.maxSupportDist_m)
                    {
                        continue;
                    }

                    passing.push_back({candidate.indexA,
                                       candidate.indexB,
                                       normalAngle_rad,
                                       offsetResidual_m,
                                       supportDist_m,
                                       normalAngle_rad + offsetResidual_m});
                }

                /* Greedy one-to-one dedup: best (lowest combined residual)
                 * correspondence wins each wall. */
                std::sort(
                    passing.begin(),
                    passing.end(),
                    [](const PassingPair &left_in, const PassingPair &right_in)
                    { return left_in.combined < right_in.combined; });
                std::vector<bool>           usedA(wallsA_in.size(), false);
                std::vector<bool>           usedB(wallsB_in.size(), false);
                std::vector<WallInlierPair> inliers;
                for (const PassingPair &candidate : passing)
                {
                    if (usedA[candidate.indexA] || usedB[candidate.indexB])
                    {
                        continue;
                    }
                    usedA[candidate.indexA] = true;
                    usedB[candidate.indexB] = true;
                    WallInlierPair inlierPair;
                    inlierPair.wallIdA = wallsA_in[candidate.indexA].wallId;
                    inlierPair.wallIdB = wallsB_in[candidate.indexB].wallId;
                    inlierPair.normalAngleResidual_rad =
                        candidate.normalAngle_rad;
                    inlierPair.offsetResidual_m      = candidate.offset_m;
                    inlierPair.supportDistResidual_m = candidate.supportDist_m;
                    inliers.push_back(inlierPair);
                }

                Hypothesis hypothesis;
                hypothesis.rotation        = rotationFit.rotation;
                hypothesis.translation     = translationFit.translation;
                hypothesis.rank            = translationFit.rank;
                hypothesis.conditionNumber = translationFit.conditionNumber;
                hypothesis.inliers         = std::move(inliers);
                allHypotheses.push_back(std::move(hypothesis));
            }
        }
    }

    if (allHypotheses.empty())
    {
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::NO_VALID_HYPOTHESIS;
        return result;
    }

    const auto signatureOf = [](const Hypothesis &hypothesis_in)
    {
        std::vector<std::pair<int, int>> signature;
        signature.reserve(hypothesis_in.inliers.size());
        for (const WallInlierPair &inlier : hypothesis_in.inliers)
        {
            signature.emplace_back(inlier.wallIdA, inlier.wallIdB);
        }
        std::sort(signature.begin(), signature.end());
        return signature;
    };

    /* Deduplicate by inlier-set signature, keeping the best-conditioned
     * representative per distinct signature, then rank the distinct
     * solutions by inlier count (ties by lower condition number). */
    std::vector<Hypothesis> distinctHypotheses;
    for (Hypothesis &candidate : allHypotheses)
    {
        const std::vector<std::pair<int, int>> candidateSignature =
            signatureOf(candidate);
        const auto existing =
            std::find_if(distinctHypotheses.begin(),
                         distinctHypotheses.end(),
                         [&](const Hypothesis &entry_in) {
                             return signatureOf(entry_in) == candidateSignature;
                         });
        if (existing == distinctHypotheses.end())
        {
            distinctHypotheses.push_back(std::move(candidate));
        }
        else if (candidate.conditionNumber < existing->conditionNumber)
        {
            *existing = std::move(candidate);
        }
    }
    std::sort(distinctHypotheses.begin(),
              distinctHypotheses.end(),
              [](const Hypothesis &left_in, const Hypothesis &right_in)
              {
                  if (left_in.inliers.size() != right_in.inliers.size())
                  {
                      return left_in.inliers.size() > right_in.inliers.size();
                  }
                  return left_in.conditionNumber < right_in.conditionNumber;
              });

    const std::vector<Hypothesis> best = std::move(distinctHypotheses);

    const std::size_t topInliers = best[0].inliers.size();
    const std::size_t runnerUpInliers =
        best.size() > 1U ? best[1].inliers.size() : 0U;

    /* Computed and attached to the result unconditionally from here on --
     * every remaining exit path (ambiguous, below-threshold, unobservable
     * refined fit, or PASS) has a genuine topInliers/runnerUp/ratio to
     * report, not the struct's zero default. */
    const double inlierRatio =
        static_cast<double>(topInliers) /
        static_cast<double>(std::min(wallsA_in.size(), wallsB_in.size()));
    result.topInlierCount      = topInliers;
    result.runnerUpInlierCount = runnerUpInliers;
    result.inlierRatio         = inlierRatio;

    if (topInliers < runnerUpInliers + config_in.ambiguityMarginInliers)
    {
        /* Insufficient discrimination between the top two DISTINCT
         * hypotheses. */
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::AMBIGUOUS_TOP_HYPOTHESES;
        return result;
    }

    if (inlierRatio < config_in.minInlierRatio)
    {
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::BELOW_MIN_INLIER_RATIO;
        return result;
    }

    const Hypothesis &seed = best[0];

    /* Nonlinear refinement (Section 17): one EdgePlaneTransformSE3 unary
     * factor per accepted inlier wall pair, Huber-robustified. */
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solverPtr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *algorithm =
        new g2o::OptimizationAlgorithmLevenberg(solverPtr);
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false);

    g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
    vertex->setEstimate(g2o::SE3Quat(seed.rotation, seed.translation));
    vertex->setId(0);
    vertex->setFixed(false);
    optimizer.addVertex(vertex);

    const double omegaTheta =
        1.0 / (config_in.sigmaTheta_rad * config_in.sigmaTheta_rad);
    const double omegaOffset =
        1.0 / (config_in.sigmaOffset_m * config_in.sigmaOffset_m);
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    information(0, 0)           = omegaTheta;
    information(1, 1)           = omegaTheta;
    information(2, 2)           = omegaOffset;

    for (const WallInlierPair &inlier : seed.inliers)
    {
        const VerifyWallObservation *observationA =
            findByWallId(wallsA_in, inlier.wallIdA);
        const VerifyWallObservation *observationB =
            findByWallId(wallsB_in, inlier.wallIdB);
        if (observationA == nullptr || observationB == nullptr)
        {
            continue;
        }

        PlanePairMeasurement measurement;
        measurement.n_A   = observationA->normal_World;
        measurement.d_A   = observationA->d;
        measurement.n_B   = observationB->normal_World;
        measurement.d_B   = observationB->d;
        measurement.sigma = 1;

        EdgePlaneTransformSE3 *edge = new EdgePlaneTransformSE3();
        edge->setVertex(0, vertex);
        edge->setMeasurement(measurement);
        edge->setInformation(information);
        g2o::RobustKernelHuber *kernel = new g2o::RobustKernelHuber();
        kernel->setDelta(config_in.huberDelta);
        edge->setRobustKernel(kernel);
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(static_cast<int>(config_in.optimizerIterations));

    const g2o::SE3Quat    refinedEstimate = vertex->estimate();
    const Eigen::Matrix3d refinedRotation =
        refinedEstimate.rotation().toRotationMatrix();
    const Eigen::Vector3d refinedTranslation = refinedEstimate.translation();

    /* Section 17.5 observability re-check on the refined transform: full
     * translational rank via N_B over the inlier set (as above), full
     * rotational rank via >=2 nonparallel inlier normal directions in the
     * surviving (B) frame (Section 11.3's stated equivalence). The complete
     * 6x6 Hessian SVD inspection Section 17.5 also describes is not
     * extracted from g2o's internal solver state here -- this is a
     * documented simplification, not a silent one. */
    std::vector<Eigen::Vector3d> inlierRotatedNormalsA;
    std::vector<double>          inlierOffsetsA;
    std::vector<Eigen::Vector3d> inlierNormalsB;
    std::vector<double>          inlierOffsetsB;
    double                       angularResidualSum = 0.0;
    std::vector<double>          angularResiduals;
    for (const WallInlierPair &inlier : seed.inliers)
    {
        const VerifyWallObservation *observationA =
            findByWallId(wallsA_in, inlier.wallIdA);
        const VerifyWallObservation *observationB =
            findByWallId(wallsB_in, inlier.wallIdB);
        if (observationA == nullptr || observationB == nullptr)
        {
            continue;
        }
        inlierRotatedNormalsA.push_back(refinedRotation *
                                        observationA->normal_World);
        inlierOffsetsA.push_back(observationA->d);
        inlierNormalsB.push_back(observationB->normal_World);
        inlierOffsetsB.push_back(observationB->d);
        angularResiduals.push_back(inlier.normalAngleResidual_rad);
        angularResidualSum += inlier.normalAngleResidual_rad;
    }

    const TranslationFit refinedFit = fitTranslation(refinedRotation,
                                                     inlierRotatedNormalsA,
                                                     inlierOffsetsA,
                                                     inlierNormalsB,
                                                     inlierOffsetsB);

    bool rotationObservable = false;
    for (std::size_t indexA = 0U;
         indexA < inlierNormalsB.size() && !rotationObservable;
         ++indexA)
    {
        for (std::size_t indexB = indexA + 1U; indexB < inlierNormalsB.size();
             ++indexB)
        {
            if (inlierNormalsB[indexA].cross(inlierNormalsB[indexB]).norm() >
                1e-3)
            {
                rotationObservable = true;
                break;
            }
        }
    }

    if (refinedFit.rank < 3U || !rotationObservable)
    {
        result.status          = VerificationStatus::REJECTED;
        result.rejectReason    = VerifyRejectReason::REFINED_FIT_NOT_OBSERVABLE;
        result.rank            = refinedFit.rank;
        result.conditionNumber = refinedFit.conditionNumber;
        return result;
    }

    std::sort(angularResiduals.begin(), angularResiduals.end());
    const double medianAngularResidual_rad =
        angularResiduals.empty()
            ? 0.0
            : angularResiduals[angularResiduals.size() / 2U];
    static_cast<void>(angularResidualSum);

    result.status                       = VerificationStatus::PASS;
    result.pass                         = true;
    result.transform_AToB               = Eigen::Isometry3d::Identity();
    result.transform_AToB.linear()      = refinedRotation;
    result.transform_AToB.translation() = refinedTranslation;
    result.inliers                      = seed.inliers;
    result.rank                         = refinedFit.rank;
    result.conditionNumber              = refinedFit.conditionNumber;
    result.normalisedConditionNumber =
        std::isfinite(refinedFit.conditionNumber)
            ? std::min(1.0,
                       refinedFit.conditionNumber /
                           config_in.maxConditionNumber)
            : 1.0;
    result.inlierRatio         = inlierRatio;
    result.angularResidual_rad = medianAngularResidual_rad;
    result.confidence =
        std::clamp(inlierRatio * (1.0 - 0.5 * result.normalisedConditionNumber),
                   0.0,
                   1.0);

    return result;
}

bool SemanticVerify::runFloorGate(
    SemanticVerifyResult    &result_inout,
    core::Map                     *p_survivingMap_in,
    core::Map                     *p_absorbedMap_in,
    const Eigen::Isometry3d &transform_absorbedToSurviving_in)
{
    const g2o::Sim3 transform(transform_absorbedToSurviving_in.linear(),
                              transform_absorbedToSurviving_in.translation(),
                              1.0);
    std::string     resultText;
    const bool      passed       = verifyLoopMergeFloors(p_survivingMap_in,
                                              p_absorbedMap_in,
                                              transform,
                                              resultText);
    result_inout.floorGateRan    = true;
    result_inout.floorGatePassed = passed;
    result_inout.floorGateResult = resultText;
    return passed;
}

namespace
{
std::string stableRoomIdentity(const RoomContextSnapshot &context_in)
{
    if (!context_in.roomTag.empty())
    {
        return context_in.roomTag;
    }
    return std::string("room_") + std::to_string(context_in.roomId);
}

enum class AlignmentCheck
{
    ALIGNED,
    MISSING,
    CONTRADICTION
};

AlignmentCheck checkFixedTransformWalls(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<VerifyWallObservation> &absorbedWalls_in,
    const g2o::Sim3                          &transform_in,
    const SemanticVerifyConfig               &config_in,
    std::size_t                              &matchedCount_out)
{
    matchedCount_out = 0U;
    if (survivingWalls_in.size() < 3U || absorbedWalls_in.size() < 3U)
    {
        return AlignmentCheck::MISSING;
    }

    const double scale = transform_in.scale();
    if (!std::isfinite(scale) || scale <= 0.0)
    {
        return AlignmentCheck::CONTRADICTION;
    }
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    const Eigen::Vector3d translation =
        transform_in.translation().cast<double>();
    if (!rotation.allFinite() || !translation.allFinite())
    {
        return AlignmentCheck::CONTRADICTION;
    }

    std::set<std::size_t> usedSurvivingWalls;
    for (const VerifyWallObservation &absorbedWall : absorbedWalls_in)
    {
        const Eigen::Vector3d transformedNormal =
            rotation * absorbedWall.normal_World;
        const double transformedOffset =
            scale * absorbedWall.d - transformedNormal.dot(translation);
        if (!transformedNormal.allFinite() || !std::isfinite(transformedOffset))
        {
            continue;
        }

        std::size_t bestIndex    = survivingWalls_in.size();
        double      bestResidual = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0U; index < survivingWalls_in.size(); ++index)
        {
            if (usedSurvivingWalls.count(index) > 0U)
            {
                continue;
            }
            const VerifyWallObservation &survivingWall =
                survivingWalls_in[index];
            const double cosine =
                std::clamp(transformedNormal.dot(survivingWall.normal_World),
                           -1.0,
                           1.0);
            const double angle_deg =
                std::acos(cosine) * 180.0 / std::acos(-1.0);
            const double offset_m =
                std::abs(transformedOffset - survivingWall.d);
            if (angle_deg > config_in.maxNormalAngle_deg ||
                offset_m > config_in.maxOffset_m)
            {
                continue;
            }
            const double residual =
                angle_deg / std::max(config_in.maxNormalAngle_deg, 1e-9) +
                offset_m / std::max(config_in.maxOffset_m, 1e-9);
            if (residual < bestResidual)
            {
                bestResidual = residual;
                bestIndex    = index;
            }
        }
        if (bestIndex != survivingWalls_in.size())
        {
            usedSurvivingWalls.insert(bestIndex);
            ++matchedCount_out;
        }
    }

    const std::size_t evidenceCount =
        std::max(survivingWalls_in.size(), absorbedWalls_in.size());
    const double inlierRatio = static_cast<double>(matchedCount_out) /
                               static_cast<double>(evidenceCount);
    return matchedCount_out >= 3U && inlierRatio >= config_in.minInlierRatio
               ? AlignmentCheck::ALIGNED
               : AlignmentCheck::CONTRADICTION;
}

AlignmentCheck
    checkPassageTopology(const RoomContextSnapshot  &survivingContext_in,
                         const RoomContextSnapshot  &absorbedContext_in,
                         const g2o::Sim3            &transform_in,
                         const SemanticVerifyConfig &config_in,
                         std::size_t                &matchedCount_out,
                         SemanticMergeReason        &contradictionReason_out)
{
    matchedCount_out = 0U;
    if (survivingContext_in.passageContexts.empty() ||
        absorbedContext_in.passageContexts.empty())
    {
        return AlignmentCheck::MISSING;
    }

    std::map<int, const PassageContext *> survivingPassages;
    std::map<int, const PassageContext *> absorbedPassages;
    for (const PassageContext &passage : survivingContext_in.passageContexts)
    {
        survivingPassages.emplace(passage.id, &passage);
    }
    for (const PassageContext &passage : absorbedContext_in.passageContexts)
    {
        absorbedPassages.emplace(passage.id, &passage);
    }

    bool hasIncompletePassage =
        survivingPassages.size() != absorbedPassages.size();
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    for (const auto &entry : absorbedPassages)
    {
        const std::map<int, const PassageContext *>::const_iterator match =
            survivingPassages.find(entry.first);
        if (match == survivingPassages.end())
        {
            hasIncompletePassage = true;
            continue;
        }

        const PassageContext &absorbedPassage  = *entry.second;
        const PassageContext &survivingPassage = *match->second;
        ++matchedCount_out;
        if (absorbedPassage.passable != survivingPassage.passable)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
            return AlignmentCheck::CONTRADICTION;
        }
        if (absorbedPassage.hasKnownSideRoom !=
                survivingPassage.hasKnownSideRoom ||
            absorbedPassage.hasFarSideRoom != survivingPassage.hasFarSideRoom)
        {
            hasIncompletePassage = true;
        }
        if (!absorbedPassage.hasKnownSideRoom ||
            !survivingPassage.hasKnownSideRoom ||
            !absorbedPassage.hasFarSideRoom || !survivingPassage.hasFarSideRoom)
        {
            hasIncompletePassage = true;
        }
        if (absorbedPassage.hasKnownSideRoom &&
            survivingPassage.hasKnownSideRoom &&
            absorbedPassage.knownSideRoomId != survivingPassage.knownSideRoomId)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
            return AlignmentCheck::CONTRADICTION;
        }
        if (absorbedPassage.hasFarSideRoom && survivingPassage.hasFarSideRoom &&
            absorbedPassage.secondaryRoomId != survivingPassage.secondaryRoomId)
        {
            contradictionReason_out =
                SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
            return AlignmentCheck::CONTRADICTION;
        }
        if (absorbedPassage.hasKnownSideDirection !=
            survivingPassage.hasKnownSideDirection)
        {
            hasIncompletePassage = true;
        }
        if (!absorbedPassage.hasKnownSideDirection ||
            !survivingPassage.hasKnownSideDirection)
        {
            hasIncompletePassage = true;
        }
        if (absorbedPassage.hasKnownSideDirection &&
            survivingPassage.hasKnownSideDirection)
        {
            const Eigen::Vector3d transformedDirection =
                rotation * absorbedPassage.knownSideDirection_World;
            const double directionAgreement =
                transformedDirection.normalized().dot(
                    survivingPassage.knownSideDirection_World.normalized());
            if (!std::isfinite(directionAgreement) ||
                directionAgreement < config_in.minAbsCosNormalAngle)
            {
                contradictionReason_out =
                    SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION;
                return AlignmentCheck::CONTRADICTION;
            }
        }
    }

    if (matchedCount_out == 0U)
    {
        contradictionReason_out =
            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
        return AlignmentCheck::CONTRADICTION;
    }
    return hasIncompletePassage ? AlignmentCheck::MISSING
                                : AlignmentCheck::ALIGNED;
}

SemanticMergeRoomEvidence
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &config_in)
{
    SemanticMergeRoomEvidence evidence;
    evidence.context.roomId   = p_room_in->getId();
    evidence.context.roomTag  = p_room_in->getRoomTag();
    evidence.context.centroid = p_room_in->getCentroid();
    Floor *p_floor            = p_room_in->getFloor();
    evidence.context.floorId  = p_floor != nullptr ? p_floor->getId() : -1;
    evidence.walls =
        SemanticVerify::collectWallObservations(p_room_in, config_in);
    for (Passage *p_passage : p_room_in->getPassages())
    {
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }
        PassageContext context;
        context.id              = p_passage->getId();
        context.passable        = p_passage->isPassable();
        context.centroid_World  = p_passage->getCentroid();
        context.width_m         = p_passage->getWidth();
        context.height_m        = p_passage->getHeight();
        context.isRecoveryProxy = p_passage->isRecoveryProxy();
        context.apertureValid   = std::isfinite(context.width_m) &&
                                std::isfinite(context.height_m) &&
                                context.width_m > 0.0 && context.height_m > 0.0;
        const Passage::KnownSideProvenance knownSide =
            p_passage->getKnownSideProvenance();
        context.hasKnownSideRoom = knownSide.pRoom != nullptr;
        if (context.hasKnownSideRoom)
        {
            context.knownSideRoomId = knownSide.pRoom->getId();
        }
        context.hasKnownSideDirection = knownSide.hasDirection();
        if (context.hasKnownSideDirection)
        {
            context.knownSideDirection_World = knownSide.direction_World;
        }
        const std::optional<int> farSideRoomId =
            p_passage->getProspectiveRoomId();
        context.hasFarSideRoom = farSideRoomId.has_value();
        if (farSideRoomId.has_value())
        {
            context.secondaryRoomId = *farSideRoomId;
        }
        evidence.context.passageContexts.push_back(context);
    }
    return evidence;
}

/*! @brief Consecutive-map anchor: one tag-matched room pair. Tags are the
 * only correspondence key; map-local IDs are never compared across maps. */
struct ConsecutiveAnchorPair
{
    const SemanticMergeRoomEvidence *p_surviving{nullptr};
    const SemanticMergeRoomEvidence *p_absorbed{nullptr};
};

std::vector<ConsecutiveAnchorPair> collectConsecutiveAnchors(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in)
{
    std::map<std::string, const SemanticMergeRoomEvidence *> survivingByTag;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        if (!room.context.roomTag.empty())
        {
            survivingByTag.emplace(room.context.roomTag, &room);
        }
    }
    std::vector<ConsecutiveAnchorPair> pairs;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        if (room.context.roomTag.empty())
        {
            continue;
        }
        const auto match = survivingByTag.find(room.context.roomTag);
        if (match != survivingByTag.end())
        {
            ConsecutiveAnchorPair pair;
            pair.p_surviving = match->second;
            pair.p_absorbed  = &room;
            pairs.push_back(pair);
        }
    }
    return pairs;
}

bool transformAbsorbedPoint(const g2o::Sim3       &transform_in,
                            const Eigen::Vector3d &point_in,
                            Eigen::Vector3d       &mapped_out)
{
    const double          scale = transform_in.scale();
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    const Eigen::Vector3d translation =
        transform_in.translation().cast<double>();
    if (!std::isfinite(scale) || scale <= 0.0 || !rotation.allFinite() ||
        !translation.allFinite() || !point_in.allFinite())
    {
        return false;
    }
    mapped_out = scale * (rotation * point_in) + translation;
    return mapped_out.allFinite();
}

bool checkConsecutiveFloors(core::Map             *p_survivingMap_in,
                            core::Map             *p_absorbedMap_in,
                            const g2o::Sim3 &transform_in,
                            double           maximumOffset_m_in,
                            std::string     &decision_out)
{
    Floor *p_survivingFloor =
        Floor::selectBestObservedFloor(p_survivingMap_in->GetAllFloors());
    Floor *p_absorbedFloor =
        Floor::selectBestObservedFloor(p_absorbedMap_in->GetAllFloors());
    const std::optional<Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? p_survivingFloor->getPlaneIdentity()
                                    : std::nullopt;
    const std::optional<Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? p_absorbedFloor->getPlaneIdentity()
                                   : std::nullopt;
    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        decision_out = "DEFERRED";
        return false;
    }
    const std::optional<Floor::PlaneIdentity> transformedIdentity =
        Floor::transformPlaneIdentity(*absorbedIdentity, transform_in);
    double     normalAngle_deg = std::numeric_limits<double>::infinity();
    double     offset_m        = std::numeric_limits<double>::infinity();
    const bool floorsMatch =
        transformedIdentity.has_value() &&
        Floor::planeIdentitiesMatch(*survivingIdentity,
                                    *transformedIdentity,
                                    Floor::kMergeMaxPlaneNormalAngle_deg,
                                    maximumOffset_m_in,
                                    normalAngle_deg,
                                    offset_m);
    std::cout << "[ConsecutiveMerge] Floor check: "
              << (floorsMatch ? "ACCEPTED" : "REJECTED")
              << " (angle=" << normalAngle_deg << " deg, offset=" << offset_m
              << " m; limits=" << Floor::kMergeMaxPlaneNormalAngle_deg
              << " deg/" << maximumOffset_m_in << " m)." << std::endl;
    decision_out = floorsMatch ? "ACCEPTED" : "REJECTED";
    return floorsMatch;
}

AlignmentCheck
    checkAnchorRoomCentroids(const std::vector<ConsecutiveAnchorPair> &pairs_in,
                             const g2o::Sim3 &transform_in,
                             double           maximumDistance_m_in)
{
    if (pairs_in.empty())
    {
        return AlignmentCheck::MISSING;
    }
    for (const ConsecutiveAnchorPair &pair : pairs_in)
    {
        Eigen::Vector3d mappedCentroid = Eigen::Vector3d::Zero();
        if (!transformAbsorbedPoint(transform_in,
                                    pair.p_absorbed->context.centroid,
                                    mappedCentroid) ||
            !pair.p_surviving->context.centroid.allFinite())
        {
            return AlignmentCheck::MISSING;
        }
        if ((mappedCentroid - pair.p_surviving->context.centroid).norm() >
            maximumDistance_m_in)
        {
            return AlignmentCheck::CONTRADICTION;
        }
    }
    return AlignmentCheck::ALIGNED;
}

bool passageGeometryIsUsable(const PassageContext &context_in)
{
    return context_in.centroid_World.allFinite() &&
           std::isfinite(context_in.width_m) && context_in.width_m > 0.0 &&
           std::isfinite(context_in.height_m) && context_in.height_m > 0.0;
}

/*! @brief Minimum known-side direction agreement for paired passages. Mirrors
 * the verification default (SemanticVerifyConfig::minAbsCosNormalAngle). */
constexpr double kConsecutiveMinDirectionAgreement = 0.85;

AlignmentCheck checkConsecutivePassageTopology(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3                              &transform_in,
    double                                        maximumCentroidDistance_m_in,
    std::size_t                                  &matchedCount_out,
    SemanticMergeReason                          &contradictionReason_out)
{
    matchedCount_out          = 0U;
    bool hasIncompletePassage = false;

    /* Endpoint identity is tag-based: "t:<tag>" for tagged rooms (stable
     * across maps), side-prefixed "a:u:<id>"/"s:u:<id>" for untagged ones
     * (never equal across maps), empty for a genuinely absent side. */
    std::map<int, std::string> survivingIdToTag;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        survivingIdToTag.emplace(room.context.roomId, room.context.roomTag);
    }
    std::map<int, std::string> absorbedIdToTag;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        absorbedIdToTag.emplace(room.context.roomId, room.context.roomTag);
    }
    const auto endpointKey = [](bool                              hasSide_in,
                                int                               roomId_in,
                                const std::map<int, std::string> &idToTag_in,
                                const char                       *sidePrefix_in)
    {
        if (!hasSide_in)
        {
            return std::string();
        }
        const auto tagMatch = idToTag_in.find(roomId_in);
        if (tagMatch != idToTag_in.end() && !tagMatch->second.empty())
        {
            return std::string("t:") + tagMatch->second;
        }
        return std::string(sidePrefix_in) + std::to_string(roomId_in);
    };

    std::vector<const PassageContext *>   survivingPassages;
    std::map<int, const PassageContext *> survivingById;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        for (const PassageContext &passage : room.context.passageContexts)
        {
            survivingById.emplace(passage.id, &passage);
            if (passageGeometryIsUsable(passage))
            {
                survivingPassages.push_back(&passage);
            }
        }
    }

    bool hasAbsorbedPassageEvidence = false;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        for (const PassageContext &absorbedPassage :
             room.context.passageContexts)
        {
            /* Same stable ID means same doorway lineage: passage IDs are
             * mission-unique, so a proxy on either side pairs by lineage
             * even without geometry, and content re-derives at fusion.
             * A both-real same-ID pair still runs the full endpoint and
             * geometry checks below. */
            const auto lineageMatch = survivingById.find(absorbedPassage.id);
            if (lineageMatch != survivingById.end() &&
                (absorbedPassage.isRecoveryProxy ||
                 lineageMatch->second->isRecoveryProxy))
            {
                if (absorbedPassage.isRecoveryProxy &&
                    lineageMatch->second->isRecoveryProxy)
                {
                    /* Dormant pair: no constraint either way. */
                    continue;
                }
                if (lineageMatch->second->passable != absorbedPassage.passable)
                {
                    contradictionReason_out =
                        SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                    return AlignmentCheck::CONTRADICTION;
                }
                ++matchedCount_out;
                hasAbsorbedPassageEvidence = true;
                continue;
            }
            if (!passageGeometryIsUsable(absorbedPassage))
            {
                /* Recovery proxies and geometry-less hypotheses contribute
                 * no constraints. */
                continue;
            }
            hasAbsorbedPassageEvidence = true;
            const std::string absorbedKnownKey =
                endpointKey(absorbedPassage.hasKnownSideRoom,
                            absorbedPassage.knownSideRoomId,
                            absorbedIdToTag,
                            "a:u:");
            const std::string absorbedFarKey =
                endpointKey(absorbedPassage.hasFarSideRoom,
                            absorbedPassage.secondaryRoomId,
                            absorbedIdToTag,
                            "a:u:");

            bool                  paired                 = false;
            const PassageContext *p_conflictingSurviving = nullptr;
            for (const PassageContext *p_surviving : survivingPassages)
            {
                const std::string survivingKnownKey =
                    endpointKey(p_surviving->hasKnownSideRoom,
                                p_surviving->knownSideRoomId,
                                survivingIdToTag,
                                "s:u:");
                const std::string survivingFarKey =
                    endpointKey(p_surviving->hasFarSideRoom,
                                p_surviving->secondaryRoomId,
                                survivingIdToTag,
                                "s:u:");
                if (absorbedPassage.hasKnownSideRoom ==
                        p_surviving->hasKnownSideRoom &&
                    absorbedPassage.hasFarSideRoom ==
                        p_surviving->hasFarSideRoom &&
                    absorbedKnownKey == survivingKnownKey &&
                    absorbedFarKey == survivingFarKey)
                {
                    paired = true;
                    if (p_surviving->passable != absorbedPassage.passable)
                    {
                        contradictionReason_out =
                            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                        return AlignmentCheck::CONTRADICTION;
                    }
                    if (absorbedPassage.hasKnownSideDirection &&
                        p_surviving->hasKnownSideDirection)
                    {
                        const Eigen::Vector3d transformedDirection =
                            transform_in.rotation()
                                .toRotationMatrix()
                                .cast<double>() *
                            absorbedPassage.knownSideDirection_World;
                        const double directionAgreement =
                            transformedDirection.normalized().dot(
                                p_surviving->knownSideDirection_World
                                    .normalized());
                        if (!std::isfinite(directionAgreement) ||
                            directionAgreement <
                                kConsecutiveMinDirectionAgreement)
                        {
                            contradictionReason_out = SemanticMergeReason::
                                PASSAGE_DIRECTION_CONTRADICTION;
                            return AlignmentCheck::CONTRADICTION;
                        }
                    }
                    else
                    {
                        hasIncompletePassage = true;
                    }
                    Eigen::Vector3d mappedCentroid = Eigen::Vector3d::Zero();
                    if (!transformAbsorbedPoint(transform_in,
                                                absorbedPassage.centroid_World,
                                                mappedCentroid))
                    {
                        hasIncompletePassage = true;
                    }
                    else if ((mappedCentroid - p_surviving->centroid_World)
                                 .norm() > maximumCentroidDistance_m_in)
                    {
                        contradictionReason_out =
                            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
                        return AlignmentCheck::CONTRADICTION;
                    }
                }
                else
                {
                    /* Partial overlap on one translated present side while
                     * both sides disagree on the other is positive evidence
                     * of different doorways, not of missing evidence. */
                    if (!absorbedKnownKey.empty() &&
                        absorbedKnownKey == survivingKnownKey &&
                        absorbedPassage.hasFarSideRoom &&
                        p_surviving->hasFarSideRoom &&
                        !absorbedFarKey.empty() && !survivingFarKey.empty() &&
                        absorbedFarKey != survivingFarKey)
                    {
                        p_conflictingSurviving = p_surviving;
                    }
                    if (!absorbedFarKey.empty() &&
                        absorbedFarKey == survivingFarKey &&
                        absorbedPassage.hasKnownSideRoom &&
                        p_surviving->hasKnownSideRoom &&
                        !absorbedKnownKey.empty() &&
                        !survivingKnownKey.empty() &&
                        absorbedKnownKey != survivingKnownKey)
                    {
                        p_conflictingSurviving = p_surviving;
                    }
                }
            }
            if (paired)
            {
                ++matchedCount_out;
            }
            else if (p_conflictingSurviving != nullptr)
            {
                contradictionReason_out =
                    SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION;
                return AlignmentCheck::CONTRADICTION;
            }
            else
            {
                hasIncompletePassage = true;
            }
        }
    }

    if (!hasAbsorbedPassageEvidence)
    {
        contradictionReason_out = SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        return AlignmentCheck::MISSING;
    }
    if (matchedCount_out == 0U)
    {
        contradictionReason_out =
            SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION;
        return AlignmentCheck::CONTRADICTION;
    }
    return hasIncompletePassage ? AlignmentCheck::MISSING
                                : AlignmentCheck::ALIGNED;
}

/*! @brief Maximum plane offset for wall pairing. Mirrors the verification
 * default (SemanticVerifyConfig::maxOffset_m); the map_merge section tunes
 * the angle and the edge overlap, not the offset. */
constexpr double kConsecutiveMaxPlaneOffset_m = 0.35;

bool wallSamplesSpanInterval(const VerifyWallObservation &wall_in,
                             const Eigen::Vector3d       &axis_in,
                             const Eigen::Vector3d       &origin_in,
                             double                      &minimum_out,
                             double                      &maximum_out)
{
    bool   hasSample = false;
    double minimum   = std::numeric_limits<double>::infinity();
    double maximum   = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &sample : wall_in.supportSample_World)
    {
        if (!sample.allFinite())
        {
            continue;
        }
        const double coordinate = (sample - origin_in).dot(axis_in);
        minimum                 = std::min(minimum, coordinate);
        maximum                 = std::max(maximum, coordinate);
        hasSample               = true;
    }
    if (!hasSample)
    {
        return false;
    }
    minimum_out = minimum;
    maximum_out = maximum;
    return true;
}

/*! @brief Requires coplanar wall pairs to also overlap along the wall
 * direction. Same infinite plane with disjoint extents means different
 * walls (or different places): positive disjointness evidence contradicts,
 * while walls without enough samples are skipped (the angle/offset core
 * owns their verdict). */
AlignmentCheck checkConsecutiveWallEdgeOverlap(
    const std::vector<ConsecutiveAnchorPair> &pairs_in,
    const g2o::Sim3                          &transform_in,
    double                                    maximumNormalAngle_deg_in,
    double                                    minimumOverlap_m_in)
{
    for (const ConsecutiveAnchorPair &pair : pairs_in)
    {
        for (const VerifyWallObservation &absorbedWall : pair.p_absorbed->walls)
        {
            if (absorbedWall.supportSample_World.size() < 2U ||
                !absorbedWall.normal_World.allFinite() ||
                !std::isfinite(absorbedWall.d) ||
                !absorbedWall.centroid_World.allFinite())
            {
                continue;
            }
            const double absorbedNormalNorm = absorbedWall.normal_World.norm();
            if (!std::isfinite(absorbedNormalNorm) || absorbedNormalNorm < 1e-8)
            {
                continue;
            }
            /* Directions rotate only; the offset carries scale and
             * translation (same plane convention as
             * checkFixedTransformWalls). */
            const double          absorbedScale = transform_in.scale();
            const Eigen::Matrix3d absorbedRotation =
                transform_in.rotation().toRotationMatrix().cast<double>();
            const Eigen::Vector3d absorbedTranslation =
                transform_in.translation().cast<double>();
            if (!std::isfinite(absorbedScale) || absorbedScale <= 0.0 ||
                !absorbedRotation.allFinite() ||
                !absorbedTranslation.allFinite())
            {
                continue;
            }
            const Eigen::Vector3d mappedNormal =
                absorbedRotation * absorbedWall.normal_World;
            const double mappedOffset = absorbedScale * absorbedWall.d -
                                        mappedNormal.dot(absorbedTranslation);
            bool hasOverlapPartner = false;
            bool hasCompatibleWall = false;
            for (const VerifyWallObservation &survivingWall :
                 pair.p_surviving->walls)
            {
                if (survivingWall.supportSample_World.size() < 2U ||
                    !survivingWall.normal_World.allFinite() ||
                    !std::isfinite(survivingWall.d) ||
                    !survivingWall.centroid_World.allFinite())
                {
                    continue;
                }
                const double survivingNormalNorm =
                    survivingWall.normal_World.norm();
                if (!std::isfinite(survivingNormalNorm) ||
                    survivingNormalNorm < 1e-8)
                {
                    continue;
                }
                const double cosine = std::clamp(
                    mappedNormal.dot(survivingWall.normal_World) /
                        std::max(mappedNormal.norm() * survivingNormalNorm,
                                 1e-9),
                    -1.0,
                    1.0);
                const double angle_deg =
                    std::acos(cosine) * 180.0 / std::acos(-1.0);
                const double offset_m =
                    std::abs(mappedOffset - survivingWall.d);
                if (angle_deg > maximumNormalAngle_deg_in ||
                    offset_m > kConsecutiveMaxPlaneOffset_m)
                {
                    continue;
                }
                hasCompatibleWall = true;
                /* In-plane axis from the surviving wall normal. Walls are
                 * near-vertical by admission (max_tilt_wall), so normal x
                 * world-Z spans the wall length. */
                Eigen::Vector3d axis =
                    survivingWall.normal_World.cross(Eigen::Vector3d::UnitZ());
                if (axis.squaredNorm() < 1e-8)
                {
                    axis = survivingWall.normal_World.cross(
                        Eigen::Vector3d::UnitX());
                }
                if (axis.squaredNorm() < 1e-8)
                {
                    continue;
                }
                axis.normalize();
                double survivingMinimum = 0.0;
                double survivingMaximum = 0.0;
                if (!wallSamplesSpanInterval(survivingWall,
                                             axis,
                                             survivingWall.centroid_World,
                                             survivingMinimum,
                                             survivingMaximum))
                {
                    continue;
                }
                double absorbedMinimum = 0.0;
                double absorbedMaximum = 0.0;
                bool   hasMappedSample = false;
                double mappedMinimum = std::numeric_limits<double>::infinity();
                double mappedMaximum = -std::numeric_limits<double>::infinity();
                for (const Eigen::Vector3d &sample :
                     absorbedWall.supportSample_World)
                {
                    Eigen::Vector3d mappedSample = Eigen::Vector3d::Zero();
                    if (!transformAbsorbedPoint(transform_in,
                                                sample,
                                                mappedSample))
                    {
                        continue;
                    }
                    const double coordinate =
                        (mappedSample - survivingWall.centroid_World).dot(axis);
                    mappedMinimum   = std::min(mappedMinimum, coordinate);
                    mappedMaximum   = std::max(mappedMaximum, coordinate);
                    hasMappedSample = true;
                }
                if (!hasMappedSample)
                {
                    continue;
                }
                absorbedMinimum = mappedMinimum;
                absorbedMaximum = mappedMaximum;
                const double overlap_m =
                    std::min(survivingMaximum, absorbedMaximum) -
                    std::max(survivingMinimum, absorbedMinimum);
                if (overlap_m >= minimumOverlap_m_in)
                {
                    hasOverlapPartner = true;
                    break;
                }
            }
            if (hasCompatibleWall && !hasOverlapPartner)
            {
                return AlignmentCheck::CONTRADICTION;
            }
        }
    }
    return AlignmentCheck::ALIGNED;
}
} // namespace

SemanticMergeGateResult SemanticVerify::evaluateMergeAlignment(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3            &transform_absorbedToSurviving_in,
    const SemanticVerifyConfig &config_in)
{
    SemanticMergeGateResult                                  result;
    std::map<std::string, const SemanticMergeRoomEvidence *> survivingById;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        survivingById.emplace(stableRoomIdentity(room.context), &room);
    }

    bool hasMissingEvidence = false;
    for (const SemanticMergeRoomEvidence &absorbedRoom : absorbedRooms_in)
    {
        const std::map<std::string,
                       const SemanticMergeRoomEvidence *>::const_iterator
            match =
                survivingById.find(stableRoomIdentity(absorbedRoom.context));
        if (match == survivingById.end())
        {
            continue;
        }
        ++result.sharedRoomCount;

        std::size_t          matchedWalls = 0U;
        const AlignmentCheck wallCheck =
            checkFixedTransformWalls(match->second->walls,
                                     absorbedRoom.walls,
                                     transform_absorbedToSurviving_in,
                                     config_in,
                                     matchedWalls);
        result.matchedWallCount += matchedWalls;
        if (wallCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            return result;
        }
        if (wallCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        }

        std::size_t         matchedPassages = 0U;
        SemanticMergeReason topologyReason =
            SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        const AlignmentCheck topologyCheck =
            checkPassageTopology(match->second->context,
                                 absorbedRoom.context,
                                 transform_absorbedToSurviving_in,
                                 config_in,
                                 matchedPassages,
                                 topologyReason);
        result.matchedPassageCount += matchedPassages;
        if (topologyCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = topologyReason;
            return result;
        }
        if (topologyCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
        }
        if (wallCheck == AlignmentCheck::ALIGNED &&
            topologyCheck == AlignmentCheck::ALIGNED)
        {
            ++result.alignedRoomCount;
        }
    }

    if (result.sharedRoomCount == 0U)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        return result;
    }
    if (hasMissingEvidence || result.alignedRoomCount != result.sharedRoomCount)
    {
        result.decision = SemanticMergeDecision::DEFER;
        return result;
    }
    result.decision = SemanticMergeDecision::ACCEPT;
    result.reason   = SemanticMergeReason::ALIGNED;
    return result;
}

SemanticMergeGateResult SemanticVerify::evaluateMapMergeGate(
    core::Map                        *p_survivingMap_in,
    core::Map                        *p_absorbedMap_in,
    const g2o::Sim3            &transform_absorbedToSurviving_in,
    const SemanticVerifyConfig &config_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        return result;
    }

    if (!verifyLoopMergeFloors(p_survivingMap_in,
                               p_absorbedMap_in,
                               transform_absorbedToSurviving_in,
                               result.floorDecision))
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        return result;
    }

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    for (Room *p_room : p_survivingMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            survivingRooms.push_back(copyMergeRoomEvidence(p_room, config_in));
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    for (Room *p_room : p_absorbedMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            absorbedRooms.push_back(copyMergeRoomEvidence(p_room, config_in));
        }
    }
    result               = evaluateMergeAlignment(survivingRooms,
                                    absorbedRooms,
                                    transform_absorbedToSurviving_in,
                                    config_in);
    result.floorDecision = "ACCEPTED";
    return result;
}

SemanticVerify::MapMergeConfig SemanticVerify::mapMergeConfigFromSystemParams()
{
    MapMergeConfig config;
    config.passage_match_tolerance_m = static_cast<double>(
        types::SystemParams::GetParams()->map_merge.passage_match_tolerance_m);
    config.wall_coplanar_angle_deg = static_cast<double>(
        types::SystemParams::GetParams()->map_merge.wall_coplanar_angle_deg);
    config.wall_edge_overlap_m = static_cast<double>(
        types::SystemParams::GetParams()->map_merge.wall_edge_overlap_m);
    config.floor_match_tolerance_m = static_cast<double>(
        types::SystemParams::GetParams()->map_merge.floor_match_tolerance_m);
    config.room_centroid_tolerance_m = static_cast<double>(
        types::SystemParams::GetParams()->map_merge.room_centroid_tolerance_m);
    return config;
}

SemanticMergeGateResult SemanticVerify::evaluateConsecutiveMergeGate(
    core::Map                  *p_survivingMap_in,
    core::Map                  *p_absorbedMap_in,
    const g2o::Sim3      &transform_absorbedToSurviving_in,
    const MapMergeConfig &config_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        return result;
    }

    if (!checkConsecutiveFloors(p_survivingMap_in,
                                p_absorbedMap_in,
                                transform_absorbedToSurviving_in,
                                config_in.floor_match_tolerance_m,
                                result.floorDecision))
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        return result;
    }
    result.floorDecision = "ACCEPTED";

    SemanticVerifyConfig verifyConfig;
    verifyConfig.maxNormalAngle_deg = config_in.wall_coplanar_angle_deg;

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    for (Room *p_room : p_survivingMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            survivingRooms.push_back(
                copyMergeRoomEvidence(p_room, verifyConfig));
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    for (Room *p_room : p_absorbedMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            absorbedRooms.push_back(
                copyMergeRoomEvidence(p_room, verifyConfig));
        }
    }

    const std::vector<ConsecutiveAnchorPair> anchorPairs =
        collectConsecutiveAnchors(survivingRooms, absorbedRooms);
    result.sharedRoomCount = anchorPairs.size();
    if (anchorPairs.empty())
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        return result;
    }

    /* Room-prior seed: the old final room and the new starting room must
     * be the same tag-matched anchor. */
    Room *p_oldFinalRoom = p_absorbedMap_in->getFinalRoom();
    Room *p_newStartRoom = p_survivingMap_in->getStartingRoom();
    bool  seedAnchored   = false;
    if (p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
        p_oldFinalRoom->hasRoomTag() && p_newStartRoom->hasRoomTag() &&
        !p_oldFinalRoom->getRoomTag().empty() &&
        p_oldFinalRoom->getRoomTag() == p_newStartRoom->getRoomTag())
    {
        for (const ConsecutiveAnchorPair &pair : anchorPairs)
        {
            if (pair.p_surviving->context.roomTag ==
                p_newStartRoom->getRoomTag())
            {
                seedAnchored = true;
                break;
            }
        }
    }
    if (!seedAnchored)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        return result;
    }

    const AlignmentCheck centroidCheck =
        checkAnchorRoomCentroids(anchorPairs,
                                 transform_absorbedToSurviving_in,
                                 config_in.passage_match_tolerance_m);
    if (centroidCheck == AlignmentCheck::CONTRADICTION)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
        return result;
    }
    if (centroidCheck == AlignmentCheck::MISSING)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = SemanticMergeReason::WALL_EVIDENCE_MISSING;
        return result;
    }

    bool                hasMissingEvidence = false;
    std::size_t         matchedPassages    = 0U;
    SemanticMergeReason topologyReason =
        SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
    const AlignmentCheck topologyCheck =
        checkConsecutivePassageTopology(survivingRooms,
                                        absorbedRooms,
                                        transform_absorbedToSurviving_in,
                                        config_in.passage_match_tolerance_m,
                                        matchedPassages,
                                        topologyReason);
    result.matchedPassageCount = matchedPassages;
    if (topologyCheck == AlignmentCheck::CONTRADICTION)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = topologyReason;
        return result;
    }
    if (topologyCheck == AlignmentCheck::MISSING)
    {
        hasMissingEvidence = true;
        result.reason      = SemanticMergeReason::PASSAGE_EVIDENCE_MISSING;
    }

    std::size_t alignedRoomCount = 0U;
    for (const ConsecutiveAnchorPair &pair : anchorPairs)
    {
        std::size_t          pairMatchedWalls = 0U;
        const AlignmentCheck wallCheck =
            checkFixedTransformWalls(pair.p_surviving->walls,
                                     pair.p_absorbed->walls,
                                     transform_absorbedToSurviving_in,
                                     verifyConfig,
                                     pairMatchedWalls);
        result.matchedWallCount += pairMatchedWalls;
        if (wallCheck == AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            return result;
        }
        if (wallCheck == AlignmentCheck::MISSING)
        {
            hasMissingEvidence = true;
            result.reason      = SemanticMergeReason::WALL_EVIDENCE_MISSING;
            continue;
        }
        if (checkConsecutiveWallEdgeOverlap({pair},
                                            transform_absorbedToSurviving_in,
                                            config_in.wall_coplanar_angle_deg,
                                            config_in.wall_edge_overlap_m) ==
            AlignmentCheck::CONTRADICTION)
        {
            result.decision = SemanticMergeDecision::REJECT;
            result.reason   = SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION;
            return result;
        }
        ++alignedRoomCount;
    }

    if (hasMissingEvidence || alignedRoomCount != anchorPairs.size())
    {
        result.decision = SemanticMergeDecision::DEFER;
        return result;
    }
    result.decision = SemanticMergeDecision::ACCEPT;
    result.reason   = SemanticMergeReason::ALIGNED;
    return result;
}

const char *
    SemanticVerify::mergeDecisionName(const SemanticMergeDecision decision_in)
{
    switch (decision_in)
    {
    case SemanticMergeDecision::ACCEPT:
        return "ACCEPT";
    case SemanticMergeDecision::DEFER:
        return "DEFER";
    case SemanticMergeDecision::REJECT:
        return "REJECT";
    }
    return "UNKNOWN";
}

const char *SemanticVerify::mergeReasonName(const SemanticMergeReason reason_in)
{
    switch (reason_in)
    {
    case SemanticMergeReason::ALIGNED:
        return "ALIGNED";
    case SemanticMergeReason::INVALID_INPUT:
        return "INVALID_INPUT";
    case SemanticMergeReason::FLOOR_EVIDENCE_MISSING:
        return "FLOOR_EVIDENCE_MISSING";
    case SemanticMergeReason::FLOOR_CONTRADICTION:
        return "FLOOR_CONTRADICTION";
    case SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING:
        return "SHARED_ROOM_IDENTITY_MISSING";
    case SemanticMergeReason::WALL_EVIDENCE_MISSING:
        return "WALL_EVIDENCE_MISSING";
    case SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION:
        return "WALL_ALIGNMENT_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_EVIDENCE_MISSING:
        return "PASSAGE_EVIDENCE_MISSING";
    case SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION:
        return "PASSAGE_IDENTITY_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION:
        return "PASSAGE_ENDPOINT_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION:
        return "PASSAGE_DIRECTION_CONTRADICTION";
    }
    return "UNKNOWN";
}

VerificationVerdict SemanticVerifyResult::toVerificationVerdict() const
{
    VerificationVerdict verdict;
    verdict.status      = status;
    verdict.pass        = pass && floorGatePassed;
    verdict.inlierCount = static_cast<unsigned int>(inliers.size());
    verdict.inlierRatio = inlierRatio;
    verdict.normalisedConditionNumber = normalisedConditionNumber;
    verdict.angularResidual_rad       = angularResidual_rad;
    verdict.confidence                = confidence;
    return verdict;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
