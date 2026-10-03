/*!
 * @file            verify.cc
 *
 * @brief           Implements SemanticVerify::verify(), declared in
 *                  Semantic/SemanticVerify.h.
 */

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
#include <rclcpp/logging.hpp>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus
    SemanticVerify::verify(const std::vector<VerifyWallObservation> &wallsA_in,
                           const std::vector<VerifyWallObservation> &wallsB_in,
                           SemanticVerifyResult                     &result_out,
                           const SemanticVerifyConfig &configuration_in)
{
    SemanticVerifyResult result;
    result.candidateWallPairCount = wallsA_in.size() * wallsB_in.size();

    /* Minimal sample: 3 planes for full SE(3). Fewer than
     * 3 walls on either side can never form a full-rank hypothesis. */
    if (wallsA_in.size() < 3U || wallsB_in.size() < 3U)
    {
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::TOO_FEW_WALLS;
        result_out          = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    /* Full cross-product candidate set. No pre-filtering by raw normal
     * similarity here: the two rooms are in different, as-yet-unrelated map
     * frames, so a candidate pair's normals cannot be compared directly
     * before a hypothesis rotation exists. Robustness to wrong candidate
     * pairs comes from the 3-subset hypothesis + inlier-count step below. */
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

    const double maximumNormalAngle_rad =
        configuration_in.maxNormalAngle_deg * M_PI / 180.0;

    /* All hypotheses passing the rank/condition-number gates. Deduplicated
     * by inlier-set signature after enumeration: distinct minimal (3-wall)
     * samples routinely rediscover the exact same correct solution on
     * well-conditioned, noise-free data, and that must not look like two
     * competing hypotheses to the ambiguity-rejection step below. */
    std::vector<Hypothesis> allHypotheses;
    std::size_t             hypothesesEvaluated = 0U;
    const std::size_t       pairCount           = candidatePairs.size();

    for (std::size_t firstIndex = 0U;
         firstIndex < pairCount &&
         hypothesesEvaluated < configuration_in.maxHypotheses;
         ++firstIndex)
    {
        for (std::size_t secondIndex = firstIndex + 1U;
             secondIndex < pairCount &&
             hypothesesEvaluated < configuration_in.maxHypotheses;
             ++secondIndex)
        {
            for (std::size_t thirdIndex = secondIndex + 1U;
                 thirdIndex < pairCount &&
                 hypothesesEvaluated < configuration_in.maxHypotheses;
                 ++thirdIndex)
            {
                const CandidatePair &pair0 = candidatePairs[firstIndex];
                const CandidatePair &pair1 = candidatePairs[secondIndex];
                const CandidatePair &pair2 = candidatePairs[thirdIndex];

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
                    wallsA_in[pair0.indexA].wallNormal_world,
                    wallsA_in[pair1.indexA].wallNormal_world,
                    wallsA_in[pair2.indexA].wallNormal_world};
                const std::vector<Eigen::Vector3d> normalsB = {
                    wallsB_in[pair0.indexB].wallNormal_world,
                    wallsB_in[pair1.indexB].wallNormal_world,
                    wallsB_in[pair2.indexB].wallNormal_world};

                RotationFit rotationFit{};
                if (fitRotationFromNormals(normalsA, normalsB, rotationFit) !=
                    SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: fitRotationFromNormals returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
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

                TranslationFit translationFit{};
                if (fitTranslation(rotationFit.rotation,
                                   rotatedNormalsA,
                                   offsetsA,
                                   normalsB,
                                   offsetsB,
                                   translationFit) !=
                    SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: fitTranslation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (!translationFit.valid || translationFit.rank < 3U)
                {
                    continue;
                }
                if (!std::isfinite(translationFit.conditionNumber) ||
                    translationFit.conditionNumber >
                        configuration_in.maxConditionNumber)
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
                        rotationFit.rotation * wallA.wallNormal_world;
                    const double predictedOffset =
                        wallA.d -
                        predictedNormal.dot(translationFit.translation);
                    double normalAngle_rad{};
                    if (angleBetween_rad(predictedNormal,
                                         wallB.wallNormal_world,
                                         normalAngle_rad) !=
                        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: angleBetween_rad returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const double offsetResidual_m =
                        std::abs(predictedOffset - wallB.d);

                    if (normalAngle_rad > maximumNormalAngle_rad)
                    {
                        continue;
                    }
                    if (offsetResidual_m > configuration_in.maxOffset_m)
                    {
                        continue;
                    }
                    /* Explicit |cos(theta)| gate, distinct
                     * from the angle gate above. */
                    if (std::abs(std::cos(normalAngle_rad)) <=
                        configuration_in.minAbsCosNormalAngle)
                    {
                        continue;
                    }
                    double supportDistance_m{};
                    if (symmetricSupportDistance(wallA,
                                                 wallB,
                                                 rotationFit.rotation,
                                                 translationFit.translation,
                                                 supportDistance_m) !=
                        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
                    {
                        // symmetricSupportDistance cannot fail; continue as
                        // before.
                    }
                    if (supportDistance_m > configuration_in.maxSupportDist_m)
                    {
                        continue;
                    }

                    passing.push_back({candidate.indexA,
                                       candidate.indexB,
                                       normalAngle_rad,
                                       offsetResidual_m,
                                       supportDistance_m,
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
        result_out          = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
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
        const std::vector<Hypothesis>::iterator existing =
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

    if (topInliers < runnerUpInliers + configuration_in.ambiguityMarginInliers)
    {
        /* Insufficient discrimination between the top two DISTINCT
         * hypotheses. */
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::AMBIGUOUS_TOP_HYPOTHESES;
        result_out          = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    if (inlierRatio < configuration_in.minInlierRatio)
    {
        result.status       = VerificationStatus::REJECTED;
        result.rejectReason = VerifyRejectReason::BELOW_MIN_INLIER_RATIO;
        result_out          = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    const Hypothesis &seed = best[0];

    /* Nonlinear refinement: one EdgePlaneTransformSE3 unary
     * factor per accepted inlier wall pair, Huber-robustified. */
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *p_solver = new g2o::BlockSolverX(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_algorithm =
        new g2o::OptimizationAlgorithmLevenberg(p_solver);
    optimizer.setAlgorithm(p_algorithm);
    optimizer.setVerbose(false);

    g2o::VertexSE3Expmap *p_vertex = new g2o::VertexSE3Expmap();
    p_vertex->setEstimate(g2o::SE3Quat(seed.rotation, seed.translation));
    p_vertex->setId(0);
    p_vertex->setFixed(false);
    optimizer.addVertex(p_vertex);

    const double omegaTheta = 1.0 / (configuration_in.sigmaTheta_rad *
                                     configuration_in.sigmaTheta_rad);
    const double omegaOffset =
        1.0 / (configuration_in.sigmaOffset_m * configuration_in.sigmaOffset_m);
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    information(0, 0)           = omegaTheta;
    information(1, 1)           = omegaTheta;
    information(2, 2)           = omegaOffset;

    for (const WallInlierPair &inlier : seed.inliers)
    {
        const VerifyWallObservation *p_observationA = nullptr;
        if (findByWallId(wallsA_in, inlier.wallIdA, p_observationA) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findByWallId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const VerifyWallObservation *p_observationB = nullptr;
        if (findByWallId(wallsB_in, inlier.wallIdB, p_observationB) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findByWallId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_observationA == nullptr || p_observationB == nullptr)
        {
            continue;
        }

        PlanePairMeasurement measurement;
        measurement.n_A   = p_observationA->wallNormal_world;
        measurement.d_A   = p_observationA->d;
        measurement.n_B   = p_observationB->wallNormal_world;
        measurement.d_B   = p_observationB->d;
        measurement.sigma = 1;

        EdgePlaneTransformSE3 *p_edge = new EdgePlaneTransformSE3();
        p_edge->setVertex(0, p_vertex);
        p_edge->setMeasurement(measurement);
        p_edge->setInformation(information);
        g2o::RobustKernelHuber *p_kernel = new g2o::RobustKernelHuber();
        p_kernel->setDelta(configuration_in.huberDelta);
        p_edge->setRobustKernel(p_kernel);
        optimizer.addEdge(p_edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(static_cast<int>(configuration_in.optimizerIterations));

    const g2o::SE3Quat    refinedEstimate = p_vertex->estimate();
    const Eigen::Matrix3d refinedRotation =
        refinedEstimate.rotation().toRotationMatrix();
    const Eigen::Vector3d refinedTranslation = refinedEstimate.translation();

    /* Observability re-check on the refined transform: full
     * translational rank via N_B over the inlier set (as above), full
     * rotational rank via >=2 nonparallel inlier normal directions in the
     * surviving (B) frame (the stated equivalence). A complete
     * 6x6 Hessian SVD inspection is not extracted from g2o's internal
     * solver state here -- this is a documented simplification, not a
     * silent one. */
    std::vector<Eigen::Vector3d> inlierRotatedNormalsA;
    std::vector<double>          inlierOffsetsA;
    std::vector<Eigen::Vector3d> inlierNormalsB;
    std::vector<double>          inlierOffsetsB;
    std::vector<double>          angularResiduals;
    for (const WallInlierPair &inlier : seed.inliers)
    {
        const VerifyWallObservation *p_observationA = nullptr;
        if (findByWallId(wallsA_in, inlier.wallIdA, p_observationA) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findByWallId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const VerifyWallObservation *p_observationB = nullptr;
        if (findByWallId(wallsB_in, inlier.wallIdB, p_observationB) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findByWallId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_observationA == nullptr || p_observationB == nullptr)
        {
            continue;
        }
        inlierRotatedNormalsA.push_back(refinedRotation *
                                        p_observationA->wallNormal_world);
        inlierOffsetsA.push_back(p_observationA->d);
        inlierNormalsB.push_back(p_observationB->wallNormal_world);
        inlierOffsetsB.push_back(p_observationB->d);
        angularResiduals.push_back(inlier.normalAngleResidual_rad);
    }

    TranslationFit refinedFit{};
    if (fitTranslation(refinedRotation,
                       inlierRotatedNormalsA,
                       inlierOffsetsA,
                       inlierNormalsB,
                       inlierOffsetsB,
                       refinedFit) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: fitTranslation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
        result_out             = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::sort(angularResiduals.begin(), angularResiduals.end());
    const double medianAngularResidual_rad =
        angularResiduals.empty()
            ? 0.0
            : angularResiduals[angularResiduals.size() / 2U];

    result.status                              = VerificationStatus::PASS;
    result.hasPassed                           = true;
    result.roomTransform_roomAToRoomB          = Eigen::Isometry3d::Identity();
    result.roomTransform_roomAToRoomB.linear() = refinedRotation;
    result.roomTransform_roomAToRoomB.translation() = refinedTranslation;
    result.inliers                                  = seed.inliers;
    result.rank                                     = refinedFit.rank;
    result.conditionNumber = refinedFit.conditionNumber;
    result.normalisedConditionNumber =
        std::isfinite(refinedFit.conditionNumber)
            ? std::min(1.0,
                       refinedFit.conditionNumber /
                           configuration_in.maxConditionNumber)
            : 1.0;
    result.inlierRatio         = inlierRatio;
    result.angularResidual_rad = medianAngularResidual_rad;
    result.confidence =
        std::clamp(inlierRatio * (1.0 - 0.5 * result.normalisedConditionNumber),
                   0.0,
                   1.0);

    result_out = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
