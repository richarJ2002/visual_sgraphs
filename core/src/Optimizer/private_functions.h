/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  Optimizer translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of Optimizer.cc; external linkage
 *                  here is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_OPTIMIZER_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_OPTIMIZER_PRIVATE_FUNCTIONS_H

#include <atomic>
#include <cstddef>
#include <utility>
#include <vector>

#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"

namespace g2o
{
class SparseOptimizer;
} // namespace g2o

namespace vs_graphs
{
namespace core
{

class EdgeMonoOnlyPose;
class EdgeStereoOnlyPose;
class Frame;
class MapPoint;
class VertexPose;

/*! Bridges a thread-safe cancellation request into g2o's thread-local flag. */
class AtomicOptimizerStopBridge final : public g2o::HyperGraphAction
{
  public:
    AtomicOptimizerStopBridge(const std::atomic_bool *p_stopRequested_in,
                              bool                   *p_localStopFlag_inout) :
        p_stopRequested(p_stopRequested_in),
        p_localStopFlag(p_localStopFlag_inout)
    {}

    g2o::HyperGraphAction *
        operator()(const g2o::HyperGraph *p_graph_in,
                   Parameters            *p_parameters_inout = nullptr) override
    {
        (void)p_graph_in;
        (void)p_parameters_inout;

        if (p_stopRequested != nullptr && p_localStopFlag != nullptr &&
            p_stopRequested->load(std::memory_order_acquire))
        {
            *p_localStopFlag = true;
        }

        return this;
    }

  private:
    const std::atomic_bool *p_stopRequested;
    bool                   *p_localStopFlag;
};

bool sortByVal(const std::pair<MapPoint *, int> &firstEntry_in,
               const std::pair<MapPoint *, int> &secondEntry_in);

/*!
 * @brief        Adds a pose-only reprojection edge, with a Huber kernel, for
 *               every map point matched in the frame (left monocular, stereo
 *               and right monocular observations) and marks those keypoints
 *               as inliers. Holds MapPoint::globalMutex while reading the
 *               map points.
 *
 * @param[in,out] p_frame_inout
 *                Frame whose matched keypoints become edges; shall be
 *                non-null. Its outlier flags are cleared for those keypoints.
 * @param[in]    p_poseVertex_in
 *               Pose vertex of the frame; the optimizer owns it.
 * @param[in,out] optimizer_inout
 *                Optimizer that takes ownership of the new edges.
 * @param[in,out] edgesMonos_inout
 *                Monocular edges, appended in keypoint order (borrowed).
 * @param[in,out] edgesStereos_inout
 *                Stereo edges, appended in keypoint order (borrowed).
 * @param[in,out] monoEdgeIndices_inout
 *                Keypoint index of each monocular edge.
 * @param[in,out] stereoEdgeIndices_inout
 *                Keypoint index of each stereo edge.
 * @param[in,out] initialMonoCorrespondenceCount_inout
 *                Incremented once per monocular edge.
 * @param[in,out] initialStereoCorrespondenceCount_inout
 *                Incremented once per stereo edge.
 */
void addPoseOnlyObservationEdges(
    Frame                             *p_frame_inout,
    VertexPose                        *p_poseVertex_in,
    g2o::SparseOptimizer              &optimizer_inout,
    std::vector<EdgeMonoOnlyPose *>   &edgesMonos_inout,
    std::vector<EdgeStereoOnlyPose *> &edgesStereos_inout,
    std::vector<size_t>               &monoEdgeIndices_inout,
    std::vector<size_t>               &stereoEdgeIndices_inout,
    int                               &initialMonoCorrespondenceCount_inout,
    int                               &initialStereoCorrespondenceCount_inout);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_OPTIMIZER_PRIVATE_FUNCTIONS_H */
