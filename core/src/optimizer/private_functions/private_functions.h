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

#include "optimizer/objects/OptimizerEdgeLookupStatusEnum.h"
#include "optimizer/optimizer.h"
#include <vs_graphs_g2o/core/hyper_graph_action.h>

namespace vs_graphs::core::frame { class Frame; }
namespace vs_graphs::core::g2o_types::edge_mono_only_pose { class EdgeMonoOnlyPose; }
namespace vs_graphs::core::g2o_types::edge_stereo_only_pose { class EdgeStereoOnlyPose; }
namespace vs_graphs::core::g2o_types::vertex_pose { class VertexPose; }
namespace vs_graphs::core::map_point { class MapPoint; }

namespace g2o
{
class SparseOptimizer;
} /* namespace g2o */

namespace vs_graphs
{
namespace core
{







/*!
 * @brief           Bridges a thread-safe cancellation request into g2o's
 *                  thread-local flag.
 */
class AtomicOptimizerStopBridge final : public g2o::HyperGraphAction
{
  public:
    /*!
     * @brief           Binds the cancellation request to the g2o stop flag.
     *
     * @param[in]       p_stopRequested_in
     *                  Thread-safe cancellation request written by another
     *                  thread; may be null. Borrowed; shall outlive the bridge.
     *
     * @param[in,out]   p_localStopFlag_inout
     *                  g2o's force-stop flag, set to true when a stop was
     *                  requested; may be null. Borrowed; shall outlive the
     *                  bridge.
     */
    AtomicOptimizerStopBridge(const std::atomic_bool *p_stopRequested_in,
                              bool                   *p_localStopFlag_inout) :
        p_stopRequested(p_stopRequested_in),
        p_localStopFlag(p_localStopFlag_inout)
    {}

    /*!
     * @brief           Copies a pending stop request into the g2o stop flag;
     *                  g2o calls it before and after each iteration.
     *
     * @param[in]       p_graph_in
     *                  Graph being optimised; not used.
     *
     * @param[in,out]   p_parameters_inout
     *                  Action parameters; not used.
     *
     * @return          This action, always.
     */
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
    /*!
     * @brief           Cancellation request set from another thread; may be
     *                  null. Borrowed.
     */
    const std::atomic_bool *p_stopRequested;

    /*!
     * @brief           g2o's force-stop flag that this bridge sets; may be
     *                  null. Borrowed.
     */
    bool *p_localStopFlag;
};

/*!
 * @brief           Orders map point entries by their integer value, ascending.
 *
 * @param[in]       firstEntry_in
 *                  Map point and its value.
 *
 * @param[in]       secondEntry_in
 *                  Map point and its value.
 *
 * @return          True when the first entry's value is smaller than the
 *                  second's.
 */
bool sortByVal(const std::pair<map_point::MapPoint *, int> &firstEntry_in,
               const std::pair<map_point::MapPoint *, int> &secondEntry_in);

/*!
 * @brief           Adds a pose-only reprojection edge, with a Huber kernel, for
 *                  every map point matched in the frame (left monocular, stereo
 *                  and right monocular observations) and marks those keypoints
 *                  as inliers. Holds MapPoint::globalMutex while reading the
 *                  map points.
 *
 * @param[in,out]   p_frame_inout
 *                  Frame whose matched keypoints become edges; shall be
 *                  non-null. Its outlier flags are cleared for those keypoints.
 *
 * @param[in]       p_poseVertex_in
 *                  Pose vertex of the frame; the optimizer owns it.
 *
 * @param[in,out]   optimizer_inout
 *                  Optimizer that takes ownership of the new edges.
 *
 * @param[in,out]   edgesMonos_inout
 *                  Monocular edges, appended in keypoint order (borrowed).
 *
 * @param[in,out]   edgesStereos_inout
 *                  Stereo edges, appended in keypoint order (borrowed).
 *
 * @param[in,out]   monoEdgeIndices_inout
 *                  Keypoint index of each monocular edge.
 *
 * @param[in,out]   stereoEdgeIndices_inout
 *                  Keypoint index of each stereo edge.
 *
 * @param[in,out]   initialMonoCorrespondenceCount_inout
 *                  Incremented once per monocular edge.
 *
 * @param[in,out]   initialStereoCorrespondenceCount_inout
 *                  Incremented once per stereo edge.
 */
/*!
 * @brief           Sets p_keyFrame_out to the pointer at index_in in
 *                  edgeKeyFrames_in, or to nullptr when index_in is out of
 *                  bounds; always succeeds.
 *
 * Bundle-adjustment edge bookkeeping keeps one KeyFrame* per edge in a
 * vector parallel to the edge's own vector. Naming the correct parallel
 * vector at each call site is easy to get wrong by copy-paste, so this
 * helper makes the intended vector an explicit argument at each call
 * site and removes the out-of-bounds risk.
 *
 * @param[in]       edgeKeyFrames_in
 *                  Parallel keyframe vector for the edge vector being
 *                  processed; borrowed, only read.
 *
 * @param[in]       index_in
 *                  Edge index into edgeKeyFrames_in; may be out of bounds.
 *
 * @param[out]      p_keyFrame_out
 *                  Pointer at index_in, or nullptr when index_in is out of
 *                  bounds; always written.
 *
 * @return          OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS always.
 */
template <typename PointerT>
inline optimizer::OptimizerEdgeLookupStatus edgeSourceKeyFrame(
    const std::vector<PointerT> &edgeKeyFrames_in,
    std::size_t                  index_in,
    PointerT                    &p_keyFrame_out)
{
    p_keyFrame_out = index_in < edgeKeyFrames_in.size()
                         ? edgeKeyFrames_in[index_in]
                         : nullptr;
    return optimizer::OptimizerEdgeLookupStatus::
        OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS;
}

/*!
 * @brief           Adds a pose-only reprojection edge, with a Huber kernel, for
 *                  every map point matched in the frame (left monocular, stereo
 *                  and right monocular observations) and marks those keypoints
 *                  as inliers. Holds MapPoint::globalMutex while reading the
 *                  map points.
 *
 * @param[in,out]   p_frame_inout
 *                  Frame whose matched keypoints become edges; shall be
 *                  non-null. Its outlier flags are cleared for those keypoints.
 *
 * @param[in]       p_poseVertex_in
 *                  Pose vertex of the frame; the optimizer owns it.
 *
 * @param[in,out]   optimizer_inout
 *                  Optimizer that takes ownership of the new edges.
 *
 * @param[in,out]   edgesMonos_inout
 *                  Monocular edges, appended in keypoint order (borrowed).
 *
 * @param[in,out]   edgesStereos_inout
 *                  Stereo edges, appended in keypoint order (borrowed).
 *
 * @param[in,out]   monoEdgeIndices_inout
 *                  Keypoint index of each monocular edge.
 *
 * @param[in,out]   stereoEdgeIndices_inout
 *                  Keypoint index of each stereo edge.
 *
 * @param[in,out]   initialMonoCorrespondenceCount_inout
 *                  Incremented once per monocular edge.
 *
 * @param[in,out]   initialStereoCorrespondenceCount_inout
 *                  Incremented once per stereo edge.
 *
 * @return          OPTIMIZER_STATUS_SUCCESS always.
 */
[[nodiscard]] optimizer::OptimizerStatus addPoseOnlyObservationEdges(
    frame::Frame                             *p_frame_inout,
    g2o_types::vertex_pose::VertexPose                        *p_poseVertex_in,
    g2o::SparseOptimizer              &optimizer_inout,
    std::vector<g2o_types::edge_mono_only_pose::EdgeMonoOnlyPose *>   &edgesMonos_inout,
    std::vector<g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPose *> &edgesStereos_inout,
    std::vector<size_t>               &monoEdgeIndices_inout,
    std::vector<size_t>               &stereoEdgeIndices_inout,
    int                               &initialMonoCorrespondenceCount_inout,
    int                               &initialStereoCorrespondenceCount_inout);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_OPTIMIZER_PRIVATE_FUNCTIONS_H */
