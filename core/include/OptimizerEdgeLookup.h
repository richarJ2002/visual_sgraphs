/*!
 * @file OptimizerEdgeLookup.h
 * @brief Bounds-checked lookup into a bundle-adjustment edge's parallel
 *        keyframe vector.
 */

#ifndef VS_GRAPHS_CORE_OPTIMIZER_EDGE_LOOKUP_H
#define VS_GRAPHS_CORE_OPTIMIZER_EDGE_LOOKUP_H

#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * Returns the pointer at index_in in edgeKeyFrames_in, or nullptr when
 * index_in is out of bounds.
 *
 * Bundle-adjustment edge bookkeeping keeps one KeyFrame* per edge in a
 * vector parallel to the edge's own vector (e.g. vpEdgesMono/vpEdgeKFMono,
 * vpEdgesStereo/vpEdgeKFStereo). Naming the correct parallel vector at each
 * call site is easy to get wrong by copy-paste -- found swapped at three
 * call sites across Optimizer::BundleAdjustment and
 * Optimizer::LoopClosureLocalBundleAdjustment, silently comparing the wrong
 * keyframe and, whenever the two vectors differ in length, reading out of
 * bounds. This helper makes the intended vector an explicit argument at
 * each call site and removes the out-of-bounds risk.
 */
template <typename PointerT>
inline PointerT
    edgeSourceKeyFrame(const std::vector<PointerT> &edgeKeyFrames_in,
                       std::size_t                  index_in)
{
    return index_in < edgeKeyFrames_in.size() ? edgeKeyFrames_in[index_in]
                                              : nullptr;
}

} // namespace core
} // namespace vs_graphs

#endif
