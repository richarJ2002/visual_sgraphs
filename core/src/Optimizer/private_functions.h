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
#include <utility>

#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"

namespace vs_graphs
{
namespace core
{

class MapPoint;

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

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_OPTIMIZER_PRIVATE_FUNCTIONS_H */
