/*!
 * @file            mergeReasonName.cc
 *
 * @brief           Implements SemanticVerify::mergeReasonName().
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"

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

SemanticVerifyStatus
    SemanticVerify::mergeReasonName(const SemanticMergeReason reason_in,
                                    const char              *&p_name_out)
{
    switch (reason_in)
    {
    case SemanticMergeReason::ALIGNED:
    {
        p_name_out = "ALIGNED";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::INVALID_INPUT:
    {
        p_name_out = "INVALID_INPUT";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::FLOOR_EVIDENCE_MISSING:
    {
        p_name_out = "FLOOR_EVIDENCE_MISSING";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::FLOOR_CONTRADICTION:
    {
        p_name_out = "FLOOR_CONTRADICTION";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING:
    {
        p_name_out = "SHARED_ROOM_IDENTITY_MISSING";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::WALL_EVIDENCE_MISSING:
    {
        p_name_out = "WALL_EVIDENCE_MISSING";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION:
    {
        p_name_out = "WALL_ALIGNMENT_CONTRADICTION";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::PASSAGE_EVIDENCE_MISSING:
    {
        p_name_out = "PASSAGE_EVIDENCE_MISSING";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION:
    {
        p_name_out = "PASSAGE_IDENTITY_CONTRADICTION";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION:
    {
        p_name_out = "PASSAGE_ENDPOINT_CONTRADICTION";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION:
    {
        p_name_out = "PASSAGE_DIRECTION_CONTRADICTION";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    }
    p_name_out = "UNKNOWN";
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
