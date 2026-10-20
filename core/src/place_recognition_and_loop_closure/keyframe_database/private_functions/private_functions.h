/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  KeyFrameDatabase translation units.
 */

#ifndef VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H

#include "place_recognition_and_loop_closure/keyframe_database/keyframe_database.h"

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

/*!
 * @brief           Orders accumulated-score pairs by descending score.
 *
 * @param[in]       firstScoredCandidate_in
 *                  First pair.
 *
 * @param[in]       secondScoredCandidate_in
 *                  Second pair.
 *
 * @return          True when the first score exceeds the second.
 */
bool compFirst(const std::pair<float, keyframe::KeyFrame *> &firstScoredCandidate_in,
               const std::pair<float, keyframe::KeyFrame *> &secondScoredCandidate_in);

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H \
        */
