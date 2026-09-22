/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  KeyFrameDatabase translation units.
 */

#ifndef VS_GRAPHS_CORE_KEYFRAMEDATABASE_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_KEYFRAMEDATABASE_PRIVATE_FUNCTIONS_H

#include "KeyFrameDatabase.h"

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Orders accumulated-score pairs by descending score.
 *
 * @param[in]       a
 *                  First pair.
 * @param[in]       b
 *                  Second pair.
 *
 * @return          True when the first score exceeds the second.
 */
bool compFirst(const std::pair<float, KeyFrame *> &a,
               const std::pair<float, KeyFrame *> &b);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_KEYFRAMEDATABASE_PRIVATE_FUNCTIONS_H */
