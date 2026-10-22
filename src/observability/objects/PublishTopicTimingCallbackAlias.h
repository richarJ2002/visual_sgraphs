/*!
 * @file            PublishTopicTimingCallbackAlias.h
 *
 * @brief           Declares the non-owning publication timing callback.
 */

#ifndef VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_TIMING_CALLBACK_ALIAS_H
#define VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_TIMING_CALLBACK_ALIAS_H

/* Function Includes */
/* None */

/* Object Include */
#include "PublishTopicEnum.h"

/* Data include */
#include <chrono>

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

/*!
 * @brief           Receives the borrowed context, topic, execution flag and
 *                  steady-clock start/end of a publication attempt.
 *
 *                  The caller owns the context through synchronous invocation;
 *                  no exception may escape a callback boundary.
 */
using PublishTopicTimingCallback =
    void (*)(void *,
             PublishTopic,
             bool,
             std::chrono::steady_clock::time_point,
             std::chrono::steady_clock::time_point);

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_TIMING_CALLBACK_ALIAS_H */
