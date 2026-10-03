/*!
 * @file            PublishTopicsTiming.h
 *
 * @brief           Declares the non-owning publication timing diagnostic sink.
 */

#ifndef VS_GRAPHS_PUBLISH_TOPICS_TIMING_H
#define VS_GRAPHS_PUBLISH_TOPICS_TIMING_H

#include <chrono>

namespace vs_graphs::observability
{

/*!
 * @brief           Identifies which visualisation topic a timing report refers
 *                  to.
 */
enum class PublishTopic
{
    ALL_MAPPED_WALLS,
    SEGMENTED_CLOUD,
    PLANES,
    ALL_POINTS,
    TRACKED_POINTS,
    FREE_SPACE_CLUSTERS
};

/*!
 * @brief           Function type that receives one timing report for a
 *                  published topic.
 *
 *                  The arguments are, in order: the sink's context pointer, the
 *                  topic, whether the publication actually ran (false when it
 *                  was skipped, for example by a rate limit), and the
 *                  steady-clock start and end of the attempt.
 */
using PublishTopicTimingCallback =
    void (*)(void *,
             PublishTopic,
             bool,
             std::chrono::steady_clock::time_point,
             std::chrono::steady_clock::time_point);

/*!
 * @brief           Pair of a callback and its context that publication code
 *                  reports topic timings to; neither member is owned.
 */
struct PublishTopicsTimingSink
{
    /*!
     * @brief           Borrowed pointer passed back to the callback; may be
     *                  null.
     */
    void *p_context{nullptr};

    /*!
     * @brief           Function called with each timing report; null disables
     *                  reporting.
     */
    PublishTopicTimingCallback callback{nullptr};
};

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_PUBLISH_TOPICS_TIMING_H */
