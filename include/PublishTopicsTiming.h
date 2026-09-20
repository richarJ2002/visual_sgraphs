/*!
 * @file PublishTopicsTiming.h
 * @brief Declares the non-owning publication timing diagnostic sink.
 */

#ifndef VS_GRAPHS_PUBLISH_TOPICS_TIMING_H
#define VS_GRAPHS_PUBLISH_TOPICS_TIMING_H

#include <chrono>

namespace vs_graphs::observability
{

enum class PublishTopic
{
    ALL_MAPPED_WALLS,
    SEGMENTED_CLOUD,
    PLANES,
    ALL_POINTS,
    TRACKED_POINTS,
    FREE_SPACE_CLUSTERS
};

using PublishTopicTimingCallback =
    void (*)(void *,
             PublishTopic,
             bool,
             std::chrono::steady_clock::time_point,
             std::chrono::steady_clock::time_point);

struct PublishTopicsTimingSink
{
    void                      *p_context{nullptr};
    PublishTopicTimingCallback callback{nullptr};
};

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_PUBLISH_TOPICS_TIMING_H */
