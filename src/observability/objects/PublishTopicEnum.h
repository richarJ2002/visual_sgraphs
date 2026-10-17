/*!
 * @file            PublishTopicEnum.h
 *
 * @brief           Identifies a visualisation topic measured at publication.
 */

#ifndef VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_ENUM_H
#define VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_ENUM_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

/*!
 * @brief           Distinguishes the publication timing counters.
 */
enum class PublishTopic
{
    /*!
     * @brief           All mapped walls.
     */
    ALL_MAPPED_WALLS,
    /*!
     * @brief           Segmented point cloud.
     */
    SEGMENTED_CLOUD,
    /*!
     * @brief           Detected planes.
     */
    PLANES,
    /*!
     * @brief           All map points.
     */
    ALL_POINTS,
    /*!
     * @brief           Currently tracked points.
     */
    TRACKED_POINTS,
    /*!
     * @brief           Free-space clusters.
     */
    FREE_SPACE_CLUSTERS
};

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPIC_ENUM_H */
