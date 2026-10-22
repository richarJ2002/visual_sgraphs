/*!
 * @file            PublishTopicsTimingSinkStruct.h
 *
 * @brief           Bundles a publication timing callback and borrowed context.
 */

#ifndef VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPICS_TIMING_SINK_STRUCT_H
#define VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPICS_TIMING_SINK_STRUCT_H

/* Function Includes */
/* None */

/* Object Include */
#include "PublishTopicTimingCallbackAlias.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

/*!
 * @brief           Reports topic timings without owning the callback context.
 *
 *                  The context must outlive every synchronous callback. A null
 *                  callback disables reporting.
 */
struct PublishTopicsTimingSink
{
  public:
    /*!
     * @brief           Borrowed callback context; may be null.
     */
    void *p_context{nullptr};

    /*!
     * @brief           Function invoked on each publication timing report.
     */
    PublishTopicTimingCallback callback{nullptr};
};

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_OBSERVABILITY_PUBLISH_TOPICS_TIMING_SINK_STRUCT_H */
