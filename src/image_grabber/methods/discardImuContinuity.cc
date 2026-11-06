/*!
 * @file            discardImuContinuity.cc
 *
 * @brief           Forgets the IMU continuity state of the synchronisation
 *                  thread.
 *
 * @date            05/11/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

#include "image_grabber/objects/ImageGrabberClass.h"

namespace vs_graphs::image_grabber
{

void ImageGrabber::discardImuContinuity()
{
    pendingImuMeasurements.clear();
    pendingMaximumImuGap_seconds = 0.0;
    hasConsumedImuSample         = false;
}
} /* namespace vs_graphs::image_grabber */
