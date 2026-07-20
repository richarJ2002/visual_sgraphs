/*!
 * @File:         findNearestMarker.cpp
 *
 * @Brief:        Avoids adding duplicate markers to the buffer by checking the
 *                timestamp.
 *
 * @Date:         20/07/2026
 *
 */

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
#include "Semantic/Marker.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

std::pair<double, std::vector<ORB_SLAM3::Marker *>>
    findNearestMarker(double frameTimestamp_in)
{
    /* Init variable of the minimum time difference */
    double minTimeDifference = 100;

    /* Init a variable which will be used to find best match to marker */
    std::vector<ORB_SLAM3::Marker *> matchedMarkers;

    /* Loop through the markersBuffer */
    for (const auto &markers : markersBuffer)
    {
        /* Find the time difference */
        double timeDifference = markers[0]->getTime() - frameTimestamp_in;

        /* If better match found, update */
        if (timeDifference < minTimeDifference)
        {
            matchedMarkers    = markers;
            minTimeDifference = timeDifference;
        }
    }

    /* Return the minimum time difference and best matched marker */
    return std::make_pair(minTimeDifference, matchedMarkers);
}
