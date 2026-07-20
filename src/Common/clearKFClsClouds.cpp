/*!
 * @File:         clearKFClsClouds.cpp
 *
 * @Brief:        Function which clears the cluster points from the keyframe
 *                vectors.
 *
 * @Date:         20/07/2026
 *
 */

#include <vector>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
#include "KeyFrame.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *> keyframeVector_in)
{
    /* Iterate through keyframes and clear the cls point clouds */
    for (auto &keyframe : keyframeVector_in)
    {
        keyframe->clearClsClouds();
    }
}
