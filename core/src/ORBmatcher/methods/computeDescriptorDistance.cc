/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <stdint-gcc.h>

namespace vs_graphs
{
namespace core
{

ORBmatcherStatus
    ORBmatcher::computeDescriptorDistance(const cv::Mat &a,
                                          const cv::Mat &b,
                                          int           &descriptorDistance_out)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int distance = 0;

    for (int wordIndex = 0; wordIndex < 8; wordIndex++, pa++, pb++)
    {
        unsigned int v = *pa ^ *pb;
        v              = v - ((v >> 1) & 0x55555555);
        v              = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        distance += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    descriptorDistance_out = distance;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
