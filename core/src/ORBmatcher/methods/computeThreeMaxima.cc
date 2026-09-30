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

/*!
 * @file            computeThreeMaxima.cc
 *
 * @brief           Implements ORBmatcher::computeThreeMaxima(), declared in
 *                  ORBmatcher.h.
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <cstdint>

namespace vs_graphs
{
namespace core
{

ORBmatcherStatus
    ORBmatcher::computeThreeMaxima(std::vector<int> *p_histogram_in,
                                   const int         L,
                                   int              &maximum1_inout,
                                   int              &maximum2_inout,
                                   int              &maximum3_out)
{
    int maximum1 = 0;
    int maximum2 = 0;
    int maximum3 = 0;

    for (int histogramBinIndex = 0; histogramBinIndex < L; histogramBinIndex++)
    {
        const int s = p_histogram_in[histogramBinIndex].size();
        if (s > maximum1)
        {
            maximum3       = maximum2;
            maximum2       = maximum1;
            maximum1       = s;
            maximum3_out   = maximum2_inout;
            maximum2_inout = maximum1_inout;
            maximum1_inout = histogramBinIndex;
        }
        else if (s > maximum2)
        {
            maximum3       = maximum2;
            maximum2       = s;
            maximum3_out   = maximum2_inout;
            maximum2_inout = histogramBinIndex;
        }
        else if (s > maximum3)
        {
            maximum3     = s;
            maximum3_out = histogramBinIndex;
        }
    }

    if (maximum2 < 0.1f * static_cast<float>(maximum1))
    {
        maximum2_inout = -1;
        maximum3_out   = -1;
    }
    else if (maximum3 < 0.1f * static_cast<float>(maximum1))
    {
        maximum3_out = -1;
    }

    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
