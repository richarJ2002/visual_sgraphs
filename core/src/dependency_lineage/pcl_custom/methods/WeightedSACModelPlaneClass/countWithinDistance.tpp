/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * Derived from the Point Cloud Library (PCL) 1.14 file
 * pcl/sample_consensus/impl/sac_model_plane.hpp
 * (SampleConsensusModelPlane::countWithinDistanceStandard); its licence
 * notice follows.
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file            countWithinDistance.tpp
 *
 * @brief           Implements WeightedSACModelPlane::countWithinDistance(),
 *                  declared in pcl_custom/objects/WeightedSACModelPlaneClass.h.
 */

namespace vs_graphs::core::pcl_custom
{

template <typename PointType>
inline std::size_t WeightedSACModelPlane<PointType>::countWithinDistance(
    const Eigen::VectorXf &modelCoefficients_in,
    const double           distanceThreshold_in) const
{
    if (!this->isModelValid(modelCoefficients_in))
    {
        return (0);
    }

    /* Non-PointXYZRGBA input: run the base class implementation. */
    if (!std::is_same<PointType, pcl::PointXYZRGBA>::value)
    {
        return pcl::SampleConsensusModelPlane<PointType>::countWithinDistance(
            modelCoefficients_in,
            distanceThreshold_in);
    }

    /*!
     * Iterate through the 3d points and calculate the distances from them
     * to the plane; each inlier adds its alpha / 255 to the weighted count.
     */
    std::size_t inlierCount         = 0;
    double      weightedInlierCount = 0.0;
    for (std::size_t indexPosition = 0; indexPosition < this->indices_->size();
         indexPosition++)
    {
        /*!
         * Calculate the distance from the point to the plane normal as the
         * dot product:
         * D = (P-A).N/|N|
         * with P the point, A a point on the plane and N the plane normal,
         * all in the frame and length unit of the input cloud. With the
         * coefficients (a, b, c, d) and the homogeneous point (x, y, z, 1)
         * the dot product is a * x + b * y + c * z + d, which equals D when
         * (a, b, c) has unit length.
         */
        Eigen::Vector4f homogeneousPoint(
            (*this->input_)[(*this->indices_)[indexPosition]].x,
            (*this->input_)[(*this->indices_)[indexPosition]].y,
            (*this->input_)[(*this->indices_)[indexPosition]].z,
            1.0f);
        if (std::abs(modelCoefficients_in.dot(homogeneousPoint)) <
            distanceThreshold_in)
        {
            weightedInlierCount +=
                static_cast<int>(
                    (*this->input_)[(*this->indices_)[indexPosition]].a) /
                255.0;
        }
    }
    inlierCount = static_cast<std::size_t>(weightedInlierCount);
    return inlierCount;
}

} /* namespace vs_graphs::core::pcl_custom */
