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
 * pcl/sample_consensus/sac_model_plane.h
 * (class SampleConsensusModelPlane); its licence notice follows.
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * @file            WeightedSACModelPlaneClass.h
 *
 * @brief           Declares the plane model whose inlier count weighs each
 *                  point by its alpha channel.
 */

#ifndef VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_MODEL_PLANE_CLASS_H
#define VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_MODEL_PLANE_CLASS_H

#include <cmath>
#include <cstddef>
#include <type_traits>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/types.h>

namespace vs_graphs::core::pcl_custom
{

/*!
 * @brief           PCL plane model whose inlier count weighs each inlier by its
 *                  alpha channel.
 *
 *                  For pcl::PointXYZRGBA clouds, countWithinDistance() adds
 *                  alpha / 255 for every point closer to the plane than the
 *                  threshold, so the SAC estimators rank candidate planes by
 *                  their alpha-weighted support. Every other point type keeps
 *                  the unweighted count of pcl::SampleConsensusModelPlane.
 *
 * @tparam          PointType
 *                  PCL point type of the input cloud; pcl::PointXYZRGBA
 *                  enables the weighting.
 */
template <typename PointType>
class WeightedSACModelPlane : public pcl::SampleConsensusModelPlane<PointType>
{
  public:
    /*!
     * @brief           Shared pointer to a weighted plane model.
     *
     * @note            boost::shared_ptr as upstream; it hides the
     *                  std::shared_ptr Ptr of the PCL base. PCL alias name,
     *                  kept.
     */
    using Ptr = boost::shared_ptr<WeightedSACModelPlane<PointType>>;

    /*!
     * @brief           Shared pointer to a read-only weighted plane model.
     *
     * @note            boost::shared_ptr as upstream; it hides the
     *                  std::shared_ptr ConstPtr of the PCL base. PCL alias
     *                  name, kept.
     */
    using ConstPtr = boost::shared_ptr<const WeightedSACModelPlane<PointType>>;

    /*!
     * @brief           Creates the model over the given points of a cloud.
     *
     * @param[in]       p_inputCloud_in
     *                  Input cloud; the model shares its ownership.
     *
     * @param[in]       pointIndices_in
     *                  Indices of the cloud points the model evaluates; the
     *                  model keeps a copy.
     *
     * @param[in]       useRandomSeed_in
     *                  True seeds PCL's sample generator from
     *                  std::random_device; false uses PCL's fixed seed.
     */
    WeightedSACModelPlane(const typename pcl::SampleConsensusModelPlane<
                              PointType>::PointCloudConstPtr &p_inputCloud_in,
                          const pcl::Indices                 &pointIndices_in,
                          bool useRandomSeed_in = false) :
        pcl::SampleConsensusModelPlane<PointType>::SampleConsensusModelPlane(
            p_inputCloud_in,
            pointIndices_in,
            useRandomSeed_in)
    {}

    /*!
     * @brief           Counts the points closer to a plane than a threshold,
     *                  weighting each by its alpha channel.
     *
     * @param[in]       modelCoefficients_in
     *                  Plane coefficients (a, b, c, d) of
     *                  a * x + b * y + c * z + d = 0, in the frame of the
     *                  input cloud.
     *
     * @param[in]       distanceThreshold_in
     *                  A point is an inlier when its distance to the plane is
     *                  below this value, in the length unit of the input
     *                  cloud.
     *
     * @return          0 when the coefficients are invalid (wrong count, or
     *                  rejected by the custom model constraint); for
     *                  pcl::PointXYZRGBA clouds the sum of alpha / 255 over
     *                  the inliers, truncated toward zero; for other point
     *                  types the unweighted count of the PCL base.
     *
     * @note            Overrides the PCL virtual that the SAC estimators call
     *                  to score each candidate plane.
     */
    std::size_t
        countWithinDistance(const Eigen::VectorXf &modelCoefficients_in,
                            const double distanceThreshold_in) const override;
};

} /* namespace vs_graphs::core::pcl_custom */

#include "dependency_lineage/pcl_custom/methods/WeightedSACModelPlaneClass/countWithinDistance.tpp"

#endif /* VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_MODEL_PLANE_CLASS_H \
        */
