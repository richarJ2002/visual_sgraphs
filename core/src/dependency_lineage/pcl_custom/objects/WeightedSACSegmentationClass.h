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
 * pcl/segmentation/sac_segmentation.h
 * (class SACSegmentation); its licence notice follows.
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * @file            WeightedSACSegmentationClass.h
 *
 * @brief           Declares the SAC segmentation that fits planes with the
 *                  alpha-weighted plane model.
 */

#ifndef VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_SEGMENTATION_CLASS_H
#define VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_SEGMENTATION_CLASS_H

#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "dependency_lineage/pcl_custom/objects/WeightedSACModelPlaneClass.h"

namespace vs_graphs::core::pcl_custom
{

/*!
 * @brief           PCL SAC segmentation that fits pcl::SACMODEL_PLANE with
 *                  the alpha-weighted WeightedSACModelPlane.
 *
 *                  Every other model type is created by
 *                  pcl::SACSegmentation. The project uses it through
 *                  utils::utils::Utils::ransacPlaneFitting().
 *
 * @tparam          PointType
 *                  PCL point type of the input cloud.
 *
 * @note            Keeps exactly one type parameter: ransacPlaneFitting()
 *                  takes it as a template <typename> class argument.
 */
template <typename PointType>
class WeightedSACSegmentation : public pcl::SACSegmentation<PointType>
{
  public:
    /*!
     * @brief           The PCL segmentation this class extends.
     */
    using BaseSegmentation = pcl::SACSegmentation<PointType>;

    /*!
     * @brief           Creates a segmentation with the default settings of
     *                  pcl::SACSegmentation.
     */
    WeightedSACSegmentation() :
        BaseSegmentation()
    {}

    /*!
     * @brief           Creates the SAC model of the requested type for the
     *                  current input cloud and indices.
     *
     * @param[in]       modelType_in
     *                  pcl::SacModel value; pcl::SACMODEL_PLANE selects
     *                  WeightedSACModelPlane.
     *
     * @return          true after creating the weighted plane model; for
     *                  other types the result of
     *                  pcl::SACSegmentation::initSACModel().
     *
     * @pre             The input cloud and indices are set, as
     *                  pcl::SACSegmentation::segment() ensures (it runs
     *                  initCompute() before this call).
     *
     * @post            For pcl::SACMODEL_PLANE the base member model_ holds
     *                  a new WeightedSACModelPlane that shares input_, copies
     *                  indices_ and is seeded according to random_.
     *
     * @note            Overrides the PCL virtual that segment() calls.
     */
    bool initSACModel(const int modelType_in) override;
};

} /* namespace vs_graphs::core::pcl_custom */

#include "dependency_lineage/pcl_custom/methods/WeightedSACSegmentationClass/initSACModel.tpp"

#endif /* VS_GRAPHS_CORE_SRC_PCL_CUSTOM_OBJECTS_WEIGHTED_SAC_SEGMENTATION_CLASS_H \
        */
