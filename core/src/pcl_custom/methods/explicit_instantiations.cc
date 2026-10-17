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
 * @file            explicit_instantiations.cc
 *
 * @brief           Instantiates every pcl_custom class template for the point
 *                  types the project uses, so that each member is compiled
 *                  even when no caller uses it.
 */

#include "pcl_custom/pcl_custom.h"

#include <pcl/point_types.h>

namespace vs_graphs::core::pcl_custom
{

/*!
 * @brief           Explicit instantiation of WeightedSACModelPlane for
 *                  pcl::PointXYZRGBA clouds.
 */
template class WeightedSACModelPlane<pcl::PointXYZRGBA>;

/*!
 * @brief           Explicit instantiation of WeightedSACSegmentation for
 *                  pcl::PointXYZRGBA clouds, the type
 *                  utils::utils::Utils::ransacPlaneFitting() segments.
 */
template class WeightedSACSegmentation<pcl::PointXYZRGBA>;

} /* namespace vs_graphs::core::pcl_custom */
