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
 * @file            ImuConstants.h
 *
 * @brief           Declares the IMU gravity and singularity constants shared
 *                  by preintegration, its callers and IntegratedRotation.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_IMU_CONSTANTS_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_IMU_CONSTANTS_H

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace preintegrated
{

/*!
 * @brief        Gravity magnitude in metres per second squared.
 */
const float GRAVITY_VALUE = 9.81;

/*!
 * @brief        Singularity threshold shared by the SO(3) Jacobian
 *               helpers and IntegratedRotation.
 */
const float eps = 1e-4;

} /* namespace preintegrated */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_IMU_CONSTANTS_H   \
        */
