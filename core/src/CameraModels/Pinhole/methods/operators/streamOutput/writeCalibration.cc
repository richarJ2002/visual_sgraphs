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
 * @file            writeCalibration.cc
 *
 * @brief           Implements the Pinhole output stream operator,
 *                  declared in CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <ostream>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
std::ostream &operator<<(std::ostream &os, const Pinhole &pinhole_in)
{
    os << pinhole_in.parameters[0] << " " << pinhole_in.parameters[1] << " "
       << pinhole_in.parameters[2] << " " << pinhole_in.parameters[3];
    return os;
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
