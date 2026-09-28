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
 * @brief           Implements the KannalaBrandt8 output stream operator,
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <ostream>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
std::ostream &operator<<(std::ostream         &outputStream_inout,
                         const KannalaBrandt8 &kannala_in)
{
    outputStream_inout << kannala_in.parameters[0] << " "
                       << kannala_in.parameters[1] << " "
                       << kannala_in.parameters[2] << " "
                       << kannala_in.parameters[3] << " "
                       << kannala_in.parameters[4] << " "
                       << kannala_in.parameters[5] << " "
                       << kannala_in.parameters[6] << " "
                       << kannala_in.parameters[7];
    return outputStream_inout;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
