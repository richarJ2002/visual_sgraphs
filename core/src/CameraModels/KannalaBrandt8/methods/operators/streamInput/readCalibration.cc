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
 * @file            readCalibration.cc
 *
 * @brief           Implements the KannalaBrandt8 input stream operator,
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <cstddef>
#include <istream>

#include <assert.h>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
std::istream &operator>>(std::istream   &inputStream_inout,
                         KannalaBrandt8 &kannala_out)
{
    float nextParameter;
    for (size_t parameterIndex = 0; parameterIndex < 8; parameterIndex++)
    {
        assert(inputStream_inout.good()); // Make sure the input stream is good
        inputStream_inout >> nextParameter;
        kannala_out.parameters[parameterIndex] = nextParameter;
    }
    return inputStream_inout;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
