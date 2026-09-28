/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
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
 * @file            arePlanesApartEnough.cc
 *
 * @brief           Implements Utils::arePlanesApartEnough(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::arePlanesApartEnough(
    const vs_graphs::core::geometric::Plane *p_plane1_in,
    const vs_graphs::core::geometric::Plane *p_plane2_in,
    const double                            &threshold_in,
    bool                                    &arePlanesApartEnough_out)
{
    if (p_plane1_in == nullptr || p_plane2_in == nullptr)
    {
        arePlanesApartEnough_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    Eigen::Vector4d equation1 = p_plane1_in->getGlobalEquation().coeffs();
    Eigen::Vector4d equation2 = p_plane2_in->getGlobalEquation().coeffs();

    const double normalNorm1 = equation1.head<3>().norm();
    const double normalNorm2 = equation2.head<3>().norm();

    if (!std::isfinite(normalNorm1) || !std::isfinite(normalNorm2) ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        arePlanesApartEnough_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    const double normalAlignment = equation1.head<3>().dot(equation2.head<3>());

    if (std::abs(normalAlignment) <= 0.99)
    {
        arePlanesApartEnough_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    if (normalAlignment < 0.0)
    {
        equation2 *= -1.0;
    }

    const double perpendicularSeparation_m =
        std::abs(equation1(3) - equation2(3));

    arePlanesApartEnough_out = perpendicularSeparation_m > threshold_in;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
