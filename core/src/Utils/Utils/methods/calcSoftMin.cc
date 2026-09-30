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
 * @file            calcSoftMin.cc
 *
 * @brief           Implements Utils::calcSoftMin(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::calcSoftMin(std::vector<double> &values_in,
                               double              &softMin_out)
{
    // parameter controlling the softness/sharpness of the soft-min
    // the smaller the value, the more conservative the soft-min
    const double tau = 0.1;

    // soft-min = sum(exp(-value/tau) * value) / sum(exp(-value/tau))
    Eigen::Map<Eigen::VectorXd> confs(values_in.data(), values_in.size());
    Eigen::VectorXd             term = ((1.0 - confs.array()) / tau).exp();
    softMin_out = ((term / term.sum()).array() * confs.array()).sum();
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
