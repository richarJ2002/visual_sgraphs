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
 * @file            computeMedianValue.cc
 *
 * @brief           Implements computeMedianValue(), declared in
 *                  passage_detection/slab_profiles/private_functions/
 *                  private_functions.h.
 */

#include "private_functions.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{

double computeMedianValue(const std::vector<double> &values_in)
{
    std::vector<double> partlySortedValues = values_in;
    const std::size_t   middle             = partlySortedValues.size() / 2U;
    std::nth_element(partlySortedValues.begin(),
                     partlySortedValues.begin() + middle,
                     partlySortedValues.end());
    return partlySortedValues[middle];
}

} /* namespace core */
} /* namespace vs_graphs */
