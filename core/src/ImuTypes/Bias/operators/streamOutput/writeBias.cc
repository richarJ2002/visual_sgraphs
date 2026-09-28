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

#include "ImuTypes.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

std::ostream &operator<<(std::ostream &out_inout, const Bias &b_in)
{
    if (b_in.bwx > 0)
        out_inout << " ";
    out_inout << b_in.bwx << ",";
    if (b_in.bwy > 0)
        out_inout << " ";
    out_inout << b_in.bwy << ",";
    if (b_in.bwz > 0)
        out_inout << " ";
    out_inout << b_in.bwz << ",";
    if (b_in.bax > 0)
        out_inout << " ";
    out_inout << b_in.bax << ",";
    if (b_in.bay > 0)
        out_inout << " ";
    out_inout << b_in.bay << ",";
    if (b_in.baz > 0)
        out_inout << " ";
    out_inout << b_in.baz;

    return out_inout;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
