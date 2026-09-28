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

namespace vs_graphs
{
namespace core
{
namespace IMU
{

void Preintegrated::copyFrom(Preintegrated *p_sourcePreintegrated_in)
{
    dT      = p_sourcePreintegrated_in->dT;
    C       = p_sourcePreintegrated_in->C;
    Info    = p_sourcePreintegrated_in->Info;
    Nga     = p_sourcePreintegrated_in->Nga;
    NgaWalk = p_sourcePreintegrated_in->NgaWalk;
    b.copyFrom(p_sourcePreintegrated_in->b);
    dR   = p_sourcePreintegrated_in->dR;
    dV   = p_sourcePreintegrated_in->dV;
    dP   = p_sourcePreintegrated_in->dP;
    JRg  = p_sourcePreintegrated_in->JRg;
    JVg  = p_sourcePreintegrated_in->JVg;
    JVa  = p_sourcePreintegrated_in->JVa;
    JPg  = p_sourcePreintegrated_in->JPg;
    JPa  = p_sourcePreintegrated_in->JPa;
    avgA = p_sourcePreintegrated_in->avgA;
    avgW = p_sourcePreintegrated_in->avgW;
    bu.copyFrom(p_sourcePreintegrated_in->bu);
    db           = p_sourcePreintegrated_in->db;
    measurements = p_sourcePreintegrated_in->measurements;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
