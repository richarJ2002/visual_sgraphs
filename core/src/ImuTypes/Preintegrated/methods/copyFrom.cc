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

void Preintegrated::copyFrom(Preintegrated *pImuPre)
{
    dT      = pImuPre->dT;
    C       = pImuPre->C;
    Info    = pImuPre->Info;
    Nga     = pImuPre->Nga;
    NgaWalk = pImuPre->NgaWalk;
    b.copyFrom(pImuPre->b);
    dR   = pImuPre->dR;
    dV   = pImuPre->dV;
    dP   = pImuPre->dP;
    JRg  = pImuPre->JRg;
    JVg  = pImuPre->JVg;
    JVa  = pImuPre->JVa;
    JPg  = pImuPre->JPg;
    JPa  = pImuPre->JPa;
    avgA = pImuPre->avgA;
    avgW = pImuPre->avgW;
    bu.copyFrom(pImuPre->bu);
    db             = pImuPre->db;
    mvMeasurements = pImuPre->mvMeasurements;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
