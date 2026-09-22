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

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

void Preintegrated::mergePrevious(Preintegrated *pPrev)
{
    if (pPrev == this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias                         bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<Integrable> aux1 = pPrev->mvMeasurements;
    const std::vector<Integrable> aux2 = mvMeasurements;

    initialize(bav);
    for (size_t i = 0; i < aux1.size(); i++)
        integrateNewMeasurement(aux1[i].a, aux1[i].w, aux1[i].t);
    for (size_t i = 0; i < aux2.size(); i++)
        integrateNewMeasurement(aux2[i].a, aux2[i].w, aux2[i].t);
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
