/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Tracking.h"

#include <iostream>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Tracking::preintegrateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        Verbose::printMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        currentFrame.setIntegrated();
        return;
    }

    imuFromLastFrame.clear();
    imuFromLastFrame.reserve(queueImuData.size());
    if (queueImuData.size() == 0)
    {
        Verbose::printMess("Not IMU data in mlQueueImuData!!",
                           Verbose::VERBOSITY_NORMAL);
        currentFrame.setIntegrated();
        return;
    }

    while (true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if (!queueImuData.empty())
            {
                IMU::Point *m = &queueImuData.front();
                cout.precision(17);
                if (m->t < currentFrame.p_previousFrame->timeStamp - imuPeriod)
                    queueImuData.pop_front();
                else if (m->t < currentFrame.timeStamp - imuPeriod)
                {
                    imuFromLastFrame.push_back(*m);
                    queueImuData.pop_front();
                }
                else
                {
                    imuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if (bSleep)
            usleep(500);
    }

    const int n = imuFromLastFrame.size() - 1;
    if (n == 0)
    {
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame =
        std::make_shared<IMU::Preintegrated>(lastFrame.imuBias,
                                             currentFrame.imuCalibration);

    for (int i = 0; i < n; i++)
    {
        float           tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1)))
        {
            float tab = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
            float tini =
                imuFromLastFrame[i].t - currentFrame.p_previousFrame->timeStamp;
            acc = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a -
                   (imuFromLastFrame[i + 1].a - imuFromLastFrame[i].a) *
                       (tini / tab)) *
                  0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w -
                      (imuFromLastFrame[i + 1].w - imuFromLastFrame[i].w) *
                          (tini / tab)) *
                     0.5f;
            tstep = imuFromLastFrame[i + 1].t -
                    currentFrame.p_previousFrame->timeStamp;
        }
        else if (i < (n - 1))
        {
            acc    = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w) * 0.5f;
            tstep  = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
        }
        else if ((i > 0) && (i == (n - 1)))
        {
            float tab  = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
            float tend = imuFromLastFrame[i + 1].t - currentFrame.timeStamp;
            acc        = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a -
                   (imuFromLastFrame[i + 1].a - imuFromLastFrame[i].a) *
                       (tend / tab)) *
                  0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w -
                      (imuFromLastFrame[i + 1].w - imuFromLastFrame[i].w) *
                          (tend / tab)) *
                     0.5f;
            tstep = currentFrame.timeStamp - imuFromLastFrame[i].t;
        }
        else if ((i == 0) && (i == (n - 1)))
        {
            acc    = imuFromLastFrame[i].a;
            angVel = imuFromLastFrame[i].w;
            tstep  = currentFrame.timeStamp -
                    currentFrame.p_previousFrame->timeStamp;
        }

        if (!p_imuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        p_imuPreintegratedFromLastKF->integrateNewMeasurement(acc,
                                                              angVel,
                                                              tstep);
        pImuPreintegratedFromLastFrame->integrateNewMeasurement(acc,
                                                                angVel,
                                                                tstep);
    }

    currentFrame.p_imuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    currentFrame.p_imuPreintegrated      = p_imuPreintegratedFromLastKF;
    currentFrame.p_lastKeyFrame          = p_lastKeyFrame;

    currentFrame.setIntegrated();

    // Verbose::PrintMess("Preintegration is finished!! ",
    // Verbose::VERBOSITY_DEBUG);
}

} // namespace core
} // namespace vs_graphs
