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

#include "System.h"
#include "Tracking.h"

#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::preintegrateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        if (Verbose::printMess("non prev frame ", Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrame.setIntegrated() != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setIntegrated returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    imuFromLastFrame.clear();
    imuFromLastFrame.reserve(queueImuData.size());
    if (queueImuData.size() == 0)
    {
        if (Verbose::printMess("Not IMU data in mlQueueImuData!!",
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrame.setIntegrated() != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setIntegrated returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    while (true)
    {
        bool shouldSleep = false;
        {
            std::unique_lock<std::mutex> lock(imuQueueMutex);
            if (!queueImuData.empty())
            {
                IMU::Point *m = &queueImuData.front();
                std::cout.precision(17);
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
                shouldSleep = true;
            }
        }
        if (shouldSleep)
            usleep(500);
    }

    const int n = imuFromLastFrame.size() - 1;
    if (n == 0)
    {
        std::cout << "Empty IMU measurements vector!!!\n";
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame =
        std::make_shared<IMU::Preintegrated>(lastFrame.imuBias,
                                             currentFrame.imuCalibration);

    for (int measurementIndex = 0; measurementIndex < n; measurementIndex++)
    {
        float           tstep;
        Eigen::Vector3f acceleration, angleVelocity;
        if ((measurementIndex == 0) && (measurementIndex < (n - 1)))
        {
            float tab = imuFromLastFrame[measurementIndex + 1].t -
                        imuFromLastFrame[measurementIndex].t;
            float tini = imuFromLastFrame[measurementIndex].t -
                         currentFrame.p_previousFrame->timeStamp;
            acceleration = (imuFromLastFrame[measurementIndex].a +
                            imuFromLastFrame[measurementIndex + 1].a -
                            (imuFromLastFrame[measurementIndex + 1].a -
                             imuFromLastFrame[measurementIndex].a) *
                                (tini / tab)) *
                           0.5f;
            angleVelocity = (imuFromLastFrame[measurementIndex].w +
                             imuFromLastFrame[measurementIndex + 1].w -
                             (imuFromLastFrame[measurementIndex + 1].w -
                              imuFromLastFrame[measurementIndex].w) *
                                 (tini / tab)) *
                            0.5f;
            tstep = imuFromLastFrame[measurementIndex + 1].t -
                    currentFrame.p_previousFrame->timeStamp;
        }
        else if (measurementIndex < (n - 1))
        {
            acceleration = (imuFromLastFrame[measurementIndex].a +
                            imuFromLastFrame[measurementIndex + 1].a) *
                           0.5f;
            angleVelocity = (imuFromLastFrame[measurementIndex].w +
                             imuFromLastFrame[measurementIndex + 1].w) *
                            0.5f;
            tstep = imuFromLastFrame[measurementIndex + 1].t -
                    imuFromLastFrame[measurementIndex].t;
        }
        else if ((measurementIndex > 0) && (measurementIndex == (n - 1)))
        {
            float tab = imuFromLastFrame[measurementIndex + 1].t -
                        imuFromLastFrame[measurementIndex].t;
            float tend = imuFromLastFrame[measurementIndex + 1].t -
                         currentFrame.timeStamp;
            acceleration = (imuFromLastFrame[measurementIndex].a +
                            imuFromLastFrame[measurementIndex + 1].a -
                            (imuFromLastFrame[measurementIndex + 1].a -
                             imuFromLastFrame[measurementIndex].a) *
                                (tend / tab)) *
                           0.5f;
            angleVelocity = (imuFromLastFrame[measurementIndex].w +
                             imuFromLastFrame[measurementIndex + 1].w -
                             (imuFromLastFrame[measurementIndex + 1].w -
                              imuFromLastFrame[measurementIndex].w) *
                                 (tend / tab)) *
                            0.5f;
            tstep =
                currentFrame.timeStamp - imuFromLastFrame[measurementIndex].t;
        }
        else if ((measurementIndex == 0) && (measurementIndex == (n - 1)))
        {
            acceleration  = imuFromLastFrame[measurementIndex].a;
            angleVelocity = imuFromLastFrame[measurementIndex].w;
            tstep         = currentFrame.timeStamp -
                    currentFrame.p_previousFrame->timeStamp;
        }

        if (!p_imuPreintegratedFromLastKF)
            std::cout << "mpImuPreintegratedFromLastKF does not exist"
                      << std::endl;
        if (p_imuPreintegratedFromLastKF->integrateNewMeasurement(acceleration,
                                                                  angleVelocity,
                                                                  tstep) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: integrateNewMeasurement returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (pImuPreintegratedFromLastFrame
                ->integrateNewMeasurement(acceleration, angleVelocity, tstep) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: integrateNewMeasurement returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    currentFrame.p_imuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    currentFrame.p_imuPreintegrated      = p_imuPreintegratedFromLastKF;
    currentFrame.p_lastKeyFrame          = p_lastKeyFrame;

    if (currentFrame.setIntegrated() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setIntegrated returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Verbose::PrintMess("Preintegration is finished!! ",
    // Verbose::VERBOSITY_DEBUG);

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
