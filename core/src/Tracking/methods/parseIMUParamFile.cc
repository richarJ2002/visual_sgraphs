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

#include <cmath>
#include <iostream>

namespace vs_graphs
{
namespace core
{

bool Tracking::parseIMUParamFile(cv::FileStorage &fSettings)
{
    bool  boolMissingParam = false;
    float Ng               = 0.0F;
    float Na               = 0.0F;
    float Ngw              = 0.0F;
    float Naw              = 0.0F;

    cv::Mat      cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if (!node.empty())
    {
        cvTbc = node.mat();
        if (cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr
                << "\t- Tbc matrix needs to be a 4x4 transformation matrix!"
                << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "\t- Tbc matrix does not exist!" << std::endl;
        return false;
    }
    cout << "\t- Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f                                Tbc(eigTbc);

    node          = fSettings["InsertKFsWhenLost"];
    insertKFsLost = true;
    if (!node.empty() && node.isInt())
    {
        insertKFsLost = (bool)node.operator int();
    }

    if (!insertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;

    node = fSettings["IMU.Frequency"];
    if (!node.empty() && node.isInt())
    {
        imuFrequency = node.operator int();
        imuPeriod    = 1.0 / static_cast<double>(imuFrequency);
    }
    else
    {
        std::cerr
            << "*IMU.Frequency parameter doesn't exist or is not an integer*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if (!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.Threshold"];
    if (!node.empty() && node.isReal())
        imuThresh = node.real();
    else
    {
        std::cerr << "- IMU.Threshold parameter doesn't exist or is not a real "
                     "number!"
                  << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if (!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if (!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.GyroWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.AccWalk"];
    if (!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.AccWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node     = fSettings["IMU.FastInit"];
    fastInit = false;
    if (!node.empty())
        fastInit = static_cast<int>(fSettings["IMU.FastInit"]) != 0;

    if (fastInit)
        std::cout << "\t- Fast IMU initialization triggered! Acceleration is "
                     "not checked!"
                  << std::endl;

    if (boolMissingParam)
        return false;

    const float sf = sqrt(imuFrequency);
    cout << endl;
    cout << "IMU frequency: " << imuFrequency << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    p_imuCalibration =
        new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    p_imuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);

    return true;
}

} // namespace core
} // namespace vs_graphs
