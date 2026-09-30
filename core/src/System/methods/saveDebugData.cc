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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "LocalMapping.h"
#include "System.h"

#include <iomanip>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::saveDebugData(const int &initialIndex_in)
{
    // 0. Save initialization trajectory
    if (saveTrajectoryEuRoC("init_FrameTrajectoy_" +
                            std::to_string(p_localMapper->initSection) + "_" +
                            std::to_string(initialIndex_in) + ".txt") !=
        SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: saveTrajectoryEuRoC returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // 1. Save scale
    std::ofstream f;
    f.open("init_Scale_" + std::to_string(p_localMapper->initSection) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << p_localMapper->scale << std::endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + std::to_string(p_localMapper->initSection) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << p_localMapper->mRwg(0, 0) << "," << p_localMapper->mRwg(0, 1) << ","
      << p_localMapper->mRwg(0, 2) << std::endl;
    f << p_localMapper->mRwg(1, 0) << "," << p_localMapper->mRwg(1, 1) << ","
      << p_localMapper->mRwg(1, 2) << std::endl;
    f << p_localMapper->mRwg(2, 0) << "," << p_localMapper->mRwg(2, 1) << ","
      << p_localMapper->mRwg(2, 2) << std::endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + std::to_string(p_localMapper->initSection) +
               ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << p_localMapper->costTime << std::endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + std::to_string(p_localMapper->initSection) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << p_localMapper->mbg(0) << "," << p_localMapper->mbg(1) << ","
      << p_localMapper->mbg(2) << std::endl;
    f << p_localMapper->mba(0) << "," << p_localMapper->mba(1) << ","
      << p_localMapper->mba(2) << std::endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + std::to_string(p_localMapper->initSection) +
               "_" + std::to_string(initialIndex_in) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    for (int rowIndex = 0; rowIndex < p_localMapper->mcovInertial.rows();
         rowIndex++)
    {
        for (int columnIndex = 0;
             columnIndex < p_localMapper->mcovInertial.cols();
             columnIndex++)
        {
            if (columnIndex != 0)
                f << ",";
            f << std::setprecision(15)
              << p_localMapper->mcovInertial(rowIndex, columnIndex);
        }
        f << std::endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + std::to_string(p_localMapper->initSection) + ".txt",
           std::ios_base::app);
    f << std::fixed;
    f << p_localMapper->initTime << std::endl;
    f.close();

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
