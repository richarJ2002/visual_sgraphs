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

#include "System.h"

#include <iomanip>

namespace vs_graphs
{
namespace core
{

void System::saveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    saveTrajectoryEuRoC("init_FrameTrajectoy_" +
                        to_string(p_localMapper->initSection) + "_" +
                        to_string(initIdx) + ".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->scale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->mRwg(0, 0) << "," << p_localMapper->mRwg(0, 1) << ","
      << p_localMapper->mRwg(0, 2) << endl;
    f << p_localMapper->mRwg(1, 0) << "," << p_localMapper->mRwg(1, 1) << ","
      << p_localMapper->mRwg(1, 2) << endl;
    f << p_localMapper->mRwg(2, 0) << "," << p_localMapper->mRwg(2, 1) << ","
      << p_localMapper->mRwg(2, 2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->costTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->mbg(0) << "," << p_localMapper->mbg(1) << ","
      << p_localMapper->mbg(2) << endl;
    f << p_localMapper->mba(0) << "," << p_localMapper->mba(1) << ","
      << p_localMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + to_string(p_localMapper->initSection) + "_" +
               to_string(initIdx) + ".txt",
           ios_base::app);
    f << fixed;
    for (int i = 0; i < p_localMapper->mcovInertial.rows(); i++)
    {
        for (int j = 0; j < p_localMapper->mcovInertial.cols(); j++)
        {
            if (j != 0)
                f << ",";
            f << setprecision(15) << p_localMapper->mcovInertial(i, j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->initTime << endl;
    f.close();
}

} // namespace core
} // namespace vs_graphs
