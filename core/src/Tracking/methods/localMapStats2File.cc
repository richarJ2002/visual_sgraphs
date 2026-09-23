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

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
void Tracking::localMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
         "culling[ms], Total[ms]"
      << endl;
    for (int i = 0; i < p_localMapper->vdLMTotal_ms.size(); ++i)
    {
        f << p_localMapper->vdKFInsert_ms[i] << ","
          << p_localMapper->vdMPCulling_ms[i] << ","
          << p_localMapper->vdMPCreation_ms[i] << ","
          << p_localMapper->vdLBASync_ms[i] << ","
          << p_localMapper->vdKFCullingSync_ms[i] << ","
          << p_localMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for (int i = 0; i < p_localMapper->vdLBASync_ms.size(); ++i)
    {
        f << p_localMapper->vdLBASync_ms[i] << ","
          << p_localMapper->vnLBA_KFopt[i] << ","
          << p_localMapper->vnLBA_KFfixed[i] << ","
          << p_localMapper->vnLBA_MPs[i] << "," << p_localMapper->vnLBA_edges[i]
          << endl;
    }

    f.close();
}
#endif

} // namespace core
} // namespace vs_graphs
