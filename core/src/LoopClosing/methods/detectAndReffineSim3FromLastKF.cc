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

#include "LoopClosing.h"

#include "Optimizer.h"

namespace vs_graphs
{
namespace core
{

bool LoopClosing::detectAndReffineSim3FromLastKF(
    KeyFrame                *pCurrentKF,
    KeyFrame                *pMatchedKF,
    g2o::Sim3               &gScw,
    int                     &nNumProjMatches,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    set<MapPoint *> spAlreadyMatchedMPs;
    nNumProjMatches = findMatchesByProjection(pCurrentKF,
                                              pMatchedKF,
                                              gScw,
                                              spAlreadyMatchedMPs,
                                              vpMPs,
                                              vpMatchedMPs);

    int nProjMatches    = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if (nNumProjMatches >= nProjMatches)
    {
        // Verbose::PrintMess("Sim3 reffine: There are " +
        // to_string(nNumProjMatches) + " initial matches ",
        // Verbose::VERBOSITY_DEBUG);
        Sophus::SE3d mTwm = pMatchedKF->getPoseInverse().cast<double>();
        g2o::Sim3    gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3    gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale =
            fixScale; // TODO CHECK; Solo para el monocular inertial
        if (p_tracker->sensor == System::IMU_MONOCULAR &&
            !pCurrentKF->getMap()->getInertialBA2())
            bFixedScale = false;
        int numOptMatches = Optimizer::optimizeSim3(p_currentKF,
                                                    pMatchedKF,
                                                    vpMatchedMPs,
                                                    gScm,
                                                    10,
                                                    bFixedScale,
                                                    mHessian7x7,
                                                    true);

        // Verbose::PrintMess("Sim3 reffine: There are " +
        // to_string(numOptMatches) + " matches after of the optimization ",
        // Verbose::VERBOSITY_DEBUG);

        if (numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

            vector<MapPoint *> vpMatchedMP;
            vpMatchedMP.resize(p_currentKF->getMapPointMatches().size(),
                               static_cast<MapPoint *>(nullptr));

            nNumProjMatches = findMatchesByProjection(pCurrentKF,
                                                      pMatchedKF,
                                                      gScw_estimation,
                                                      spAlreadyMatchedMPs,
                                                      vpMPs,
                                                      vpMatchedMPs);
            if (nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

} // namespace core
} // namespace vs_graphs
