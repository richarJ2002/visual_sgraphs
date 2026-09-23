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

#include "ORBmatcher.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

int LoopClosing::findMatchesByProjection(KeyFrame        *pCurrentKF,
                                         KeyFrame        *pMatchedKFw,
                                         g2o::Sim3       &g2oScw,
                                         set<MapPoint *> &spMatchedMPinOrigin,
                                         vector<MapPoint *> &vpMapPoints,
                                         vector<MapPoint *> &vpMatchedMapPoints)
{
    int                nNumCovisibles = 10;
    vector<KeyFrame *> vpCovKFm =
        pMatchedKFw->getBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame *> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame *> spCurrentCovisbles = pCurrentKF->getConnectedKeyFrames();
    if (nInitialCov < nNumCovisibles)
    {
        for (int i = 0; i < nInitialCov; ++i)
        {
            vector<KeyFrame *> vpKFs =
                vpCovKFm[i]->getBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j         = 0;
            while (j < vpKFs.size() && nInserted < nNumCovisibles)
            {
                if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() &&
                    spCurrentCovisbles.find(vpKFs[j]) ==
                        spCurrentCovisbles.end())
                {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPoint *> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for (KeyFrame *pKFi : vpCovKFm)
    {
        for (MapPoint *pMPij : pKFi->getMapPointMatches())
        {
            if (!pMPij || pMPij->isBad())
                continue;

            if (spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    Sophus::Sim3f correctedPose = utils::converter::Converter::toSophus(g2oScw);
    ORBmatcher    matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->getMapPointMatches().size(),
                              static_cast<MapPoint *>(nullptr));
    int num_matches = matcher.searchByProjection(pCurrentKF,
                                                 correctedPose,
                                                 vpMapPoints,
                                                 vpMatchedMapPoints,
                                                 3,
                                                 1.5);

    return num_matches;
}

} // namespace core
} // namespace vs_graphs
