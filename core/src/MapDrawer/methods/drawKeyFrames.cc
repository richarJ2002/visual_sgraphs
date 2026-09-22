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

#include "KeyFrame.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace vs_graphs
{
namespace core
{

void MapDrawer::drawKeyFrames(const bool bDrawKF,
                              const bool bDrawGraph,
                              const bool bDrawInertialGraph,
                              const bool bDrawOptLba)
{
    const float &w = keyFrameSize;
    const float  h = w * 0.75;
    const float  z = w * 0.6;

    Map                        *pActiveMap = p_atlas->getCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs   = pActiveMap->optKeyFrameIds;
    std::set<long unsigned int> sFixedKFs = pActiveMap->fixedKeyFrameIds;

    if (!pActiveMap)
        return;

    const vector<KeyFrame *> vpKFs = pActiveMap->getAllKeyFrames();

    if (bDrawKF)
    {
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame       *pKF         = vpKFs[i];
            Eigen::Matrix4f Twc         = pKF->getPoseInverse().matrix();
            unsigned int    index_color = pKF->originMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!pKF->getParent()) // It is the first KF in the map
            {
                glLineWidth(keyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                // cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(keyFrameLineWidth);
                if (bDrawOptLba)
                {
                    if (sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                    }
                    else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if (bDrawGraph)
    {
        glLineWidth(graphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame *> vCovKFs =
                vpKFs[i]->getCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->getCameraCenter();
            if (!vCovKFs.empty())
            {
                for (vector<KeyFrame *>::const_iterator vit  = vCovKFs.begin(),
                                                        vend = vCovKFs.end();
                     vit != vend;
                     vit++)
                {
                    if ((*vit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->getCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame *pParent = vpKFs[i]->getParent();
            if (pParent)
            {
                Eigen::Vector3f Owp = pParent->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            // Loops
            set<KeyFrame *> sLoopKFs = vpKFs[i]->getLoopEdges();
            for (set<KeyFrame *>::iterator sit  = sLoopKFs.begin(),
                                           send = sLoopKFs.end();
                 sit != send;
                 sit++)
            {
                if ((*sit)->mnId < vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    if (bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(graphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // Draw inertial links
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame       *pKFi  = vpKFs[i];
            Eigen::Vector3f Ow    = pKFi->getCameraCenter();
            KeyFrame       *pNext = pKFi->p_nextKF;
            if (pNext)
            {
                Eigen::Vector3f Owp = pNext->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    vector<Map *> vpMaps = p_atlas->getAllMaps();

    if (bDrawKF)
    {
        for (Map *pMap : vpMaps)
        {
            if (pMap == pActiveMap)
                continue;

            vector<KeyFrame *> vpKFs = pMap->getAllKeyFrames();

            for (size_t i = 0; i < vpKFs.size(); i++)
            {
                KeyFrame       *pKF         = vpKFs[i];
                Eigen::Matrix4f Twc         = pKF->getPoseInverse().matrix();
                unsigned int    index_color = pKF->originMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *)Twc.data());

                if (!vpKFs[i]->getParent()) // It is the first KF in the map
                {
                    glLineWidth(keyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(keyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],
                              mfFrameColors[index_color][1],
                              mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
