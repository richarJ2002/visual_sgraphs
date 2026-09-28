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

void MapDrawer::drawKeyFrames(const bool shouldDrawKeyFrames_in,
                              const bool shouldDrawGraph_in,
                              const bool shouldDrawInertialGraph_in,
                              const bool shouldDrawOptimizedLba_in)
{
    const float &w = keyFrameSize;
    const float  h = w * 0.75;
    const float  z = w * 0.6;

    Map                        *p_activeMap = p_atlas->getCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> optKeyFrames   = p_activeMap->optKeyFrameIds;
    std::set<long unsigned int> fixedKeyFrames = p_activeMap->fixedKeyFrameIds;

    if (!p_activeMap)
        return;

    const vector<KeyFrame *> keyFrames = p_activeMap->getAllKeyFrames();

    if (shouldDrawKeyFrames_in)
    {
        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            KeyFrame       *p_keyFrame = keyFrames[keyFrameIndex];
            Eigen::Matrix4f Twc        = p_keyFrame->getPoseInverse().matrix();
            unsigned int    indexColor = p_keyFrame->originMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *)Twc.data());

            if (!p_keyFrame->getParent()) // It is the first KF in the map
            {
                glLineWidth(keyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                // cout << "Child KF: " << vpKFs[i]->id << endl;
                glLineWidth(keyFrameLineWidth);
                if (shouldDrawOptimizedLba_in)
                {
                    if (optKeyFrames.find(p_keyFrame->id) != optKeyFrames.end())
                    {
                        glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                    }
                    else if (fixedKeyFrames.find(p_keyFrame->id) !=
                             fixedKeyFrames.end())
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

    if (shouldDrawGraph_in)
    {
        glLineWidth(graphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            // Covisibility Graph
            const vector<KeyFrame *> covisibleKeyFrames =
                keyFrames[keyFrameIndex]->getCovisiblesByWeight(100);
            Eigen::Vector3f Ow = keyFrames[keyFrameIndex]->getCameraCenter();
            if (!covisibleKeyFrames.empty())
            {
                for (vector<KeyFrame *>::const_iterator
                         vit  = covisibleKeyFrames.begin(),
                         vend = covisibleKeyFrames.end();
                     vit != vend;
                     vit++)
                {
                    if ((*vit)->id < keyFrames[keyFrameIndex]->id)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->getCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame *p_parent = keyFrames[keyFrameIndex]->getParent();
            if (p_parent)
            {
                Eigen::Vector3f Owp = p_parent->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            // Loops
            set<KeyFrame *> loopKeyFrames =
                keyFrames[keyFrameIndex]->getLoopEdges();
            for (set<KeyFrame *>::iterator sit  = loopKeyFrames.begin(),
                                           send = loopKeyFrames.end();
                 sit != send;
                 sit++)
            {
                if ((*sit)->id < keyFrames[keyFrameIndex]->id)
                    continue;
                Eigen::Vector3f Owl = (*sit)->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    if (shouldDrawInertialGraph_in && p_activeMap->isImuInitialized())
    {
        glLineWidth(graphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // Draw inertial links
        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            KeyFrame       *p_drawnKeyFrame = keyFrames[keyFrameIndex];
            Eigen::Vector3f Ow     = p_drawnKeyFrame->getCameraCenter();
            KeyFrame       *p_next = p_drawnKeyFrame->p_nextKF;
            if (p_next)
            {
                Eigen::Vector3f Owp = p_next->getCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    vector<Map *> maps = p_atlas->getAllMaps();

    if (shouldDrawKeyFrames_in)
    {
        for (Map *p_map : maps)
        {
            if (p_map == p_activeMap)
                continue;

            vector<KeyFrame *> keyFrames = p_map->getAllKeyFrames();

            for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
                 keyFrameIndex++)
            {
                KeyFrame       *p_keyFrame = keyFrames[keyFrameIndex];
                Eigen::Matrix4f Twc = p_keyFrame->getPoseInverse().matrix();
                unsigned int    indexColor = p_keyFrame->originMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *)Twc.data());

                if (!keyFrames[keyFrameIndex]
                         ->getParent()) // It is the first KF in the map
                {
                    glLineWidth(keyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(keyFrameLineWidth);
                    glColor3f(frameColors[indexColor][0],
                              frameColors[indexColor][1],
                              frameColors[indexColor][2]);
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
