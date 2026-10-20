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

/*!
 * @file            drawKeyFrames.cc
 *
 * @brief           Implements MapDrawer::drawKeyFrames(), declared in
 *                  MapDrawer.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "views_io/map_drawer/objects/MapDrawerClass.h"
#include <cstddef>
#include <iterator>
#include <mutex>
#include <pangolin/pangolin.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace map_drawer
{

MapDrawerStatus MapDrawer::drawKeyFrames(const bool shouldDrawKeyFrames_in,
                                         const bool shouldDrawGraph_in,
                                         const bool shouldDrawInertialGraph_in,
                                         const bool shouldDrawOptimizedLba_in)
{
    const float &w = keyFrameSize;
    const float  h = w * 0.75;
    const float  z = w * 0.6;

    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!p_activeMap)
        return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;

    /* DEBUG LBA */
    std::set<long unsigned int> optKeyFrames   = p_activeMap->optKeyFrameIds;
    std::set<long unsigned int> fixedKeyFrames = p_activeMap->fixedKeyFrameIds;

    std::vector<keyframe::KeyFrame *> keyFrames{};
    if (p_activeMap->getAllKeyFrames(keyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (shouldDrawKeyFrames_in)
    {
        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            keyframe::KeyFrame    *p_keyFrame = keyFrames[keyFrameIndex];
            Sophus::SE3f keyFramePoseInverse{};
            if (p_keyFrame->getPoseInverse(keyFramePoseInverse) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Matrix4f cameraPose_cameraToWorld =
                keyFramePoseInverse.matrix();

            glPushMatrix();

            glMultMatrixf(cameraPose_cameraToWorld.data());

            keyframe::KeyFrame *p_keyFrameParent = nullptr;
            if (p_keyFrame->getParent(p_keyFrameParent) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParent returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_keyFrameParent) /* It is the first KF in the map */
            {
                glLineWidth(keyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                glLineWidth(keyFrameLineWidth);
                if (shouldDrawOptimizedLba_in)
                {
                    if (optKeyFrames.find(p_keyFrame->id) != optKeyFrames.end())
                    {
                        glColor3f(0.0f, 1.0f, 0.0f); /* Green -> Opt KFs */
                    }
                    else if (fixedKeyFrames.find(p_keyFrame->id) !=
                             fixedKeyFrames.end())
                    {
                        glColor3f(1.0f, 0.0f, 0.0f); /* Red -> Fixed KFs */
                    }
                    else
                    {
                        glColor3f(0.0f, 0.0f, 1.0f); /* Basic color */
                    }
                }
                else
                {
                    glColor3f(0.0f, 0.0f, 1.0f); /* Basic color */
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

        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            /* Covisibility Graph */
            std::vector<keyframe::KeyFrame *> covisibleKeyFrames{};
            if (keyFrames[keyFrameIndex]->getCovisiblesByWeight(
                    100,
                    covisibleKeyFrames) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getCovisiblesByWeight returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            Eigen::Vector3f cameraCenter_world{};
            if (keyFrames[keyFrameIndex]->getCameraCenter(cameraCenter_world) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!covisibleKeyFrames.empty())
            {
                for (std::vector<keyframe::KeyFrame *>::const_iterator
                         vit  = covisibleKeyFrames.begin(),
                         vend = covisibleKeyFrames.end();
                     vit != vend;
                     vit++)
                {
                    if ((*vit)->id < keyFrames[keyFrameIndex]->id)
                        continue;
                    Eigen::Vector3f cameraCenter2_world{};
                    if ((*vit)->getCameraCenter(cameraCenter2_world) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    glVertex3f(cameraCenter_world(0),
                               cameraCenter_world(1),
                               cameraCenter_world(2));
                    glVertex3f(cameraCenter2_world(0),
                               cameraCenter2_world(1),
                               cameraCenter2_world(2));
                }
            }

            /* Spanning tree */
            keyframe::KeyFrame *p_parent = nullptr;
            if (keyFrames[keyFrameIndex]->getParent(p_parent) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParent returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_parent)
            {
                Eigen::Vector3f Owp{};
                if (p_parent->getCameraCenter(Owp) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCameraCenter returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                glVertex3f(cameraCenter_world(0),
                           cameraCenter_world(1),
                           cameraCenter_world(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            /* Loops */
            std::set<keyframe::KeyFrame *> loopKeyFrames{};
            if (keyFrames[keyFrameIndex]->getLoopEdges(loopKeyFrames) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getLoopEdges returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (std::set<keyframe::KeyFrame *>::iterator sit  = loopKeyFrames.begin(),
                                                send = loopKeyFrames.end();
                 sit != send;
                 sit++)
            {
                if ((*sit)->id < keyFrames[keyFrameIndex]->id)
                    continue;
                Eigen::Vector3f Owl{};
                if ((*sit)->getCameraCenter(Owl) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCameraCenter returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                glVertex3f(cameraCenter_world(0),
                           cameraCenter_world(1),
                           cameraCenter_world(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    bool activeMapIsImuInitialized{};
    if ((shouldDrawInertialGraph_in) &&
        p_activeMap->isImuInitialized(activeMapIsImuInitialized) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (shouldDrawInertialGraph_in && activeMapIsImuInitialized)
    {
        glLineWidth(graphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        /* Draw inertial links */
        for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
             keyFrameIndex++)
        {
            keyframe::KeyFrame       *p_drawnKeyFrame = keyFrames[keyFrameIndex];
            Eigen::Vector3f cameraCenter_world{};
            if (p_drawnKeyFrame->getCameraCenter(cameraCenter_world) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            keyframe::KeyFrame *p_next = p_drawnKeyFrame->p_nextKF;
            if (p_next)
            {
                Eigen::Vector3f Owp{};
                if (p_next->getCameraCenter(Owp) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCameraCenter returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                glVertex3f(cameraCenter_world(0),
                           cameraCenter_world(1),
                           cameraCenter_world(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    std::vector<map::Map *> maps{};
    if (p_atlas->getAllMaps(maps) != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (shouldDrawKeyFrames_in)
    {
        for (map::Map *p_map : maps)
        {
            if (p_map == p_activeMap)
                continue;

            std::vector<keyframe::KeyFrame *> mapKeyFrames{};
            if (p_map->getAllKeyFrames(mapKeyFrames) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            for (size_t keyFrameIndex = 0; keyFrameIndex < mapKeyFrames.size();
                 keyFrameIndex++)
            {
                keyframe::KeyFrame    *p_keyFrame = mapKeyFrames[keyFrameIndex];
                Sophus::SE3f keyFramePoseInverse2{};
                if (p_keyFrame->getPoseInverse(keyFramePoseInverse2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPoseInverse returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Matrix4f cameraPose_cameraToWorld =
                    keyFramePoseInverse2.matrix();
                /* The colour rows repeat for origin map ids past the end. */
                const std::size_t indexColor =
                    p_keyFrame->originMapId % std::size(frameColors);

                glPushMatrix();

                glMultMatrixf(cameraPose_cameraToWorld.data());

                keyframe::KeyFrame *p_parent2 = nullptr;
                if (mapKeyFrames[keyFrameIndex]->getParent(p_parent2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParent returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!p_parent2) /* It is the first KF in the map */
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

    return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;
}

} /* namespace map_drawer */
} /* namespace core */
} /* namespace vs_graphs */
