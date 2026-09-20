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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Atlas.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Settings.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace vs_graphs
{
namespace core
{

class Settings;

class MapDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings);

    void newParameterLoader(Settings *settings);

    Atlas *p_atlas;

    void drawMapPoints();
    void drawKeyFrames(const bool bDrawKF,
                       const bool bDrawGraph,
                       const bool bDrawInertialGraph,
                       const bool bDrawOptLba);
    void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void setCurrentCameraPose(const Sophus::SE3f &Tcw);
    void setReferenceKeyFrame(KeyFrame *pKF);
    void getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M,
                                      pangolin::OpenGlMatrix &MOw);

  private:
    bool parseViewerParamFile(cv::FileStorage &fSettings);

    float keyFrameSize;
    float keyFrameLineWidth;
    float graphLineWidth;
    float pointSize;
    float cameraSize;
    float cameraLineWidth;

    Sophus::SE3f cameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                 {0.8f, 0.4f, 1.0f},
                                 {1.0f, 0.2f, 0.4f},
                                 {0.6f, 0.0f, 1.0f},
                                 {1.0f, 1.0f, 0.0f},
                                 {0.0f, 1.0f, 1.0f}};
};

} // namespace core
} // namespace vs_graphs

#endif // MAPDRAWER_H
