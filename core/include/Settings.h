/**
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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H

// Flag to activate the measurement of time in each process (track,localmap,
// place recognition). #define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

namespace ORB_SLAM3
{

class System;

class Settings
{
  public:
    /*
     * Enum for the different camera types implemented
     */
    enum CameraType
    {
        PinHole       = 0,
        Rectified     = 1,
        KannalaBrandt = 2
    };

    /*
     * Delete default constructor
     */
    Settings() = delete;

    /*
     * Constructor from file
     */
    Settings(const std::string &configFile, const int &sensor);

    /*
     * Ostream operator overloading to dump settings to the terminal
     */
    friend std::ostream &operator<<(std::ostream &output, const Settings &s);

    /*
     * Getter methods
     */
    CameraType cameraType()
    {
        return cameraType_;
    }
    GeometricCamera *camera1()
    {
        return calibration1_;
    }
    GeometricCamera *camera2()
    {
        return calibration2_;
    }
    cv::Mat camera1DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion1_.size(),
                       1,
                       CV_32F,
                       vPinHoleDistorsion1_.data());
    }
    cv::Mat camera2DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion2_.size(),
                       1,
                       CV_32F,
                       vPinHoleDistorsion2_.data());
    }

    Sophus::SE3f Tlr()
    {
        return Tlr_;
    }
    double bf()
    {
        return bf_;
    }
    double b()
    {
        return b_;
    }
    double thDepth()
    {
        return thDepth_;
    }

    bool needToUndistort()
    {
        return bNeedToUndistort_;
    }

    cv::Size newImSize()
    {
        return newImSize_;
    }
    double fps()
    {
        return fps_;
    }
    bool rgb()
    {
        return bRGB_;
    }
    bool needToResize()
    {
        return bNeedToResize1_;
    }
    bool needToRectify()
    {
        return bNeedToRectify_;
    }

    // IMU parameters
    double accWalk()
    {
        return accWalk_;
    }
    double gyroWalk()
    {
        return gyroWalk_;
    }
    double noiseAcc()
    {
        return noiseAcc_;
    }
    double imuFrequency()
    {
        return imuFrequency_;
    }
    double imuThreshold()
    {
        return imuThreshold_;
    }
    double noiseGyro()
    {
        return noiseGyro_;
    }
    Sophus::SE3f Tbc()
    {
        return Tbc_;
    }
    bool insertKFsWhenLost()
    {
        return insertKFsWhenLost_;
    }

    double depthMapFactor()
    {
        return depthMapFactor_;
    }

    int nFeatures()
    {
        return nFeatures_;
    }
    int nLevels()
    {
        return nLevels_;
    }
    double initThFAST()
    {
        return initThFAST_;
    }
    double minThFAST()
    {
        return minThFAST_;
    }
    double scaleFactor()
    {
        return scaleFactor_;
    }

    double keyFrameSize()
    {
        return keyFrameSize_;
    }
    double keyFrameLineWidth()
    {
        return keyFrameLineWidth_;
    }
    double graphLineWidth()
    {
        return graphLineWidth_;
    }
    double pointSize()
    {
        return pointSize_;
    }
    double cameraSize()
    {
        return cameraSize_;
    }
    double cameraLineWidth()
    {
        return cameraLineWidth_;
    }
    double viewPointX()
    {
        return viewPointX_;
    }
    double viewPointY()
    {
        return viewPointY_;
    }
    double viewPointZ()
    {
        return viewPointZ_;
    }
    double viewPointF()
    {
        return viewPointF_;
    }
    double imageViewerScale()
    {
        return imageViewerScale_;
    }

    std::string atlasLoadFile()
    {
        return sLoadFrom_;
    }
    std::string atlasSaveFile()
    {
        return sSaveto_;
    }

    double thFarPoints()
    {
        return thFarPoints_;
    }

    cv::Mat M1l()
    {
        return M1l_;
    }
    cv::Mat M2l()
    {
        return M2l_;
    }
    cv::Mat M1r()
    {
        return M1r_;
    }
    cv::Mat M2r()
    {
        return M2r_;
    }

  private:
    template <typename T>
    T readParameter(cv::FileStorage   &fSettings,
                    const std::string &name,
                    bool              &found,
                    const bool         required = true)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name
                          << " required parameter does not exist, aborting..."
                          << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..."
                          << std::endl;
                found = false;
                return T();
            }
        }
        else
        {
            found = true;
            return (T)node;
        }
    }

    void readCamera1(cv::FileStorage &fSettings);
    void readCamera2(cv::FileStorage &fSettings);
    void readImageInfo(cv::FileStorage &fSettings);
    void readIMU(cv::FileStorage &fSettings);
    void readRGBD(cv::FileStorage &fSettings);
    void readORB(cv::FileStorage &fSettings);
    void readViewer(cv::FileStorage &fSettings);
    void readLoadAndSave(cv::FileStorage &fSettings);
    void readOtherParameters(cv::FileStorage &fSettings);

    void precomputeRectificationMaps();

    int        sensor_;
    CameraType cameraType_; // Camera type

    /*
     * Visual stuff
     */
    GeometricCamera    *calibration1_, *calibration2_; // Camera calibration
    GeometricCamera    *originalCalib1_, *originalCalib2_;
    std::vector<double> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

    cv::Size originalImSize_, newImSize_;
    double   fps_;
    bool     bRGB_;

    bool bNeedToUndistort_;
    bool bNeedToRectify_;
    bool bNeedToResize1_, bNeedToResize2_;

    Sophus::SE3f Tlr_;
    double       thDepth_;
    double       bf_, b_;

    /*
     * Rectification stuff
     */
    cv::Mat M1l_, M2l_;
    cv::Mat M1r_, M2r_;

    /*
     * Inertial stuff
     */
    double       noiseGyro_, noiseAcc_;
    double       gyroWalk_, accWalk_;
    double       imuFrequency_;
    double       imuThreshold_;
    Sophus::SE3f Tbc_;
    bool         insertKFsWhenLost_;

    /*
     * RGBD stuff
     */
    double depthMapFactor_;
    double nearThresh_, farThresh_;

    /*
     * ORB stuff
     */
    int    nFeatures_;
    double scaleFactor_;
    int    nLevels_;
    int    initThFAST_, minThFAST_;

    /*
     * Viewer stuff
     */
    double keyFrameSize_;
    double keyFrameLineWidth_;
    double graphLineWidth_;
    double pointSize_;
    double cameraSize_;
    double cameraLineWidth_;
    double viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
    double imageViewerScale_;

    /*
     * Save & load maps
     */
    std::string sLoadFrom_, sSaveto_;

    /*
     * Other stuff
     */
    double thFarPoints_;
};
}; // namespace ORB_SLAM3

#endif // ORB_SLAM3_SETTINGS_H
