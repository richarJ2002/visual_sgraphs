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

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "Utils/Converter/objects/Converter.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

bool Tracking::parseCamParamFile(cv::FileStorage &settings_in)
{
    distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool isParameterMissing = false;

    string cameraName = settings_in["Camera.type"];
    if (cameraName == "PinHole")
    {
        float fx   = 0.0F;
        float fy   = 0.0F;
        float cx   = 0.0F;
        float cy   = 0.0F;
        imageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = settings_in["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        // Distortion parameters
        node = settings_in["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(0) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(1) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.p1"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(2) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.p1 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.p2"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(3) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.p2 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.resize(5);
            distortionCoefficients.at<float>(4) = node.real();
        }

        node = settings_in["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            imageScale = node.real();
        }

        if (isParameterMissing)
        {
            return false;
        }

        if (imageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * imageScale;
            fy = fy * imageScale;
            cx = cx * imageScale;
            cy = cy * imageScale;
        }

        vector<float> cameraCalibrations{fx, fy, cx, cy};

        p_camera = new camera_models::pinhole::Pinhole(cameraCalibrations);

        p_camera = p_atlas->addCamera(p_camera);

        std::cout << "- Camera: camera_models::Pinhole" << std::endl;
        std::cout << "- Image scale: " << imageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << distortionCoefficients.at<float>(0)
                  << std::endl;
        std::cout << "- k2: " << distortionCoefficients.at<float>(1)
                  << std::endl;

        std::cout << "- p1: " << distortionCoefficients.at<float>(2)
                  << std::endl;
        std::cout << "- p2: " << distortionCoefficients.at<float>(3)
                  << std::endl;

        if (distortionCoefficients.rows == 5)
            std::cout << "- k3: " << distortionCoefficients.at<float>(4)
                      << std::endl;

        calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
        calibrationMatrix.at<float>(0, 0) = fx;
        calibrationMatrix.at<float>(1, 1) = fy;
        calibrationMatrix.at<float>(0, 2) = cx;
        calibrationMatrix.at<float>(1, 2) = cy;

        calibrationMatrixEigen.setIdentity();
        calibrationMatrixEigen(0, 0) = fx;
        calibrationMatrixEigen(1, 1) = fy;
        calibrationMatrixEigen(0, 2) = cx;
        calibrationMatrixEigen(1, 2) = cy;
    }
    else if (cameraName == "camera_models::KannalaBrandt8")
    {
        float fx   = 0.0F;
        float fy   = 0.0F;
        float cx   = 0.0F;
        float cy   = 0.0F;
        float k1   = 0.0F;
        float k2   = 0.0F;
        float k3   = 0.0F;
        float k4   = 0.0F;
        imageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = settings_in["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }
        node = settings_in["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        // Distortion parameters
        node = settings_in["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }
        node = settings_in["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k3 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.k4"];
        if (!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k4 parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }

        node = settings_in["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            imageScale = node.real();
        }

        if (!isParameterMissing)
        {
            if (imageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * imageScale;
                fy = fy * imageScale;
                cx = cx * imageScale;
                cy = cy * imageScale;
            }

            vector<float> cameraCalibrations{fx, fy, cx, cy, k1, k2, k3, k4};
            p_camera = new camera_models::kannalabrandt8::KannalaBrandt8(
                cameraCalibrations);
            p_camera = p_atlas->addCamera(p_camera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << imageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
            calibrationMatrix.at<float>(0, 0) = fx;
            calibrationMatrix.at<float>(1, 1) = fy;
            calibrationMatrix.at<float>(0, 2) = cx;
            calibrationMatrix.at<float>(1, 2) = cy;

            calibrationMatrixEigen.setIdentity();
            calibrationMatrixEigen(0, 0) = fx;
            calibrationMatrixEigen(1, 1) = fy;
            calibrationMatrixEigen(0, 2) = cx;
            calibrationMatrixEigen(1, 2) = cy;
        }

        if (sensor == System::STEREO || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD)
        {
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = settings_in["Camera2.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }
            node = settings_in["Camera2.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            node = settings_in["Camera2.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            node = settings_in["Camera2.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            // Distortion parameters
            node = settings_in["Camera2.k1"];
            if (!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }
            node = settings_in["Camera2.k2"];
            if (!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            node = settings_in["Camera2.k3"];
            if (!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            node = settings_in["Camera2.k4"];
            if (!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            int leftLappingBegin = -1;
            int leftLappingEnd   = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd   = -1;

            node = settings_in["Camera.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera.lappingBegin not correctly defined"
                    << std::endl;
            }
            node = settings_in["Camera.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined"
                          << std::endl;
            }
            node = settings_in["Camera2.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera2.lappingBegin not correctly defined"
                    << std::endl;
            }
            node = settings_in["Camera2.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined"
                          << std::endl;
            }

            node = settings_in["Tlr"];
            cv::Mat cvTlr;
            if (!node.empty())
            {
                cvTlr = node.mat();
                if (cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr
                        << "*Tlr matrix have to be a 3x4 transformation matrix*"
                        << std::endl;
                    isParameterMissing = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                isParameterMissing = true;
            }

            if (!isParameterMissing)
            {
                if (imageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * imageScale;
                    fy = fy * imageScale;
                    cx = cx * imageScale;
                    cy = cy * imageScale;

                    leftLappingBegin  = leftLappingBegin * imageScale;
                    leftLappingEnd    = leftLappingEnd * imageScale;
                    rightLappingBegin = rightLappingBegin * imageScale;
                    rightLappingEnd   = rightLappingEnd * imageScale;
                }

                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera)
                    ->lappingArea[0] = leftLappingBegin;
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera)
                    ->lappingArea[1] = leftLappingEnd;

                p_frameDrawer->shouldDrawBothImages = true;

                vector<float>
                    cameraCalibration2{fx, fy, cx, cy, k1, k2, k3, k4};
                p_camera2 = new camera_models::kannalabrandt8::KannalaBrandt8(
                    cameraCalibration2);
                p_camera2 = p_atlas->addCamera(p_camera2);

                Sophus::SE3<float> sophus{};
                if (utils::converter::Converter::toSophus(cvTlr, sophus) !=
                    utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
                {
                    // toSophus cannot fail; continue as before.
                }
                poseTlr = sophus;

                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera2)
                    ->lappingArea[0] = rightLappingBegin;
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_camera2)
                    ->lappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", "
                          << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << imageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", "
                          << rightLappingEnd << std::endl;
            }
        }

        if (isParameterMissing)
        {
            return false;
        }
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr
            << "Check an example configuration file with the desired sensor"
            << std::endl;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        cv::FileNode node = settings_in["Camera.bf"];
        if (!node.empty() && node.isReal())
        {
            mbf = node.real();
            if (imageScale != 1.f)
            {
                mbf *= imageScale;
            }
        }
        else
        {
            std::cerr
                << "*Camera.bf parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }
    }

    float fps = settings_in["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    minFrames = 0;
    maxFrames = fps;

    cout << "- fps: " << fps << endl;

    int rgbCount = settings_in["Camera.RGB"];
    isRgbEnabled = rgbCount;

    if (isRgbEnabled)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        float fx{};
        if (p_camera->getParameter(0, fx) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            // getParameter cannot fail; continue as before.
        }
        cv::FileNode node = settings_in["ThDepth"];
        if (!node.empty() && node.isReal())
        {
            depthThreshold = node.real();
            depthThreshold = mbf * depthThreshold / fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << depthThreshold
                 << endl;
        }
        else
        {
            std::cerr
                << "*ThDepth parameter doesn't exist or is not a real number*"
                << std::endl;
            isParameterMissing = true;
        }
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        cv::FileNode node = settings_in["DepthMapFactor"];
        if (!node.empty() && node.isReal())
        {
            depthMapFactor = node.real();
            if (fabs(depthMapFactor) < 1e-5)
                depthMapFactor = 1;
            else
                depthMapFactor = 1.0f / depthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a "
                         "real number*"
                      << std::endl;
            isParameterMissing = true;
        }
    }

    if (isParameterMissing)
    {
        return false;
    }

    return true;
}

} // namespace core
} // namespace vs_graphs
