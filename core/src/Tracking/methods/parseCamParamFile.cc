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
 * @file            parseCamParamFile.cc
 *
 * @brief           Implements Tracking::parseCamParamFile(), declared in
 *                  Tracking.h.
 */

#include "Tracking.h"

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "FrameDrawer.h"
#include "System.h"
#include "Utils/Converter/objects/Converter.h"

#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::parseCamParamFile(cv::FileStorage &settings_in,
                                           bool            &isParsed_out)
{
    distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    std::cout << std::endl << "Camera Parameters: " << std::endl;
    bool isParameterMissing = false;

    std::string cameraName = settings_in["Camera.type"];
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
            isParsed_out = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        if (imageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * imageScale;
            fy = fy * imageScale;
            cx = cx * imageScale;
            cy = cy * imageScale;
        }

        std::vector<float> cameraCalibrations{fx, fy, cx, cy};

        p_parsedCamera = std::make_unique<camera_models::pinhole::Pinhole>(
            cameraCalibrations);

        camera_models::geometriccamera::GeometricCamera *p_atlasCamera =
            nullptr;
        if (p_atlas->addCamera(p_parsedCamera.get(), p_atlasCamera) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addCamera returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_camera = p_atlasCamera;

        std::cout << "- Camera: Pinhole" << std::endl;
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
    else if (cameraName == "KannalaBrandt8")
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

            std::vector<float>
                cameraCalibrations{fx, fy, cx, cy, k1, k2, k3, k4};
            p_parsedCamera =
                std::make_unique<camera_models::kannalabrandt8::KannalaBrandt8>(
                    cameraCalibrations);
            camera_models::geometriccamera::GeometricCamera *p_atlasCamera2 =
                nullptr;
            if (p_atlas->addCamera(p_parsedCamera.get(), p_atlasCamera2) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addCamera returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_camera = p_atlasCamera2;
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
            cv::FileNode camera2Node = settings_in["Camera2.fx"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                fx = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }
            camera2Node = settings_in["Camera2.fy"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                fy = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            camera2Node = settings_in["Camera2.cx"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                cx = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            camera2Node = settings_in["Camera2.cy"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                cy = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            // Distortion parameters
            camera2Node = settings_in["Camera2.k1"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                k1 = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }
            camera2Node = settings_in["Camera2.k2"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                k2 = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            camera2Node = settings_in["Camera2.k3"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                k3 = camera2Node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                isParameterMissing = true;
            }

            camera2Node = settings_in["Camera2.k4"];
            if (!camera2Node.empty() && camera2Node.isReal())
            {
                k4 = camera2Node.real();
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

            camera2Node = settings_in["Camera.lappingBegin"];
            if (!camera2Node.empty() && camera2Node.isInt())
            {
                leftLappingBegin = camera2Node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera.lappingBegin not correctly defined"
                    << std::endl;
            }
            camera2Node = settings_in["Camera.lappingEnd"];
            if (!camera2Node.empty() && camera2Node.isInt())
            {
                leftLappingEnd = camera2Node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined"
                          << std::endl;
            }
            camera2Node = settings_in["Camera2.lappingBegin"];
            if (!camera2Node.empty() && camera2Node.isInt())
            {
                rightLappingBegin = camera2Node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera2.lappingBegin not correctly defined"
                    << std::endl;
            }
            camera2Node = settings_in["Camera2.lappingEnd"];
            if (!camera2Node.empty() && camera2Node.isInt())
            {
                rightLappingEnd = camera2Node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined"
                          << std::endl;
            }

            camera2Node = settings_in["Tlr"];
            cv::Mat cvTlr;
            if (!camera2Node.empty())
            {
                cvTlr = camera2Node.mat();
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

                std::vector<float>
                    cameraCalibration2{fx, fy, cx, cy, k1, k2, k3, k4};
                p_parsedCamera2 = std::make_unique<
                    camera_models::kannalabrandt8::KannalaBrandt8>(
                    cameraCalibration2);
                camera_models::geometriccamera::GeometricCamera
                    *p_atlasCamera3 = nullptr;
                if (p_atlas->addCamera(p_parsedCamera2.get(), p_atlasCamera3) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addCamera returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_camera2 = p_atlasCamera3;

                Sophus::SE3<float> sophus{};
                if (utils::converter::Converter::toSophus(cvTlr, sophus) !=
                    utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: toSophus returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
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
            isParsed_out = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor: '" << cameraName << "'*"
                  << std::endl;
        std::cerr
            << "Check an example configuration file with the desired sensor"
            << std::endl;
        /* No camera was built, so the steps below cannot run. */
        isParsed_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
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

    std::cout << "- fps: " << fps << std::endl;

    int rgbCount = settings_in["Camera.RGB"];
    isRgbEnabled = rgbCount;

    if (isRgbEnabled)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        float fx{};
        if (p_camera->getParameter(0, fx) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        cv::FileNode node = settings_in["ThDepth"];
        if (!node.empty() && node.isReal())
        {
            depthThreshold = node.real();
            depthThreshold = mbf * depthThreshold / fx;
            std::cout << std::endl
                      << "Depth Threshold (Close/Far Points): "
                      << depthThreshold << std::endl;
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
            if (std::fabs(depthMapFactor) < 1e-5)
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
        isParsed_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    isParsed_out = true;
    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
