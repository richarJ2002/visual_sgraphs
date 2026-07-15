/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "SemanticSegmentation.h"

namespace ORB_SLAM3
{
SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas)
{
    mpAtlas = pAtlas;

    // Get the system parameters
    sysParams = SystemParams::GetParams();

    // Set booleans according to the mode of operation
    mGeoRuns = !(sysParams->general.mode_of_operation ==
                 SystemParams::general::ModeOfOperation::SEM);
}

void SemanticSegmentation::Run()
{
    while (true)
    {
        // Check if there are new KeyFrames in the buffer
        if (segmentedImageBuffer.empty())
        {
            usleep(3000);
            continue;
        }

        // Lock keyframes
        mMutexNewKFs.lock();

        // Retrieve oldest keyframe
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> segImgTuple =
            segmentedImageBuffer.front();

        // Remove keyframe from front
        segmentedImageBuffer.pop_front();

        // Unlock keyframe
        mMutexNewKFs.unlock();

        // get the point cloud from the respective keyframe via the atlas -
        // ignore it if KF doesn't exist
        KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));

        // If keyframe is bad continue
        if (thisKF == nullptr || thisKF->isBad())
        {
            continue;
        }

        // Extract point cloud from keyframe
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud =
            thisKF->getCurrentFramePointCloud();

        // If no point cloud in keyframe, skip to next frame
        if (thisKFPointCloud == nullptr)
        {
            std::cout << "SemSeg: skipping KF ID: " << thisKF->mnId
                      << ". Missing pointcloud..." << std::endl;
            exit(1);
            continue;
        }

        /* Extract the segmentation probabilities from the image */
        pcl::PCLPointCloud2::Ptr pclPc2SegPrb = std::get<2>(segImgTuple);

        /* Extract the segmentation uncertainties from the image */
        cv::Mat segImgUncertainity = std::get<1>(segImgTuple);
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs;

        /*!
         * Seperate points into classes and filtering based on thresholds.
         *
         * clsCloudPtrs contains the points from the segmented point cloud
         * seperated into a list of point clouds with the index of the list
         * representing the semantic type of each point.
         */
        threshSeparatePointCloud(pclPc2SegPrb,
                                 segImgUncertainity,
                                 clsCloudPtrs,
                                 thisKFPointCloud);

        /*!
         * clear pointclouds as they are no longer needed and consume
         * significant memory. also
         */
        thisKF->clearPointCloud();

        /*!
         * Clear pointclouds from the keyframes that might have been skipped
         * always keep last few keyframes as there can be minor misordering in
         * keyframe processing.
         */
        int buffer = 5;
        if (thisKF->mnId - mLastProcessedKeyFrameId > buffer)
        {
            for (unsigned long int i = mLastProcessedKeyFrameId + 1;
                 i < thisKF->mnId - buffer;
                 i++)
            {
                KeyFrame *pKF = mpAtlas->GetKeyFrameById(i);
                if (pKF != nullptr &&
                    pKF->getCurrentFramePointCloud() != nullptr)
                {
                    pKF->clearPointCloud();
                    pKF->clearClsClouds();
                }
            }
            mLastProcessedKeyFrameId = thisKF->mnId - buffer;
        }

        /*!
         * Extract planes from segmented point cloud. (Does not define the type
         * the plane is)
         */
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>>
            clsPlanes = getPlanesFromClassClouds(clsCloudPtrs);

        /* Set the class specific point clouds to the keyframe */
        thisKF->setCurrentClsCloudPtrs(clsCloudPtrs);

        /* Add the planes to Atlas */
        updatePlaneData(thisKF, clsPlanes);
    }
}

void SemanticSegmentation::AddSegmentedFrameToBuffer(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
{
    unique_lock<std::mutex> lock(mMutexNewKFs);
    segmentedImageBuffer.push_back(*tuple);
}

std::list<std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr>>
    SemanticSegmentation::GetSegmentedFrameBuffer()
{
    return segmentedImageBuffer;
}

void SemanticSegmentation::threshSeparatePointCloud(
    pcl::PCLPointCloud2::Ptr                              pclPc2SegPrb,
    cv::Mat                                              &segImgUncertainity,
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr         &thisKFPointCloud)
{
    /* Extract parameters on thresholds */
    const uint8_t confidenceThresh = sysParams->sem_seg.conf_thresh * 255;
    const float   probThresh       = sysParams->sem_seg.prob_thresh;
    const float   distanceThreshNear =
        sysParams->pointcloud.distance_thresh.first;
    const float distanceThreshFar =
        sysParams->pointcloud.distance_thresh.second;

    /* Parse the PointCloud2 message */
    const int width      = pclPc2SegPrb->width;
    const int numPoints  = width * pclPc2SegPrb->height;
    const int pointStep  = pclPc2SegPrb->point_step;
    const int numClasses = pointStep / bytesPerClassProb;

    /* Clear the seperated point cloud vector `clsCloudPtrs` */
    clsCloudPtrs.clear();
    clsCloudPtrs.reserve(numClasses);

    /*!
     * For each semantic class, add an instance to the output clsCloudPtrs list.
     * This way the pointclouds can be put into there respective class.
     */
    for (int i = 0; i < numClasses; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pointCloud->is_dense = false;
        pointCloud->height   = 1;
        clsCloudPtrs.push_back(pointCloud);
    }

    /* Extract the data from the inputted segmented point cloud */
    const uint8_t *data = pclPc2SegPrb->data.data();

    /*!
     * Iterate through the number of classes.
     *  j -> class index
     *  i -> flattened pixel index
     */
    for (int j = 0; j < numClasses; j++)
    {
        /* Iterate through the number of points */
        for (int i = 0; i < numPoints; i++)
        {
            /*!
             * Initilize variable containing probability point i belongs to
             * class j
             */
            float probability;

            /* Extract probabliilty that point i belongs to class j */
            std::memcpy(&probability,
                        data + pointStep * i + bytesPerClassProb * j +
                            pclPc2SegPrb->fields[0].offset,
                        bytesPerClassProb);

            /*!
             * Apply thresholding and track confidence (complement of
             * uncertainty). This is to check that the probability of the
             * semantic.
             */
            if (probability >= probThresh)
            {
                // /* Inject coordinates as a point to respective point cloud */
                /* Initialize point to be filtered into class point cloud */
                pcl::PointXYZRGBA point;

                /* Find the pixel index of the point in the image */
                point.y = static_cast<int>(i / width);
                point.x = i % width;

                /* Extract the original point from the keyframe point cloud */
                const pcl::PointXYZRGB origPoint =
                    thisKFPointCloud->at(point.x, point.y);

                /* If the original point has invalid data, skip data point */
                if (!pcl::isFinite(origPoint))
                {
                    continue;
                }

                /* Extract the rgb uncertainty from the image */
                cv::Vec3b vec =
                    segImgUncertainity.at<cv::Vec3b>(point.y, point.x);

                /*!
                 * Convert the rgb uncertainty of the pixel to a single value
                 * and store in the alpha channel.
                 */
                point.a =
                    255 - static_cast<int>(0.299 * vec[2] + 0.587 * vec[1] +
                                           0.114 * vec[0]);

                /*!
                 * Exclude the points with low confidence that segmentation was
                 * correct.
                 */
                if (point.a < confidenceThresh)
                {
                    continue;
                }

                /* Assign the XYZ and RGB values to the surviving point */
                point.x = origPoint.x;
                point.y = origPoint.y;
                point.z = origPoint.z;
                point.r = origPoint.r;
                point.g = origPoint.g;
                point.b = origPoint.b;

                /*!
                 * Confidence as the squared inverse depth - interpolated
                 * between near and far thresholds confidence = 255 for near, 45
                 * for far, and interpolated according to squared distance
                 */
                if (point.z < distanceThreshNear)
                {
                    point.a = 255;
                }
                else if (point.z > distanceThreshFar)
                {
                    point.a = 45;
                }
                else
                {
                    point.a =
                        255 - static_cast<int>(
                                  210 * sqrt((point.z - distanceThreshNear) /
                                             (distanceThreshFar -
                                              distanceThreshNear)));
                }

                /* Add the point to the respective class specific point cloud */
                clsCloudPtrs[j]->push_back(point);
            }
        }
    }

    /* Specify size/width and header for each class specific point cloud */
    for (int i = 0; i < numClasses; i++)
    {
        clsCloudPtrs[i]->width  = clsCloudPtrs[i]->size();
        clsCloudPtrs[i]->header = pclPc2SegPrb->header;
    }
}

std::vector<std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
    SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs)
{
    std::vector<std::vector<
        std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
        clsPlanes;

    // downsample/filter the pointcloud and extract planes
    for (size_t i = 0; i < clsCloudPtrs.size(); i++)
    {
        // [TODO?] - Perhaps consider points in order of confidence instead of
        // downsampling Downsample the given pointcloud after filtering based on
        // distance

        /* Init variable for the filtered point cloud */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;

        /*!
         * Filter points based on depth from sensor.
         *
         * @note:       Parameter for min and max distance are defined as
         *              default values in:
         *              `visual_sgraphs/core/include/Types/SystemParams.h`
         */
        filteredCloud =
            Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(clsCloudPtrs[i]);

        /* Downsample points into grid based on points within voxel grid */
        filteredCloud = Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
            filteredCloud,
            sysParams->sem_seg.pointcloud.downsample.leaf_size,
            sysParams->sem_seg.pointcloud.downsample.min_points_per_voxel);

        /* Remove points that are statically isolated from neighbors */
        filteredCloud = Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(
            filteredCloud,
            sysParams->sem_seg.pointcloud.outlier_removal.std_threshold,
            sysParams->sem_seg.pointcloud.outlier_removal.mean_threshold);

        /* copy the filtered cloud for later storage into the keyframe */
        pcl::copyPointCloud(*filteredCloud, *clsCloudPtrs[i]);

        /* Initialize object to contain extracted point clouds */
        std::vector<
            std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
            extractedPlanes;

        /*!
         * Extract planes from filtered point cloud if the number of points is
         * greater than a threshold. This parameter is set in
         * `system_params.yaml`
         */
        if (filteredCloud->points.size() > sysParams->seg.pointclouds_thresh)
        {
            extractedPlanes =
                Utils::ransacPlaneFitting<pcl::PointXYZRGBA,
                                          pcl::WeightedSACSegmentation>(
                    filteredCloud);
        }
        clsPlanes.push_back(extractedPlanes);
    }
    return clsPlanes;
}

void SemanticSegmentation::updatePlaneData(
    KeyFrame                                             *pKF,
    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                      Eigen::Vector4d>>> &clsPlanes)
{
    /* Iterate through each semantic class of planes */
    for (size_t clsId = 0; clsId < clsPlanes.size(); clsId++)
    {
        /* Iterate through each plane in the semantic group */
        for (const auto &planePoint : clsPlanes[clsId])
        {
            /* Get the plane equation of the plane */
            Eigen::Vector4d estimatedPlane = planePoint.second;

            /* Initiate a 3D plane object from the detected plane */
            g2o::Plane3D detectedPlane(estimatedPlane);

            /* Convert the given plane to global coordinates */
            g2o::Plane3D globalEquation = Utils::applyPoseToPlane(
                pKF->GetPoseInverse().matrix().cast<double>(),
                detectedPlane);

            /* Extract the point cloud assoicated with the plane */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud =
                planePoint.first;

            /* Initialize the confidence vector */
            std::vector<double> confidences;

            /* Extract the confidences from each point in the point cloud */
            for (size_t i = 0; i < planeCloud->size(); i++)
            {
                confidences.push_back(
                    static_cast<int>(planeCloud->points[i].a) / 255.0);
            }

            /* Initialize the confidence variable */
            double conf = 0.0;

            /*!
             * Find the average confidence across all the points.
             *
             * @note:       There are two different ways to find the confidences
             *              with a summary below:
             *
             *                  use softmin when dealing with semantic
             *                  confidences double conf =
             *                  Utils::calcSoftMin(confidences);
             *
             *                  use average when dealing with geometric (in this
             *                  case depth) confidences
             */
            if (!confidences.empty())
            {
                conf = std::accumulate(confidences.begin(),
                                       confidences.end(),
                                       0.0) /
                       confidences.size();
            }

            /* Initialize a temporary global point cloud which is empty */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalPlaneCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>);

            /* Copy the plane cloud to the global point cloud */
            pcl::copyPointCloud(*planeCloud, *globalPlaneCloud);

            /* Transform globalPlaneCloud with the transform of the keyframe */
            pcl::transformPointCloud(
                *globalPlaneCloud,
                *globalPlaneCloud,
                pKF->GetPoseInverse().matrix().cast<float>());

            /* Get the semantic type of the observation */
            ORB_SLAM3::Plane::planeVariant semanticType =
                Utils::getPlaneTypeFromClassId(clsId);

            /* Check if we need to add the plane to the map or not */
            int matchedPlaneId = Utils::associatePlanes(
                mpAtlas->GetAllPlanes(),
                detectedPlane,
                globalPlaneCloud,
                pKF->GetPose().matrix().cast<double>(),
                semanticType,
                sysParams->seg.plane_association.ominus_thresh);

            /* If no mapped plane is associated with current plane*/
            if (matchedPlaneId == -1)
            {
                /*!
                 * If semantic segmentation is running independently, create
                 * a new plane to add to the map.
                 *
                 * This parameter is controlled by general.mode_of_operation in
                 * system_params.yaml.
                 */
                if (!mGeoRuns)
                {
                    /* Create new plant to add to map */
                    ORB_SLAM3::Plane *newMapPlane =
                        GeoSemHelpers::createMapPlane(mpAtlas,
                                                      pKF,
                                                      detectedPlane,
                                                      planeCloud,
                                                      semanticType,
                                                      conf);

                    /* Update the semantic of the plane */
                    updatePlaneSemantics(newMapPlane->getId(), clsId, conf);
                }
            }
            else
            {
                if (!mGeoRuns)
                    GeoSemHelpers::updateMapPlane(mpAtlas,
                                                  pKF,
                                                  detectedPlane,
                                                  planeCloud,
                                                  matchedPlaneId,
                                                  semanticType,
                                                  conf);
                else
                {
                    pcl::transformPointCloud(
                        *planeCloud,
                        *planeCloud,
                        pKF->GetPoseInverse().matrix().cast<float>());
                    ORB_SLAM3::Plane *matchedPlane =
                        mpAtlas->GetPlaneById(matchedPlaneId);
                    // Add the plane cloud to the matched plane
                    if (!planeCloud->empty())
                        matchedPlane->setMapClouds(planeCloud);
                }

                // Cast a vote for the plane semantics
                updatePlaneSemantics(matchedPlaneId, clsId, conf);
            }
        }
    }
}

void SemanticSegmentation::updatePlaneSemantics(int    planeId,
                                                int    clsId,
                                                double confidence)
{
    // retrieve the plane from the map
    Plane *matchedPlane = mpAtlas->GetPlaneById(planeId);

    // plane type compatible with the Plane class
    ORB_SLAM3::Plane::planeVariant planeType =
        Utils::getPlaneTypeFromClassId(clsId);

    // cast a vote for the plane semantics
    matchedPlane->castWeightedVote(planeType, confidence);
}
} // namespace ORB_SLAM3