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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <thread>
#include <unordered_set>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ORB_SLAM3
{

namespace
{
/*!
 * @brief Describes the strongest spatially connected part of a wall cloud.
 */
struct WallComponentSupport
{
    /*! @brief Source-cloud indices forming the largest component. */
    std::vector<int> pointIndices;
    /*! @brief Number of finite points considered by clustering. */
    std::size_t      finitePointCount = 0U;
    /*! @brief Fraction of finite points in the largest component. */
    double           componentRatio = 0.0;
};

/*!
 * @brief Finds the largest Euclidean component of a proposed wall plane.
 *
 *        The returned indices refer to the input cloud, allowing the same
 *        support to be selected in both camera and map frames. Invalid depth
 *        samples are excluded before building the search tree.
 *
 * @param[in] p_wallCloud_in
 *            Proposed wall support cloud.
 * @param[in] clusterTolerance_m_in
 *            Maximum Euclidean neighbour separation in metres.
 *
 * @return Largest connected component and its support statistics.
 */
WallComponentSupport findLargestWallComponent(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const double                                        clusterTolerance_m_in)
{
    WallComponentSupport support;

    if (p_wallCloud_in == nullptr || p_wallCloud_in->empty() ||
        !std::isfinite(clusterTolerance_m_in) || clusterTolerance_m_in <= 0.0)
    {
        return support;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_finiteWallCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::vector<int> finiteSourceIndices;
    p_finiteWallCloud->reserve(p_wallCloud_in->size());
    finiteSourceIndices.reserve(p_wallCloud_in->size());

    for (std::size_t pointIndex = 0U; pointIndex < p_wallCloud_in->size();
         pointIndex++)
    {
        if (!pcl::isFinite(p_wallCloud_in->points[pointIndex]))
        {
            continue;
        }

        p_finiteWallCloud->push_back(p_wallCloud_in->points[pointIndex]);
        finiteSourceIndices.push_back(static_cast<int>(pointIndex));
    }

    support.finitePointCount = p_finiteWallCloud->size();

    if (p_finiteWallCloud->empty())
    {
        return support;
    }

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr p_searchTree(
        new pcl::search::KdTree<pcl::PointXYZRGBA>);
    p_searchTree->setInputCloud(p_finiteWallCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance_m_in);
    clusterExtraction.setMinClusterSize(1);
    clusterExtraction.setMaxClusterSize(
        static_cast<int>(p_finiteWallCloud->size()));
    clusterExtraction.setSearchMethod(p_searchTree);
    clusterExtraction.setInputCloud(p_finiteWallCloud);

    std::vector<pcl::PointIndices> connectedComponents;
    clusterExtraction.extract(connectedComponents);

    if (connectedComponents.empty())
    {
        return support;
    }

    const auto largestComponentIterator = std::max_element(
        connectedComponents.begin(),
        connectedComponents.end(),
        [](const pcl::PointIndices &leftComponent,
           const pcl::PointIndices &rightComponent) {
            return leftComponent.indices.size() < rightComponent.indices.size();
        });

    support.pointIndices.reserve(largestComponentIterator->indices.size());

    for (const int finitePointIndex : largestComponentIterator->indices)
    {
        if (finitePointIndex < 0 ||
            static_cast<std::size_t>(finitePointIndex) >=
                finiteSourceIndices.size())
        {
            continue;
        }

        support.pointIndices.push_back(
            finiteSourceIndices[static_cast<std::size_t>(finitePointIndex)]);
    }

    support.componentRatio = static_cast<double>(support.pointIndices.size()) /
                             static_cast<double>(support.finitePointCount);

    return support;
}
} // namespace

SemanticSegmentation::SemanticSegmentation(Atlas *pAtlas)
{
    /* Store atlas object address */
    mpAtlas = pAtlas;

    /* Get the system parameters */
    sysParams = SystemParams::GetParams();

    /* Set the booleans according to the mode of operation */
    mGeoRuns = !(sysParams->general.mode_of_operation ==
                 SystemParams::general::ModeOfOperation::SEM);
}

void SemanticSegmentation::Run()
{
    /* Output message to indicate that semantic segmentation is starting */
    std::cout << "[SemSeg] Semantic Segmentation Started" << std::endl;

    /* Spin thread */
    while (true)
    {
        /* Graceful shutdown on System::Shutdown() */
        if (CheckFinish())
        {
            break;
        }

        /* Check if there are new segmented image in the buffer */
        if (segmentedImageBuffer.empty())
        {
            usleep(3000);
            continue;
        }

        /* Lock keyframes */
        mMutexNewKFs.lock();

        /* Retrieve oldest keyframe */
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> segImgTuple =
            segmentedImageBuffer.front();

        /* Remove keyframe from front */
        segmentedImageBuffer.pop_front();

        /* Unlock keyframe */
        mMutexNewKFs.unlock();

        /*!
         * Get the point cloud from the respective keyframe via the atlas -
         * ignore it if KF doesn't exist.
         */
        KeyFrame *thisKF = mpAtlas->GetKeyFrameById(std::get<0>(segImgTuple));

        /* If keyframe is bad continue */
        if (thisKF == nullptr || thisKF->isBad())
        {
            continue;
        }

        /* Extract point cloud from keyframe */
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud =
            thisKF->getCurrentFramePointCloud();

        /* If no point cloud in keyframe, skip to next frame */
        if (thisKFPointCloud == nullptr)
        {
            std::cerr << "[SemSeg] Skipping keyframe " << thisKF->mnId
                      << ": the RGB-D point cloud is unavailable." << std::endl;
            continue;
        }

        /* Extract the segmentation probabilities from the image */
        pcl::PCLPointCloud2::Ptr pclPc2SegPrb = std::get<2>(segImgTuple);

        /* Extract the segmentation uncertainties from the image */
        cv::Mat segImgUncertainity = std::get<1>(segImgTuple);

        /* Init an object of point clouds for seperated classes */
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
         * Clear point-cloud data from older keyframes that were skipped by this
         * processing stage.
         *
         * @note        Keep the most recent few keyframes intact because
         *              keyframes may be processed slightly out of order. Once a
         *              keyframe is older than the buffer window, its raw and
         *              classified point-cloud data are no longer needed and can
         *              be released to reduce memory usage.
         */
        if (thisKF->mnId - mLastProcessedKeyFrameId > 5)
        {
            for (unsigned long int i = mLastProcessedKeyFrameId + 1;
                 i < thisKF->mnId - 5;
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
            mLastProcessedKeyFrameId = thisKF->mnId - 5;
        }

        /* ------------------------------------------------------------------ *
         * PLANE EXTRACTION
         * ------------------------------------------------------------------ */

        /*!
         * Extract planes from segmented point cloud.
         *
         * @note        Does not define the semantic type the plane is
         */
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>>
            clsPlanes = getPlanesFromClassClouds(clsCloudPtrs);

        /* Set the class specific point clouds to the keyframe */
        thisKF->setCurrentClsCloudPtrs(clsCloudPtrs);

        {
            /*!
             * Plane association changes map-owned geometry and observation
             * edges. Serialize that short mutation with loop-closing's
             * semantic transfer; point-cloud inference remains outside the
             * transaction so it cannot unnecessarily delay a map merge.
             */
            std::unique_lock<std::mutex> semanticUpdateLock =
                mpAtlas->acquireSemanticUpdateLock();

            /*!
             * Plane extraction runs outside the semantic transaction. A map
             * merge may therefore invalidate the source keyframe or transfer
             * it away from the Atlas current map while inference is running.
             * Revalidate the source only after acquiring the transaction lock
             * so stale output cannot recreate observations in the merged map.
             */
            Map *p_currentMap = mpAtlas->GetCurrentMap();

            if (thisKF == nullptr || thisKF->isBad() ||
                thisKF->GetMap() != p_currentMap)
            {
                std::cerr
                    << "[SemSeg] Discarding stale segmentation output for "
                       "keyframe "
                    << std::get<0>(segImgTuple) << " after a map change."
                    << std::endl;
                continue;
            }

            /* Add the planes to Atlas. */
            updatePlaneData(thisKF, clsPlanes);
        }
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

    /* Downsample/filter the pointcloud and extract planes */
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
         * @note        Parameter for min and max distance are defined as
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

        /*!
         * Filtering removes arbitrary points, so the result is no longer an
         * organized image cloud. Normalize its metadata before copying it;
         * retaining the input image width makes PCL infer an invalid height
         * and emits a warning on every semantic update.
         */
        filteredCloud->width  = filteredCloud->size();
        filteredCloud->height = 1;

        /* Skip point clouds which are empty or have incalid width/height */
        if (filteredCloud->width == 0 || filteredCloud->height == 0 ||
            filteredCloud->empty())
        {
            continue;
        }

        /* Copy the filtered cloud for later storage into the keyframe */
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

            /*!
             * Associate the observation using the global plane equation and
             * global point cloud.
             *
             * @note        Performing the complete comparison in the global
             *              frame avoids inconsistencies between plane
             *              equations, centroids and point clouds.
             */
            int matchedPlaneId = Utils::associatePlanes(
                mpAtlas->GetAllPlanes(),
                globalEquation,
                globalPlaneCloud,
                Eigen::Matrix4d::Identity(),
                semanticType,
                sysParams->seg.plane_association.ominus_thresh,
                -1.0F,
                pKF->GetCameraCenter().cast<double>());

            /*!
             * If no mapped plane is associated with current plane
             *
             * TODO:       The cognitive complexity breaches the 3 indentation
             *              rule. Hence, a method/function should be introduced
             *              to help break this section of code down and make it
             *              more readable.
             */
            if (matchedPlaneId == -1)
            {
                /*!
                 * If semantic segmentation is running independetly, determine
                 * whether the observation is sufficiently large to become a
                 * new mapped plane.
                 */
                if (!mGeoRuns)
                {
                    /*!
                     * Apply an additional geometry check before creating a new
                     * wall.
                     *
                     * @note        Small wall observations may be produced by
                     *              doorframes, furniture edges and segmentation
                     *              noise. Small patches are still permited
                     *              to udpate an existing wall because this
                     *              check is only applied when a matchPlaneId is
                     *              -1.
                     */
                    if (semanticType == ORB_SLAM3::Plane::planeVariant::WALL)
                    {
                        const SystemParams::sem_seg::WallCreation
                            &wallCreationParams =
                                sysParams->sem_seg.wallCreation;

                        WallComponentSupport connectedSupport;

                        if (wallCreationParams.connectivity.enabled)
                        {
                            connectedSupport = findLargestWallComponent(
                                globalPlaneCloud,
                                wallCreationParams.connectivity
                                    .clusterTolerance_m);
                        }
                        else if (globalPlaneCloud != nullptr)
                        {
                            connectedSupport.finitePointCount =
                                globalPlaneCloud->size();
                            connectedSupport.componentRatio = 1.0;
                            connectedSupport.pointIndices.resize(
                                globalPlaneCloud->size());
                            std::iota(connectedSupport.pointIndices.begin(),
                                      connectedSupport.pointIndices.end(),
                                      0);
                        }

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                            p_connectedGlobalWallCloud(
                                new pcl::PointCloud<pcl::PointXYZRGBA>);

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                            p_connectedCameraWallCloud(
                                new pcl::PointCloud<pcl::PointXYZRGBA>);

                        if (globalPlaneCloud != nullptr &&
                            planeCloud != nullptr)
                        {
                            p_connectedGlobalWallCloud->reserve(
                                connectedSupport.pointIndices.size());
                            p_connectedCameraWallCloud->reserve(
                                connectedSupport.pointIndices.size());

                            for (const int sourcePointIndex :
                                 connectedSupport.pointIndices)
                            {
                                if (sourcePointIndex < 0 ||
                                    static_cast<std::size_t>(
                                        sourcePointIndex) >=
                                        globalPlaneCloud->size() ||
                                    static_cast<std::size_t>(
                                        sourcePointIndex) >= planeCloud->size())
                                {
                                    continue;
                                }

                                p_connectedGlobalWallCloud->push_back(
                                    globalPlaneCloud
                                        ->points[static_cast<std::size_t>(
                                            sourcePointIndex)]);
                                p_connectedCameraWallCloud->push_back(
                                    planeCloud->points[static_cast<std::size_t>(
                                        sourcePointIndex)]);
                            }
                        }

                        /* Compute finite dimensions from connected support. */
                        const std::pair<double, double> wallDimensions =
                            Utils::computePlaneWidthHeight(
                                p_connectedGlobalWallCloud);

                        /* Extract the larger planar dimension */
                        const double majorExtent =
                            std::max(wallDimensions.first,
                                     wallDimensions.second);

                        /* Extract the smaller planar dimension */
                        const double minorExtent =
                            std::min(wallDimensions.first,
                                     wallDimensions.second);

                        /* Compute the approximate observed planar area. */
                        const double observedArea = majorExtent * minorExtent;

                        /*!
                         * Reject unsupported and doorframe-sized wall planes.
                         *
                         * @note        These thresholds apply only to the
                         *              creation of new wall planes. Subsequent
                         *              smaller observations may still update a
                         *              mapped wall.
                         */
                        const bool validConnectivity =
                            !wallCreationParams.connectivity.enabled ||
                            (connectedSupport.pointIndices.size() >=
                                 wallCreationParams.connectivity
                                     .minimumComponentPointCount &&
                             connectedSupport.componentRatio >=
                                 wallCreationParams.connectivity
                                     .minimumComponentRatio);

                        /* Perform all configured new-wall admission checks. */
                        const bool validNewWallGeometry =
                            p_connectedGlobalWallCloud != nullptr &&
                            p_connectedGlobalWallCloud->size() >=
                                wallCreationParams.minimumPointCount &&
                            validConnectivity && std::isfinite(majorExtent) &&
                            std::isfinite(minorExtent) &&
                            std::isfinite(observedArea) &&
                            majorExtent >=
                                wallCreationParams.minimumMajorExtent_m &&
                            minorExtent >=
                                wallCreationParams.minimumMinorExtent_m &&
                            observedArea >= wallCreationParams.minimumArea_m2;

                        /* Reject narrow or small wall fragments */
                        if (!validNewWallGeometry)
                        {
                            std::cout
                                << "[SemSeg] Rejecting new wall "
                                   "candidate: points="
                                << connectedSupport.pointIndices.size() << '/'
                                << connectedSupport.finitePointCount
                                << " connected (ratio "
                                << connectedSupport.componentRatio << ')'
                                << ", dimensions=" << majorExtent << "x"
                                << minorExtent << " m, area=" << observedArea
                                << " m^2." << std::endl;

                            continue;
                        }

                        /* Persist only the validated connected wall support. */
                        globalPlaneCloud = p_connectedGlobalWallCloud;
                        planeCloud       = p_connectedCameraWallCloud;
                    }

                    /* Create a new mapped plane */
                    ORB_SLAM3::Plane *newMapPlane =
                        GeoSemHelpers::createMapPlane(mpAtlas,
                                                      pKF,
                                                      detectedPlane,
                                                      planeCloud,
                                                      semanticType,
                                                      conf);

                    /* Confirm that plane creation succeeded */
                    if (newMapPlane == nullptr)
                    {
                        continue;
                    }

                    /* Update the semantic votes of the new plane */
                    updatePlaneSemantics(newMapPlane->getId(), clsId, conf);
                }
            }
            else
            {
                /* Update matched mapped plane with the current observation
                 */
                if (!mGeoRuns)
                {
                    GeoSemHelpers::updateMapPlane(mpAtlas,
                                                  pKF,
                                                  detectedPlane,
                                                  planeCloud,
                                                  matchedPlaneId,
                                                  semanticType,
                                                  conf);
                }
                else
                {
                    /*!
                     * Geometric segmentation already created the plane.
                     * Transform the current observation into the global
                     frame
                     * and append it to the matched mapped plane.
                     */
                    pcl::transformPointCloud(
                        *planeCloud,
                        *planeCloud,
                        pKF->GetPoseInverse().matrix().cast<float>());

                    ORB_SLAM3::Plane *matchedPlane =
                        mpAtlas->GetPlaneById(matchedPlaneId);

                    if (matchedPlane != nullptr && !matchedPlane->isBad() &&
                        !planeCloud->empty())
                    {
                        matchedPlane->setMapClouds(planeCloud);

                        GeoSemHelpers::refitMappedPlaneFromCloud(matchedPlane);
                    }
                }

                /*!
                 * Cast the current semantic observation vote for the matched
                 * plane.
                 */
                updatePlaneSemantics(matchedPlaneId, clsId, conf);
            }
        }
    }

    SetFinish();
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

void SemanticSegmentation::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SemanticSegmentation::CheckFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SemanticSegmentation::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SemanticSegmentation::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

} // namespace ORB_SLAM3
