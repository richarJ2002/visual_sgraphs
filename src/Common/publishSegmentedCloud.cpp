/*!
 * @File:         publishSegmentedCloud.cpp
 *
 * @Brief:        Publishes the most recent available semantically segmented
 *                keyframe point cloud.
 *
 * @Date:         20/07/2026
 *
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishSegmentedCloud(
    const std::vector<ORB_SLAM3::KeyFrame *> &keyFrames_in)
{
    /* Confirm that the segmented-cloud publisher has been initialised */
    if (pubSegmentedPointcloud == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish segmented point cloud: publisher is not "
                    "initialised.");

        return;
    }

    /* Return when there are no keyframes to inspect */
    if (keyFrames_in.empty())
    {
        return;
    }

    /* Latest keyframe containing valid class-specific point clouds */
    ORB_SLAM3::KeyFrame *selectedKeyFrame = nullptr;

    std::size_t selectedKeyFrameIndex = 0;

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
        classPointClouds_camera;

    /*!
     * Search backwards without converting an unsigned container size into a
     * potentially invalid negative array index.
     */
    for (std::size_t reverseIndex = keyFrames_in.size(); reverseIndex > 0;
         reverseIndex--)
    {
        const std::size_t keyFrameIndex = reverseIndex - 1;

        ORB_SLAM3::KeyFrame *keyFrame = keyFrames_in[keyFrameIndex];

        if (keyFrame == nullptr)
        {
            continue;
        }

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            candidateClassPointClouds = keyFrame->getClsCloudPtrs();

        /* Determine whether at least one class cloud contains points */
        const bool hasSegmentedPoints = std::any_of(
            candidateClassPointClouds.begin(),
            candidateClassPointClouds.end(),
            [](const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud) {
                return classPointCloud != nullptr && !classPointCloud->empty();
            });

        if (!hasSegmentedPoints)
        {
            continue;
        }

        selectedKeyFrame = keyFrame;

        selectedKeyFrameIndex = keyFrameIndex;

        classPointClouds_camera = std::move(candidateClassPointClouds);

        break;
    }

    /* Return when no processed keyframe contains segmented points */
    if (selectedKeyFrame == nullptr)
    {
        return;
    }

    /*!
     * Clear stale class clouds belonging to keyframes older than the selected
     * keyframe.
     */
    for (std::size_t keyFrameIndex = 0; keyFrameIndex < selectedKeyFrameIndex;
         keyFrameIndex++)
    {
        ORB_SLAM3::KeyFrame *olderKeyFrame = keyFrames_in[keyFrameIndex];

        if (olderKeyFrame != nullptr)
        {
            olderKeyFrame->clearClsClouds();
        }
    }

    /* Calculate the required cloud capacity before aggregation */
    std::size_t totalSegmentedPointCount = 0;

    for (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud :
         classPointClouds_camera)
    {
        if (classPointCloud != nullptr)
        {
            totalSegmentedPointCount += classPointCloud->size();
        }
    }

    if (totalSegmentedPointCount == 0)
    {
        selectedKeyFrame->clearClsClouds();
        return;
    }

    /* Initialise the combined segmented point cloud */
    pcl::PointCloud<pcl::PointXYZRGBA> segmentedPointCloud_camera;

    segmentedPointCloud_camera.points.reserve(totalSegmentedPointCount);

    bool hasSourceHeader = false;

    /* Aggregate the class-specific point clouds */
    for (std::size_t classIndex = 0;
         classIndex < classPointClouds_camera.size();
         classIndex++)
    {
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &classPointCloud_camera =
            classPointClouds_camera[classIndex];

        /* Skip missing or empty class clouds */
        if (classPointCloud_camera == nullptr ||
            classPointCloud_camera->empty())
        {
            continue;
        }

        /* Preserve the metadata from the first valid source cloud */
        if (!hasSourceHeader)
        {
            segmentedPointCloud_camera.header = classPointCloud_camera->header;

            hasSourceHeader = true;
        }

        for (const pcl::PointXYZRGBA &sourcePoint_camera :
             classPointCloud_camera->points)
        {
            /* Skip points containing invalid coordinates */
            if (!std::isfinite(sourcePoint_camera.x) ||
                !std::isfinite(sourcePoint_camera.y) ||
                !std::isfinite(sourcePoint_camera.z))
            {
                continue;
            }

            pcl::PointXYZRGBA segmentedPoint_camera = sourcePoint_camera;

            switch (classIndex)
            {
            case 0:
            {
                /* Display ground points in green */
                segmentedPoint_camera.r = 0;
                segmentedPoint_camera.g = 255;
                segmentedPoint_camera.b = 0;
                break;
            }

            case 1:
            {
                /* Display wall points in red */
                segmentedPoint_camera.r = 255;
                segmentedPoint_camera.g = 0;
                segmentedPoint_camera.b = 0;
                break;
            }

            default:
            {
                /*!
                 * Preserve the original colour for all other semantic
                 * classes.
                 */
                break;
            }
            }

            segmentedPointCloud_camera.points.push_back(segmentedPoint_camera);
        }
    }

    /*!
     * The class clouds have now been consumed and should not be republished on
     * the next invocation.
     */
    selectedKeyFrame->clearClsClouds();

    /* Return when every available point was invalid */
    if (segmentedPointCloud_camera.empty())
    {
        return;
    }

    /* Complete the PCL point-cloud metadata */
    segmentedPointCloud_camera.width =
        static_cast<std::uint32_t>(segmentedPointCloud_camera.points.size());

    segmentedPointCloud_camera.height = 1;

    segmentedPointCloud_camera.is_dense = true;

    /* Convert the combined PCL cloud into a ROS point-cloud message */
    sensor_msgs::msg::PointCloud2 segmentedPointCloudMessage_camera;

    pcl::toROSMsg(segmentedPointCloud_camera,
                  segmentedPointCloudMessage_camera);

    /*!
     * The class-specific clouds are currently represented in the camera
     * frame.
     */
    segmentedPointCloudMessage_camera.header.frame_id = frameCamera;

    /* Publish the combined segmented point cloud */
    pubSegmentedPointcloud->publish(segmentedPointCloudMessage_camera);
}
