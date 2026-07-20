/*!
 * @File:         publishPlanes.cpp
 *
 * @Brief:        Publishes valid mapped planes as an aggregated
 *                building-component point cloud and as RViz label and normal
 *                markers.
 *
 * @Date:         20/07/2026
 *
 */

#include <array>
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

void publishPlanes(const std::vector<ORB_SLAM3::Plane *> &mappedPlanes_in,
                   const rclcpp::Time                    &msgTime_s_in)
{
    /* Return when neither required publisher has been initialised */
    if (pubBuildingComponents == nullptr && pubPlaneLabel == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish planes: plane publishers are not initialised.");

        return;
    }

    /* Return when there are no mapped planes to publish */
    if (mappedPlanes_in.empty())
    {
        return;
    }

    /* Limit expensive plane publication to once every three seconds */
    constexpr double planePublicationPeriod_s = 3.0;

    if (lastPlanePublishTime.nanoseconds() != 0 &&
        (msgTime_s_in - lastPlanePublishTime).seconds() <
            planePublicationPeriod_s)
    {
        return;
    }

    lastPlanePublishTime = msgTime_s_in;

    /* Initialise the combined building-component point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> buildingComponentPointCloud_BC;

    /* Initialise the plane visualisation marker array */
    visualization_msgs::msg::MarkerArray planeVisualizationArray;

    planeVisualizationArray.markers.reserve(mappedPlanes_in.size() * 2);

    /* Visualisation constants */
    constexpr double planeLabelOffset_BC_m = -1.5;
    constexpr double planeNormalLength_m   = 0.2;
    constexpr double normalVectorTolerance = 1e-9;

    /* Process every mapped plane */
    for (ORB_SLAM3::Plane *mappedPlane : mappedPlanes_in)
    {
        /* Skip invalid planes */
        if (mappedPlane == nullptr || mappedPlane->isBad())
        {
            continue;
        }

        /* Extract the semantic plane type once */
        const ORB_SLAM3::Plane::planeVariant planeType =
            mappedPlane->getPlaneType();

        /* Skip planes that have not received a semantic type */
        if (planeType == ORB_SLAM3::Plane::planeVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the plane point cloud */
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloud_BC =
            mappedPlane->getMapClouds();

        /* Skip planes without any mapped points */
        if (planePointCloud_BC == nullptr || planePointCloud_BC->empty())
        {
            continue;
        }

        /* Extract and validate the plane centroid */
        const Eigen::Vector3f planeCentroid_BC_m = mappedPlane->getCentroid();

        if (!planeCentroid_BC_m.allFinite())
        {
            continue;
        }

        /* Extract and validate the plane normal */
        Eigen::Vector3d planeNormal_BC =
            mappedPlane->getGlobalEquation().normal();

        if (!planeNormal_BC.allFinite() ||
            planeNormal_BC.norm() < normalVectorTolerance)
        {
            continue;
        }

        /*
         * Normalise the plane normal so the displayed arrow always has the
         * requested physical length.
         */
        planeNormal_BC.normalize();

        /* Extract the configured plane colour */
        const std::vector<std::uint8_t> configuredColour =
            mappedPlane->getColor();

        std::array<std::uint8_t, 3> planeColour_rgb = {255, 255, 255};

        if (configuredColour.size() >= 3)
        {
            planeColour_rgb = {configuredColour[0],
                               configuredColour[1],
                               configuredColour[2]};
        }

        /* Append the current plane to the aggregated point cloud */
        for (const pcl::PointXYZRGBA &sourcePoint_BC :
             planePointCloud_BC->points)
        {
            /* Skip invalid point coordinates */
            if (!std::isfinite(sourcePoint_BC.x) ||
                !std::isfinite(sourcePoint_BC.y) ||
                !std::isfinite(sourcePoint_BC.z))
            {
                continue;
            }

            pcl::PointXYZRGB colouredPoint_BC;

            colouredPoint_BC.x = sourcePoint_BC.x;
            colouredPoint_BC.y = sourcePoint_BC.y;
            colouredPoint_BC.z = sourcePoint_BC.z;

            colouredPoint_BC.r = sourcePoint_BC.r;
            colouredPoint_BC.g = sourcePoint_BC.g;
            colouredPoint_BC.b = sourcePoint_BC.b;

            /* Display detected door planes using a fixed magenta colour */
            if (planeType == ORB_SLAM3::Plane::planeVariant::DOOR)
            {
                colouredPoint_BC.r = 204;
                colouredPoint_BC.g = 0;
                colouredPoint_BC.b = 102;
            }

            buildingComponentPointCloud_BC.points.push_back(colouredPoint_BC);
        }

        /*
         * Use the persistent plane ID. The label and normal markers may share
         * the same ID because they use different namespaces.
         */
        const int planeMarkerId = static_cast<int>(mappedPlane->getId());

        /* ------------------------------------------------------------------ *
         * PLANE LABEL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker planeLabelMarker;

        planeLabelMarker.header.frame_id = frameBC;
        planeLabelMarker.header.stamp    = msgTime_s_in;

        planeLabelMarker.ns = "plane_label";
        planeLabelMarker.id = planeMarkerId;

        planeLabelMarker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        planeLabelMarker.action = visualization_msgs::msg::Marker::ADD;

        planeLabelMarker.text = "Plane#" + std::to_string(mappedPlane->getId());

        planeLabelMarker.pose.position.x = planeCentroid_BC_m.x();
        planeLabelMarker.pose.position.y =
            planeCentroid_BC_m.y() + planeLabelOffset_BC_m;
        planeLabelMarker.pose.position.z = planeCentroid_BC_m.z();

        planeLabelMarker.pose.orientation.x = 0.0;
        planeLabelMarker.pose.orientation.y = 0.0;
        planeLabelMarker.pose.orientation.z = 0.0;
        planeLabelMarker.pose.orientation.w = 1.0;

        planeLabelMarker.scale.z = 0.2;

        planeLabelMarker.color.a = 1.0;

        planeLabelMarker.color.r =
            static_cast<float>(planeColour_rgb[0]) / 255.0F;

        planeLabelMarker.color.g =
            static_cast<float>(planeColour_rgb[1]) / 255.0F;

        planeLabelMarker.color.b =
            static_cast<float>(planeColour_rgb[2]) / 255.0F;

        planeLabelMarker.lifetime = rclcpp::Duration::from_seconds(0);

        planeVisualizationArray.markers.push_back(std::move(planeLabelMarker));

        /* ------------------------------------------------------------------ *
         * PLANE NORMAL
         * ------------------------------------------------------------------ */

        visualization_msgs::msg::Marker planeNormalMarker;

        planeNormalMarker.header.frame_id = frameBC;
        planeNormalMarker.header.stamp    = msgTime_s_in;

        planeNormalMarker.ns = "plane_normal";
        planeNormalMarker.id = planeMarkerId;

        planeNormalMarker.type   = visualization_msgs::msg::Marker::ARROW;
        planeNormalMarker.action = visualization_msgs::msg::Marker::ADD;

        /* Arrow shaft diameter */
        planeNormalMarker.scale.x = 0.01;

        /* Arrowhead diameter */
        planeNormalMarker.scale.y = 0.05;

        /* Arrowhead length */
        planeNormalMarker.scale.z = 0.05;

        planeNormalMarker.color.a = 1.0;

        planeNormalMarker.color.r =
            static_cast<float>(planeColour_rgb[0]) / 255.0F;
        planeNormalMarker.color.g =
            static_cast<float>(planeColour_rgb[1]) / 255.0F;
        planeNormalMarker.color.b =
            static_cast<float>(planeColour_rgb[2]) / 255.0F;

        /* Set the beginning of the plane-normal arrow */
        geometry_msgs::msg::Point normalStartPoint_BC;

        normalStartPoint_BC.x = planeCentroid_BC_m.x();
        normalStartPoint_BC.y = planeCentroid_BC_m.y();
        normalStartPoint_BC.z = planeCentroid_BC_m.z();

        /* Set the end of the plane-normal arrow */
        geometry_msgs::msg::Point normalEndPoint_BC;

        normalEndPoint_BC.x =
            normalStartPoint_BC.x + planeNormal_BC.x() * planeNormalLength_m;

        normalEndPoint_BC.y =
            normalStartPoint_BC.y + planeNormal_BC.y() * planeNormalLength_m;

        normalEndPoint_BC.z =
            normalStartPoint_BC.z + planeNormal_BC.z() * planeNormalLength_m;

        planeNormalMarker.points.reserve(2);

        planeNormalMarker.points.push_back(normalStartPoint_BC);

        planeNormalMarker.points.push_back(normalEndPoint_BC);

        planeNormalMarker.lifetime = rclcpp::Duration::from_seconds(0);

        planeVisualizationArray.markers.push_back(std::move(planeNormalMarker));
    }

    /* Publish the aggregated building-component point cloud */
    if (pubBuildingComponents != nullptr &&
        !buildingComponentPointCloud_BC.empty())
    {
        buildingComponentPointCloud_BC.width = static_cast<std::uint32_t>(
            buildingComponentPointCloud_BC.points.size());

        buildingComponentPointCloud_BC.height = 1;

        buildingComponentPointCloud_BC.is_dense = true;

        sensor_msgs::msg::PointCloud2 buildingComponentPointCloudMessage_BC;

        pcl::toROSMsg(buildingComponentPointCloud_BC,
                      buildingComponentPointCloudMessage_BC);

        buildingComponentPointCloudMessage_BC.header.stamp = msgTime_s_in;

        buildingComponentPointCloudMessage_BC.header.frame_id = frameBC;

        pubBuildingComponents->publish(buildingComponentPointCloudMessage_BC);
    }

    /* Publish the plane labels and normal arrows */
    if (pubPlaneLabel != nullptr && !planeVisualizationArray.markers.empty())
    {
        pubPlaneLabel->publish(planeVisualizationArray);
    }
}
