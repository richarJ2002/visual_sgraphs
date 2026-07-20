/*!
 * @File:         publishAllMappedWalls.cpp
 *
 * @Brief:        Publishes all mapped walls to detect possible rooms.
 *
 * @Date:         20/07/2026
 *
 */

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *> wallsList_in,
                           rclcpp::Time                    msgTime_s_in)
{
    /* Variables */
    vs_graphs::msg::VSGraphsAllWallsData wallDataMsg;

    /* Fill the data message with wall information */
    wallDataMsg.header.stamp    = msgTime_s_in;
    wallDataMsg.header.frame_id = frameWorld;

    /* Fill in the walls data for each wall in vector */
    for (const auto &wall : wallsList_in)
    {
        if (!wall ||
            wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
            continue;

        /* Init variable of the lenfth of the wall */
        float length = 0.0f;

        /* Get the point clouds for the wall */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
            wall->getMapClouds();

        /* Calculate the length of the wall */
        if (wallCloud && wallCloud->points.size() > 1)
        {
            /* Create position vector of the start point */
            Eigen::Vector3f startPoint(wallCloud->points.front().x,
                                       wallCloud->points.front().y,
                                       wallCloud->points.front().z);

            /* Create position vector of the end point */
            Eigen::Vector3f endPoint(wallCloud->points.back().x,
                                     wallCloud->points.back().y,
                                     wallCloud->points.back().z);

            /*!
             * Calculate the length of the wall by finding the distance between
             * the first and last points
             */
            length = (endPoint - startPoint).norm();
        }

        /* Fill the wall data */
        vs_graphs::msg::VSGraphsWallData wallData;
        wallData.length     = length;
        wallData.id         = wall->getId();
        wallData.centroid.x = wall->getCentroid().x();
        wallData.centroid.y = wall->getCentroid().y();
        wallData.centroid.z = wall->getCentroid().z();
        wallData.normal.x   = wall->getGlobalEquation().normal().x();
        wallData.normal.y   = wall->getGlobalEquation().normal().y();
        wallData.normal.z   = wall->getGlobalEquation().normal().z();

        /* Add the wall to the message */
        wallDataMsg.walls.push_back(wallData);
    }

    /* Publish all mapped walls */
    pubAllWalls_new->publish(wallDataMsg);
}
