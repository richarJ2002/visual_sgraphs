/*!
 * @File:         publishFreeSpaceClusters.cpp
 *
 * @Brief:        Converts the supplied Voxblox free-space clusters into a
 *                coloured ROS point-cloud message and publishes the result.
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

void publishFreeSpaceClusters(
    const std::vector<std::vector<Eigen::Vector3d>> &freeSpaceClusters_World_in,
    const rclcpp::Time                              &msgTime_s_in)
{
    /* Confirm that the publisher has been initialised */
    if (pubFreespaceCluster == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish free-space clusters: publisher is not "
                    "initialised.");

        return;
    }

    /* Return when there are no free-space clusters to publish */
    if (freeSpaceClusters_World_in.empty())
    {
        return;
    }

    /*!
     * Fixed RGB colours used to distinguish neighbouring free-space clusters.
     * Colours are reused when the number of clusters exceeds the palette size.
     */
    static constexpr std::array<std::array<std::uint8_t, 3>, 7>
        clusterColourPalette = {{{{255, 0, 0}},
                                 {{0, 255, 0}},
                                 {{0, 0, 255}},
                                 {{255, 255, 0}},
                                 {{0, 255, 255}},
                                 {{255, 0, 255}},
                                 {{128, 0, 0}}}};

    /* Calculate the total number of points so memory can be reserved once */
    std::size_t totalPointCount = 0;

    for (const std::vector<Eigen::Vector3d> &cluster :
         freeSpaceClusters_World_in)
    {
        totalPointCount += cluster.size();
    }

    if (totalPointCount == 0)
    {
        return;
    }

    /* Initialise the combined coloured free-space point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> freeSpacePointCloud_world;

    freeSpacePointCloud_world.points.reserve(totalPointCount);

    /* Convert every free-space cluster into coloured PCL points */
    for (std::size_t clusterIndex = 0;
         clusterIndex < freeSpaceClusters_World_in.size();
         clusterIndex++)
    {
        const std::vector<Eigen::Vector3d> &clusterPoints_world =
            freeSpaceClusters_World_in[clusterIndex];

        const std::array<std::uint8_t, 3> &clusterColour =
            clusterColourPalette[clusterIndex % clusterColourPalette.size()];

        for (const Eigen::Vector3d &point_world_m : clusterPoints_world)
        {
            /* Ignore invalid points */
            if (!point_world_m.allFinite())
            {
                continue;
            }

            pcl::PointXYZRGB colouredPoint_world;

            colouredPoint_world.x = static_cast<float>(point_world_m.x());
            colouredPoint_world.y = static_cast<float>(point_world_m.y());
            colouredPoint_world.z = static_cast<float>(point_world_m.z());

            colouredPoint_world.r = clusterColour[0];
            colouredPoint_world.g = clusterColour[1];
            colouredPoint_world.b = clusterColour[2];

            freeSpacePointCloud_world.points.push_back(colouredPoint_world);
        }
    }

    /* Return when all supplied points were invalid */
    if (freeSpacePointCloud_world.empty())
    {
        return;
    }

    /* Complete the PCL metadata */
    freeSpacePointCloud_world.width =
        static_cast<std::uint32_t>(freeSpacePointCloud_world.points.size());

    freeSpacePointCloud_world.height = 1;

    freeSpacePointCloud_world.is_dense = true;

    /* Convert the PCL point cloud into a ROS message */
    sensor_msgs::msg::PointCloud2 freeSpacePointCloudMessage_world;

    pcl::toROSMsg(freeSpacePointCloud_world, freeSpacePointCloudMessage_world);

    /* Set the output message metadata */
    freeSpacePointCloudMessage_world.header.stamp = msgTime_s_in;

    freeSpacePointCloudMessage_world.header.frame_id = frameWorld;

    /* Publish the combined free-space cluster point cloud */
    pubFreespaceCluster->publish(freeSpacePointCloudMessage_world);
}
