/*!
 * @File:         setupServices.cpp
 *
 * @Brief:        Initialises the ROS services provided by the Visual S-Graphs
 *                interface.
 *
 * @Date:         20/07/2026
 *
 */

#include <memory>
#include <string>

/* Function Includes */
#include "Common.hpp"

/* Object Includes */
/* None */

/* Data Includes */
#include <vs_graphs/srv/save_map.hpp>

/* ROS Includes */
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

/* Generic Libraries */
/* None */

void setupServices(const std::shared_ptr<rclcpp::Node> &node_in,
                   const std::string                   &serviceNamespace_in)
{
    /* Confirm that a valid ROS node was supplied */
    if (node_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot initialise services: ROS node is null.");

        return;
    }

    /*!
     * Remove trailing separators so generated service names do not contain
     * repeated '/' characters.
     */
    std::string normalisedServiceNamespace = serviceNamespace_in;

    while (!normalisedServiceNamespace.empty() &&
           normalisedServiceNamespace.back() == '/')
    {
        normalisedServiceNamespace.pop_back();
    }

    /*!
     * Construct a complete service name using the configured namespace.
     *
     * An empty namespace produces relative service names such as "save_map".
     */
    const auto makeServiceName =
        [&normalisedServiceNamespace](const std::string &serviceSuffix)
    {
        if (normalisedServiceNamespace.empty())
        {
            return serviceSuffix;
        }

        return normalisedServiceNamespace + "/" + serviceSuffix;
    };

    /*!
     * Release any existing service instances before replacing them. This is
     * relevant if setupServices() is called more than once.
     */
    srvSaveMap.reset();
    srvSaveMapPoints.reset();
    srvSaveTrajectory.reset();

    /* Create the complete-map save service */
    srvSaveMap = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_map"),
        &saveMapService);

    /* Create the map-point PCD save service */
    srvSaveMapPoints = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_map_points"),
        &saveMapPointsAsPCDService);

    /* Create the trajectory save service */
    srvSaveTrajectory = node_in->create_service<vs_graphs::srv::SaveMap>(
        makeServiceName("save_traj"),
        &saveTrajectoryService);

    /* Confirm that all service objects were created */
    if (srvSaveMap == nullptr || srvSaveMapPoints == nullptr ||
        srvSaveTrajectory == nullptr)
    {
        RCLCPP_ERROR(
            node_in->get_logger(),
            "One or more Visual S-Graphs services could not be initialised.");

        return;
    }

    RCLCPP_INFO(
        node_in->get_logger(),
        "Visual S-Graphs services were initialised under namespace '%s'.",
        normalisedServiceNamespace.empty()
            ? "<relative>"
            : normalisedServiceNamespace.c_str());
}