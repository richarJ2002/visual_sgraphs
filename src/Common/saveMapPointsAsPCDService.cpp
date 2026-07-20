/*!
 * @File:         saveMapPointsAsPCDService.cpp
 *
 * @Brief:        Handles a ROS service request to save the current ORB-SLAM3
 * map points as a PCD file.
 *
 * @Date:         20/07/2026
 *
 */

#include <exception>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void saveMapPointsAsPCDService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: service response is null.");

        return;
    }

    /* Set the default response in case validation or saving fails */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (pSLAM == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: SLAM system is not initialised.");

        return;
    }

    /* Confirm that an output name was provided */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map points: output file name is empty.");

        return;
    }

    /* Request that ORB-SLAM3 save the current map points */
    try
    {
        response_out->success = pSLAM->SaveMapPointsAsPCD(request_in->name);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving map points as '%s.pcd': %s",
                     request_in->name.c_str(),
                     exception.what());

        return;
    }

    /* Report the result of the save operation */
    if (response_out->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map points were saved as '%s.pcd'.",
                    request_in->name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map points could not be saved as '%s.pcd'.",
                     request_in->name.c_str());
    }
}
