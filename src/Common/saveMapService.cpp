/*!
 * @File:         saveMapService.cpp
 *
 * @Brief:        Handles a ROS service request to save the current ORB-SLAM3
 *                map as an ORB-SLAM atlas file.
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

void saveMapService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: service response is null.");

        return;
    }

    /* Set the default response in case validation or saving fails */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (pSLAM == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: SLAM system is not initialised.");

        return;
    }

    /* Confirm that an output name was provided */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save map: output file name is empty.");

        return;
    }

    /* Request that ORB-SLAM3 save the current map */
    try
    {
        response_out->success = pSLAM->SaveMap(request_in->name);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving map as '%s.osa': %s",
                     request_in->name.c_str(),
                     exception.what());

        return;
    }

    /* Report the result of the save operation */
    if (response_out->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                    "Map was saved as '%s.osa'.",
                    request_in->name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Map could not be saved as '%s.osa'.",
                     request_in->name.c_str());
    }
}
