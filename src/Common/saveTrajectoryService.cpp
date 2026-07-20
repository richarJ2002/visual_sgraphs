/*!
 * @File:         saveTrajectoryService.cpp
 *
 * @Brief:        Handles a ROS service request to save the estimated camera and
 *                keyframe trajectories in EuRoC trajectory format.
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

void saveTrajectoryService(
    const std::shared_ptr<vs_graphs::srv::SaveMap::Request> request_in,
    std::shared_ptr<vs_graphs::srv::SaveMap::Response>      response_out)
{
    /* Confirm that a valid service response was supplied */
    if (response_out == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: service response is null.");

        return;
    }

    /* Set failure as the default service result */
    response_out->success = false;

    /* Confirm that a valid service request was supplied */
    if (request_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: service request is null.");

        return;
    }

    /* Confirm that the SLAM system has been initialised */
    if (pSLAM == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot save trajectories: SLAM system is not initialised.");

        return;
    }

    /* Confirm that a valid output base name was supplied */
    if (request_in->name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot save trajectories: output base name is empty.");

        return;
    }

    /* Construct the output trajectory file names */
    const std::string cameraTrajectoryFileName =
        request_in->name + "_cam_traj.txt";

    const std::string keyFrameTrajectoryFileName =
        request_in->name + "_kf_traj.txt";

    try
    {
        /* Save the complete estimated camera trajectory */
        pSLAM->SaveTrajectoryEuRoC(cameraTrajectoryFileName);

        /* Save the estimated keyframe trajectory */
        pSLAM->SaveKeyFrameTrajectoryEuRoC(keyFrameTrajectoryFileName);

        response_out->success = true;
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Exception while saving estimated trajectories: %s",
                     exception.what());

        return;
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Unknown exception while saving estimated trajectories.");

        return;
    }

    /* Report the completed save operation */
    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                "Camera trajectory was saved as '%s'.",
                cameraTrajectoryFileName.c_str());

    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                "Keyframe trajectory was saved as '%s'.",
                keyFrameTrajectoryFileName.c_str());
}
