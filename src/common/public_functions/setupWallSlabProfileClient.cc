/*!
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

/*!
 * @file            setupWallSlabProfileClient.cc
 *
 * @brief           Implements setupWallSlabProfileClient(), declared in
 *                  common.h, and owns the wall-slab client and timer.
 */

#include "common/common.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace vs_graphs::common
{

rclcpp::Client<voxblox_msgs::srv::GetPlaneSlabProfiles>::SharedPtr
    p_wallSlabProfileClient = nullptr;

rclcpp::TimerBase::SharedPtr p_wallSlabProfileTimer = nullptr;

namespace
{

/*!
 * Exchange state. Read and written only by the timer and response callbacks,
 * which share one mutually exclusive callback group, so no mutex is needed.
 */

/* TSDF frame of /voxblox_skeletonizer (its frame_id parameter). */
const std::string VOXBLOX_TSDF_FRAME = "map_elevated";

/* A request older than this is dropped so a lost reply cannot stall. */
constexpr double REQUEST_TIMEOUT_S = 5.0;

/* Period of the SG_PIPELINE slab_profiles summary line. */
constexpr double SUMMARY_PERIOD_S = 10.0;

bool                                  isRequestInFlight = false;
std::chrono::steady_clock::time_point requestSendTime{};
std::chrono::steady_clock::time_point lastSummaryTime{};
std::size_t                           sentRequestCount     = 0U;
std::size_t                           answeredRequestCount = 0U;
std::size_t                           failedRequestCount   = 0U;
std::size_t                           lastWallCount        = 0U;
double                                lastLatency_ms       = 0.0;
double                                maximumLatency_ms    = 0.0;
std::string                           lastFailureMessage{};

} /* namespace */

void setupWallSlabProfileClient(
    const std::shared_ptr<rclcpp::Node>    &node_in,
    const rclcpp::CallbackGroup::SharedPtr &callbackGroup_in)
{
    using voxblox_msgs::srv::GetPlaneSlabProfiles;

    if (node_in == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot start the wall-slab client: ROS node is null.");
        return;
    }

    p_wallSlabProfileClient = node_in->create_client<GetPlaneSlabProfiles>(
        "/voxblox_skeletonizer/get_plane_slab_profiles",
        rclcpp::ServicesQoS(),
        callbackGroup_in);

    const rclcpp::Clock::SharedPtr p_nodeClock = node_in->get_clock();

    const auto requestWallSlabProfiles = [p_nodeClock]()
    {
        const std::chrono::steady_clock::time_point tickTime =
            std::chrono::steady_clock::now();

        /* One summary line per period, whatever the exchange is doing. */
        if (std::chrono::duration<double>(tickTime - lastSummaryTime).count() >=
            SUMMARY_PERIOD_S)
        {
            lastSummaryTime = tickTime;
            std::cout << "SG_PIPELINE {\"event\":\"slab_profiles\""
                      << ",\"sent\":" << sentRequestCount
                      << ",\"answered\":" << answeredRequestCount
                      << ",\"failed\":" << failedRequestCount
                      << ",\"walls\":" << lastWallCount
                      << ",\"last_latency_ms\":" << lastLatency_ms
                      << ",\"max_latency_ms\":" << maximumLatency_ms
                      << ",\"last_failure\":\"" << lastFailureMessage << "\"}"
                      << std::endl;
            maximumLatency_ms = 0.0;
        }

        if (isRequestInFlight)
        {
            if (std::chrono::duration<double>(tickTime - requestSendTime)
                    .count() < REQUEST_TIMEOUT_S)
            {
                return;
            }
            p_wallSlabProfileClient->prune_pending_requests();
            isRequestInFlight = false;
            failedRequestCount++;
            lastFailureMessage = "timeout";
        }

        if (p_slamSystem == nullptr || p_wallSlabProfileClient == nullptr ||
            !p_wallSlabProfileClient->service_is_ready())
        {
            return;
        }

        std::vector<vs_graphs::core::WallSlabQuery> wallSlabQueries{};
        if (p_slamSystem->getWallSlabQueries(wallSlabQueries) !=
            vs_graphs::core::system::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWallSlabQueries returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallSlabQueries.empty())
        {
            return;
        }

        /* Queries are in the SLAM world frame; the TSDF is not. */
        tf2::Transform frameTransform_tsdfToWorld;
        if (!lookupWorldFromFrameTransform(VOXBLOX_TSDF_FRAME,
                                           frameTransform_tsdfToWorld))
        {
            return;
        }
        const tf2::Transform frameTransform_worldToTsdf =
            frameTransform_tsdfToWorld.inverse();
        const tf2::Matrix3x3 &rotationMatrix_worldToTsdf =
            frameTransform_worldToTsdf.getBasis();

        const auto toTsdfArray = [](const tf2::Vector3 &vector_tsdf_in)
        {
            return std::array<double, 3>{vector_tsdf_in.x(),
                                         vector_tsdf_in.y(),
                                         vector_tsdf_in.z()};
        };
        const auto toTf2Vector = [](const Eigen::Vector3d &vector_world_in)
        {
            return tf2::Vector3(vector_world_in.x(),
                                vector_world_in.y(),
                                vector_world_in.z());
        };

        const std::shared_ptr<GetPlaneSlabProfiles::Request> p_slabRequest =
            std::make_shared<GetPlaneSlabProfiles::Request>();
        p_slabRequest->frame_id = VOXBLOX_TSDF_FRAME;
        p_slabRequest->slabs.reserve(wallSlabQueries.size());
        for (const vs_graphs::core::WallSlabQuery &wallQuery : wallSlabQueries)
        {
            voxblox_msgs::msg::PlaneSlab planeSlab;
            planeSlab.origin =
                toTsdfArray(frameTransform_worldToTsdf *
                            toTf2Vector(wallQuery.origin_world_m));
            planeSlab.first_axis =
                toTsdfArray(rotationMatrix_worldToTsdf *
                            toTf2Vector(wallQuery.alongAxis_world));
            planeSlab.second_axis =
                toTsdfArray(rotationMatrix_worldToTsdf *
                            toTf2Vector(wallQuery.upAxis_world));
            planeSlab.normal         = toTsdfArray(rotationMatrix_worldToTsdf *
                                           toTf2Vector(wallQuery.normal_world));
            planeSlab.column_count   = wallQuery.columnCount;
            planeSlab.row_count      = wallQuery.rowCount;
            planeSlab.cell_size      = wallQuery.cellSize_m;
            planeSlab.half_thickness = wallQuery.halfThickness_m;
            p_slabRequest->slabs.push_back(planeSlab);
        }

        const std::int64_t profileStamp_ns = p_nodeClock->now().nanoseconds();
        isRequestInFlight                  = true;
        requestSendTime                    = tickTime;
        lastWallCount                      = wallSlabQueries.size();
        sentRequestCount++;

        p_wallSlabProfileClient->async_send_request(
            p_slabRequest,
            [wallSlabQueries,
             profileStamp_ns](rclcpp::Client<GetPlaneSlabProfiles>::SharedFuture
                                  slabResponseFuture_in)
            {
                isRequestInFlight = false;
                lastLatency_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - requestSendTime)
                        .count();
                maximumLatency_ms = std::max(maximumLatency_ms, lastLatency_ms);

                const std::shared_ptr<GetPlaneSlabProfiles::Response>
                    p_slabResponse = slabResponseFuture_in.get();
                if (p_slabResponse == nullptr || !p_slabResponse->success ||
                    p_slabResponse->profiles.size() != wallSlabQueries.size())
                {
                    failedRequestCount++;
                    lastFailureMessage = p_slabResponse == nullptr
                                             ? "no response"
                                             : p_slabResponse->message;
                    return;
                }

                std::vector<vs_graphs::core::WallSlabProfile>
                    wallSlabProfiles{};
                wallSlabProfiles.reserve(wallSlabQueries.size());
                for (std::size_t slabIndex = 0U;
                     slabIndex < wallSlabQueries.size();
                     slabIndex++)
                {
                    const voxblox_msgs::msg::PlaneSlabProfile &slabProfile =
                        p_slabResponse->profiles[slabIndex];
                    const vs_graphs::core::WallSlabQuery &wallQuery =
                        wallSlabQueries[slabIndex];
                    const std::size_t cellCount =
                        static_cast<std::size_t>(wallQuery.columnCount) *
                        wallQuery.rowCount;
                    if (slabProfile.cell_state.size() != cellCount ||
                        slabProfile.cell_distance.size() != cellCount)
                    {
                        continue;
                    }
                    vs_graphs::core::WallSlabProfile wallProfile{};
                    wallProfile.query           = wallQuery;
                    wallProfile.profileStamp_ns = profileStamp_ns;
                    wallProfile.cellDistances_m = slabProfile.cell_distance;
                    wallProfile.cellStates.reserve(cellCount);
                    for (const std::uint8_t cellState : slabProfile.cell_state)
                    {
                        /* Values outside the message constants are unknown. */
                        wallProfile.cellStates.push_back(
                            cellState <= voxblox_msgs::msg::PlaneSlabProfile::
                                             CELL_OCCUPIED
                                ? static_cast<vs_graphs::core::SlabCellState>(
                                      cellState)
                                : vs_graphs::core::SlabCellState::UNKNOWN);
                    }
                    wallSlabProfiles.push_back(std::move(wallProfile));
                }

                answeredRequestCount++;
                if (p_slamSystem != nullptr &&
                    p_slamSystem->setWallSlabProfiles(wallSlabProfiles) !=
                        vs_graphs::core::system::SystemStatus::
                            SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: setWallSlabProfiles returned a failure "
                                 "status although it cannot fail; continuing "
                                 "as before.",
                                 __func__);
                }
            });
    };

    p_wallSlabProfileTimer =
        rclcpp::create_timer(node_in,
                             p_nodeClock,
                             rclcpp::Duration::from_seconds(1.0),
                             requestWallSlabProfiles,
                             callbackGroup_in);
}

} /* namespace vs_graphs::common */
