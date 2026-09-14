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

#include "Types/SystemParams.h"

#include "System.h"

#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3
{
SystemParams *SystemParams::mSystemParams = nullptr;

SystemParams::SystemParams()
{
    mSystemParams = nullptr;
}

SystemParams *SystemParams::GetParams()
{
    if (mSystemParams == nullptr)
        mSystemParams = new SystemParams();
    return mSystemParams;
}

void SystemParams::SetParams(const std::string &strConfigFile)
{
    VSLAM_LOG_INFO("[SysParams] Loading system parameters from %s",
                   strConfigFile.c_str());
    try
    {
        mConfig = YAML::LoadFile(strConfigFile);
        VSLAM_LOG_INFO("[SysParams] System parameters loaded!");
    }
    catch (YAML::BadFile &e)
    {
        VSLAM_LOG_ERROR("[SysParams] Error loading configuration file %s",
                        e.what());
        VSLAM_LOG_ERROR("[SysParams] Exiting ...");
        exit(1);
    }

    // Set parameters
    try
    {
        // General Parameters
        general.env_database =
            mConfig["general"]["env_database"].as<std::string>();
        general.mode_of_operation = static_cast<general::ModeOfOperation>(
            mConfig["general"]["mode_of_operation"].as<int>());

        // Marker Parameters
        markers.impact = mConfig["markers"]["impact"].as<float>();

        // Tracking Refinement Parameters
        refine_map_points.enabled =
            mConfig["refine_map_points"]["enabled"].as<bool>();
        refine_map_points.max_distance_for_delete =
            mConfig["refine_map_points"]["max_distance_for_delete"].as<float>();
        refine_map_points.octree.resolution =
            mConfig["refine_map_points"]["octree"]["resolution"].as<float>();
        refine_map_points.octree.search_radius =
            mConfig["refine_map_points"]["octree"]["search_radius"].as<float>();
        refine_map_points.octree.min_neighbors =
            mConfig["refine_map_points"]["octree"]["min_neighbors"]
                .as<unsigned int>();

        // Plane based Covisibility Parameters
        plane_based_covisibility.enabled =
            mConfig["plane_based_covisibility"]["enabled"].as<bool>();
        plane_based_covisibility.max_keyframes =
            mConfig["plane_based_covisibility"]["max_keyframes"]
                .as<unsigned int>();
        plane_based_covisibility.score_per_plane =
            mConfig["plane_based_covisibility"]["score_per_plane"]
                .as<unsigned int>();

        // Common Segmentation Parameters
        seg.pointclouds_thresh =
            mConfig["seg"]["pointclouds_thresh"].as<unsigned int>();
        seg.plane_point_dist_thresh =
            mConfig["seg"]["plane_point_dist_thresh"].as<float>();
        seg.plane_association.ominus_thresh =
            mConfig["seg"]["plane_association"]["ominus_thresh"].as<float>();
        seg.plane_association.distance_thresh =
            mConfig["seg"]["plane_association"]["distance_thresh"].as<float>();
        seg.plane_association.centroid_thresh =
            mConfig["seg"]["plane_association"]["centroid_thresh"].as<float>();
        seg.plane_association.cluster_separation.enabled =
            mConfig["seg"]["plane_association"]["cluster_separation"]["enabled"]
                .as<bool>();
        seg.plane_association.cluster_separation.tolerance =
            mConfig["seg"]["plane_association"]["cluster_separation"]
                   ["tolerance"]
                       .as<float>();
        seg.plane_association.cluster_separation.downsample.leaf_size =
            mConfig["seg"]["plane_association"]["cluster_separation"]
                   ["downsample"]["leaf_size"]
                       .as<float>();
        seg.plane_association.cluster_separation.downsample
            .min_points_per_voxel =
            mConfig["seg"]["plane_association"]["cluster_separation"]
                   ["downsample"]["min_points_per_voxel"]
                       .as<unsigned int>();
        seg.ransac.max_planes =
            mConfig["seg"]["ransac"]["max_planes"].as<unsigned int>();
        seg.ransac.distance_thresh =
            mConfig["seg"]["ransac"]["distance_thresh"].as<float>();
        seg.ransac.max_iterations =
            mConfig["seg"]["ransac"]["max_iterations"].as<unsigned int>();

        // Optimization Parameters
        optimization.marginalize_planes =
            mConfig["optimization"]["marginalize_planes"].as<bool>();
        optimization.plane_kf.enabled =
            mConfig["optimization"]["plane_kf"]["enabled"].as<bool>();
        optimization.plane_kf.information_gain =
            mConfig["optimization"]["plane_kf"]["information_gain"].as<float>();
        optimization.plane_point.enabled =
            mConfig["optimization"]["plane_point"]["enabled"].as<bool>();
        optimization.plane_point.information_gain =
            mConfig["optimization"]["plane_point"]["information_gain"]
                .as<float>();
        optimization.plane_map_point.enabled =
            mConfig["optimization"]["plane_map_point"]["enabled"].as<bool>();
        optimization.plane_map_point.information_gain =
            mConfig["optimization"]["plane_map_point"]["information_gain"]
                .as<float>();

        // Geometric Segmentation Parameters
        geo_seg.pointcloud.downsample.leaf_size =
            mConfig["geo_seg"]["pointcloud"]["downsample"]["leaf_size"]
                .as<float>();
        geo_seg.pointcloud.downsample.min_points_per_voxel =
            mConfig["geo_seg"]["pointcloud"]["downsample"]
                   ["min_points_per_voxel"]
                       .as<unsigned int>();
        geo_seg.pointcloud.outlier_removal.std_threshold =
            mConfig["geo_seg"]["pointcloud"]["outlier_removal"]["std_threshold"]
                .as<float>();
        geo_seg.pointcloud.outlier_removal.mean_threshold =
            mConfig["geo_seg"]["pointcloud"]["outlier_removal"]
                   ["mean_threshold"]
                       .as<unsigned int>();

        // Semantic Segmentation Parameters
        sem_seg.min_votes     = mConfig["sem_seg"]["min_votes"].as<float>();
        sem_seg.prob_thresh   = mConfig["sem_seg"]["prob_thresh"].as<float>();
        sem_seg.conf_thresh   = mConfig["sem_seg"]["conf_thresh"].as<float>();
        sem_seg.max_tilt_wall = mConfig["sem_seg"]["max_tilt_wall"].as<float>();
        sem_seg.max_door_width =
            mConfig["sem_seg"]["max_door_width"].as<float>();
        sem_seg.max_door_height =
            mConfig["sem_seg"]["max_door_height"].as<float>();
        sem_seg.max_tilt_ground =
            mConfig["sem_seg"]["max_tilt_ground"].as<float>();
        sem_seg.passage_kf_window =
            mConfig["sem_seg"]["passage_kf_window"].as<int>();
        sem_seg.max_step_elevation =
            mConfig["sem_seg"]["max_step_elevation"].as<float>();
        sem_seg.max_wall_door_distance =
            mConfig["sem_seg"]["max_wall_door_distance"].as<float>();
        sem_seg.max_kf_passage_distance =
            mConfig["sem_seg"]["max_kf_passage_distance"].as<float>();
        sem_seg.enable_passage_detection =
            mConfig["sem_seg"]["enable_passage_detection"].as<bool>();
        sem_seg.passage_centroid_distance_thresh =
            mConfig["sem_seg"]["passage_centroid_distance_thresh"].as<float>();
        sem_seg.passageDetection.minimumSideDistance_m =
            mConfig["sem_seg"]["passage_detection"]["minimum_side_distance"]
                .as<float>();
        sem_seg.passageDetection.minimumEdgeNormalAlignment =
            mConfig["sem_seg"]["passage_detection"]
                   ["minimum_edge_normal_alignment"]
                       .as<float>();
        sem_seg.passageDetection.minimumOpeningRadius_m =
            mConfig["sem_seg"]["passage_detection"]["minimum_opening_radius"]
                .as<float>();
        sem_seg.passageDetection.wallBoundsMargin_m =
            mConfig["sem_seg"]["passage_detection"]["wall_bounds_margin"]
                .as<float>();
        sem_seg.passageDetection.duplicatePassageDistance_m =
            mConfig["sem_seg"]["passage_detection"]
                   ["duplicate_passage_distance"]
                       .as<float>();
        sem_seg.passageDetection.duplicateNormalAlignment =
            mConfig["sem_seg"]["passage_detection"]
                   ["duplicate_normal_alignment"]
                       .as<float>();
        sem_seg.passageDetection.ambiguousDuplicateNormalAlignment =
            mConfig["sem_seg"]["passage_detection"]
                   ["ambiguous_duplicate_normal_alignment"]
                       .as<float>();
        sem_seg.passageDetection.ambiguousDuplicatePlaneSeparation_m =
            mConfig["sem_seg"]["passage_detection"]
                   ["ambiguous_duplicate_plane_separation"]
                       .as<float>();
        sem_seg.passageDetection.crossingClusterDistance_m =
            mConfig["sem_seg"]["passage_detection"]["crossing_cluster_distance"]
                .as<float>();
        sem_seg.passageDetection.minimumConfirmationSnapshots =
            mConfig["sem_seg"]["passage_detection"]
                   ["minimum_confirmation_snapshots"]
                       .as<unsigned int>();
        sem_seg.passageDetection.minimumCrossingClusterSize =
            mConfig["sem_seg"]["passage_detection"]
                   ["minimum_crossing_cluster_size"]
                       .as<unsigned int>();
        sem_seg.passageDetection.maximumMissedSnapshots =
            mConfig["sem_seg"]["passage_detection"]["maximum_missed_snapshots"]
                .as<unsigned int>();
        sem_seg.passageDetection.minimumHorizontalFlankExtent_m =
            mConfig["sem_seg"]["passage_detection"]
                   ["minimum_horizontal_flank_extent"]
                       .as<float>();
        sem_seg.passageDetection.minimumHorizontalFlankPointCount =
            mConfig["sem_seg"]["passage_detection"]
                   ["minimum_horizontal_flank_points"]
                       .as<unsigned int>();
        sem_seg.pointcloud.downsample.leaf_size =
            mConfig["sem_seg"]["pointcloud"]["downsample"]["leaf_size"]
                .as<float>();
        sem_seg.pointcloud.downsample.min_points_per_voxel =
            mConfig["sem_seg"]["pointcloud"]["downsample"]
                   ["min_points_per_voxel"]
                       .as<unsigned int>();
        sem_seg.pointcloud.outlier_removal.std_threshold =
            mConfig["sem_seg"]["pointcloud"]["outlier_removal"]["std_threshold"]
                .as<float>();
        sem_seg.pointcloud.outlier_removal.mean_threshold =
            mConfig["sem_seg"]["pointcloud"]["outlier_removal"]
                   ["mean_threshold"]
                       .as<unsigned int>();
        sem_seg.wallCreation.minimumPointCount =
            mConfig["sem_seg"]["wall_creation"]["minimum_point_count"]
                .as<unsigned int>();
        sem_seg.wallCreation.minimumMajorExtent_m =
            mConfig["sem_seg"]["wall_creation"]["minimum_major_extent"]
                .as<float>();
        sem_seg.wallCreation.minimumMinorExtent_m =
            mConfig["sem_seg"]["wall_creation"]["minimum_minor_extent"]
                .as<float>();
        sem_seg.wallCreation.minimumArea_m2 =
            mConfig["sem_seg"]["wall_creation"]["minimum_area"].as<float>();
        sem_seg.wallCreation.connectivity.enabled =
            mConfig["sem_seg"]["wall_creation"]["connectivity"]["enabled"]
                .as<bool>();
        sem_seg.wallCreation.connectivity.clusterTolerance_m =
            mConfig["sem_seg"]["wall_creation"]["connectivity"]
                   ["cluster_tolerance"]
                       .as<float>();
        sem_seg.wallCreation.connectivity.minimumComponentPointCount =
            mConfig["sem_seg"]["wall_creation"]["connectivity"]
                   ["minimum_component_point_count"]
                       .as<unsigned int>();
        sem_seg.wallCreation.connectivity.minimumComponentRatio =
            mConfig["sem_seg"]["wall_creation"]["connectivity"]
                   ["minimum_component_ratio"]
                       .as<float>();
        sem_seg.wallPairing.minimumThickness_m =
            mConfig["sem_seg"]["wall_pairing"]["minimum_thickness"].as<float>();
        sem_seg.wallPairing.maximumThickness_m =
            mConfig["sem_seg"]["wall_pairing"]["maximum_thickness"].as<float>();
        sem_seg.wallPairing.minimumOverlapRatio =
            mConfig["sem_seg"]["wall_pairing"]["minimum_overlap_ratio"]
                .as<float>();
        sem_seg.reassociate.enabled =
            mConfig["sem_seg"]["reassociate"]["enabled"].as<bool>();
        sem_seg.reassociate.association_thresh =
            mConfig["sem_seg"]["reassociate"]["association_thresh"].as<float>();
        sem_seg.reassociate.wallExtension.enabled =
            mConfig["sem_seg"]["reassociate"]["wall_extension"]["enabled"]
                .as<bool>();
        sem_seg.reassociate.wallExtension.maximumInPlaneGap_m =
            mConfig["sem_seg"]["reassociate"]["wall_extension"]
                   ["maximum_in_plane_gap"]
                       .as<float>();
        sem_seg.reassociate.wallExtension.minimumOrthogonalOverlap_m =
            mConfig["sem_seg"]["reassociate"]["wall_extension"]
                   ["minimum_orthogonal_overlap"]
                       .as<float>();

        // Room Segmentation Parameters
        room_seg.gnn_version =
            mConfig["room_seg"]["gnn_based"]["gnn_version"].as<int>();
        room_seg.method = static_cast<room_seg::Method>(
            mConfig["room_seg"]["method"].as<int>());
        room_seg.walls_parallelism_thresh =
            mConfig["room_seg"]["parallelism_thresh"].as<float>();
        room_seg.center_distance_thresh =
            mConfig["room_seg"]["center_distance_thresh"].as<float>();
        room_seg.plane_facing_dot_thresh =
            mConfig["room_seg"]["plane_facing_dot_thresh"].as<float>();
        room_seg.min_wall_distance_thresh =
            mConfig["room_seg"]["min_wall_distance_thresh"].as<float>();
        room_seg.walls_perpendicularity_thresh =
            mConfig["room_seg"]["perpendicularity_thresh"].as<float>();
        room_seg.min_cluster_vertices =
            mConfig["room_seg"]["skeleton_based"]["min_cluster_vertices"]
                .as<unsigned int>();
        room_seg.marker_wall_distance_thresh =
            mConfig["room_seg"]["geo_based"]["marker_wall_distance_thresh"]
                .as<float>();
        room_seg.cluster_point_wall_distance_thresh =
            mConfig["room_seg"]["skeleton_based"]
                   ["cluster_point_wall_distance_thresh"]
                       .as<float>();
        room_seg.cluster_centroid_wall_centroid_distance_thresh =
            mConfig["room_seg"]["skeleton_based"]
                   ["cluster_centroid_wall_centroid_distance_thresh"]
                       .as<float>();
        room_seg.minimumWallSupportPointCount =
            mConfig["room_seg"]["skeleton_based"]["minimum_wall_support_points"]
                .as<unsigned int>();
        room_seg.minimumWallObservationCount =
            mConfig["room_seg"]["skeleton_based"]["minimum_wall_observations"]
                .as<unsigned int>();
        room_seg.minimumUndefendedWallHoldCycles =
            mConfig["room_seg"]["skeleton_based"]
                   ["minimum_undefended_wall_hold_cycles"]
                       .as<unsigned int>();
        room_seg.minimumWallSupportRatio =
            mConfig["room_seg"]["skeleton_based"]["minimum_wall_support_ratio"]
                .as<float>();
        room_seg.finiteWallBoundsMargin_m =
            mConfig["room_seg"]["skeleton_based"]["finite_wall_bounds_margin"]
                .as<float>();
        room_seg.minimumFiniteWallExtent_m =
            mConfig["room_seg"]["skeleton_based"]["minimum_finite_wall_extent"]
                .as<float>();
        room_seg.boundaryTopology.enabled =
            mConfig["room_seg"]["boundary_topology"]["enabled"].as<bool>();
        room_seg.boundaryTopology.minimumWallCount =
            mConfig["room_seg"]["boundary_topology"]["minimum_wall_count"]
                .as<unsigned int>();
        room_seg.boundaryTopology.minimumWallLength_m =
            mConfig["room_seg"]["boundary_topology"]["minimum_wall_length"]
                .as<float>();
        room_seg.boundaryTopology.maximumCornerGap_m =
            mConfig["room_seg"]["boundary_topology"]["maximum_corner_gap"]
                .as<float>();
        room_seg.boundaryTopology.maximumInteriorIntersection_m =
            mConfig["room_seg"]["boundary_topology"]
                   ["maximum_interior_intersection"]
                       .as<float>();
        room_seg.boundaryTopology.minimumEnclosedArea_m2 =
            mConfig["room_seg"]["boundary_topology"]["minimum_enclosed_area"]
                .as<float>();
        room_seg.boundaryTopology.endpointTrimRatio =
            mConfig["room_seg"]["boundary_topology"]["endpoint_trim_ratio"]
                .as<float>();
        room_seg.boundaryTopology.decisiveConflictSupportRatio =
            mConfig["room_seg"]["boundary_topology"]
                   ["decisive_conflict_support_ratio"]
                       .as<float>();
        room_seg.passagePartition.enabled =
            mConfig["room_seg"]["passage_partition"]["enabled"].as<bool>();
        room_seg.passagePartition.edgeVertexAssociationDistance_m =
            mConfig["room_seg"]["passage_partition"]
                   ["edge_vertex_association_distance"]
                       .as<float>();
        room_seg.passagePartition.minimumGraphCoverageRatio =
            mConfig["room_seg"]["passage_partition"]
                   ["minimum_graph_coverage_ratio"]
                       .as<float>();
        room_seg.passagePartition.openingMargin_m =
            mConfig["room_seg"]["passage_partition"]["opening_margin"]
                .as<float>();
        room_seg.passagePartition.minimumSideDistance_m =
            mConfig["room_seg"]["passage_partition"]["minimum_side_distance"]
                .as<float>();
        room_seg.passagePartition.detachWallsBeyondPassages =
            mConfig["room_seg"]["passage_partition"]
                   ["detach_walls_beyond_passages"]
                       .as<bool>();
        room_seg.passagePartition.wallCentroidMinimumSideDistance_m =
            mConfig["room_seg"]["passage_partition"]
                   ["wall_centroid_minimum_side_distance"]
                       .as<float>();

        // Map-merge and axiom thresholds.
        map_merge.passage_match_tolerance_m =
            mConfig["map_merge"]["passage_match_tolerance_m"].as<float>();
        map_merge.wall_coplanar_angle_deg =
            mConfig["map_merge"]["wall_coplanar_angle_deg"].as<float>();
        map_merge.wall_edge_overlap_m =
            mConfig["map_merge"]["wall_edge_overlap_m"].as<float>();
        map_merge.floor_match_tolerance_m =
            mConfig["map_merge"]["floor_match_tolerance_m"].as<float>();
        map_merge.observation_ray_check_cap =
            mConfig["map_merge"]["observation_ray_check_cap"]
                .as<unsigned int>();
        map_merge.merge_cooldown_s =
            mConfig["map_merge"]["merge_cooldown_s"].as<unsigned int>();
        map_merge.min_anchor_rooms =
            mConfig["map_merge"]["min_anchor_rooms"].as<unsigned int>();
        map_merge.min_rooms_per_map =
            mConfig["map_merge"]["min_rooms_per_map"].as<unsigned int>();
        map_merge.min_walls_per_map =
            mConfig["map_merge"]["min_walls_per_map"].as<unsigned int>();
        map_merge.room_centroid_tolerance_m =
            mConfig["map_merge"]["room_centroid_tolerance_m"].as<float>();

        // Room-Tracking State Machine Parameters (WP13 Section 18.4)
        room_tracking.crossing_dwell_s =
            mConfig["room_tracking"]["crossing_dwell_s"].as<float>();
        room_tracking.crossing_confidence =
            mConfig["room_tracking"]["crossing_confidence"].as<float>();
        room_tracking.lost_timeout_s =
            mConfig["room_tracking"]["lost_timeout_s"].as<float>();
        room_tracking.reacquire_timeout_s =
            mConfig["room_tracking"]["reacquire_timeout_s"].as<float>();
        room_tracking.reacquire_retry_interval_s =
            mConfig["room_tracking"]["reacquire_retry_interval_s"].as<float>();
        room_tracking.reacquire_max_retries =
            mConfig["room_tracking"]["reacquire_max_retries"]
                .as<unsigned int>();
        room_tracking.reacquire_min_planes =
            mConfig["room_tracking"]["reacquire_min_planes"].as<unsigned int>();

        this->candidate_gen.top_k =
            mConfig["candidate_gen"]["top_k"].as<unsigned int>();
        this->candidate_gen.candidate_pair_cap =
            mConfig["candidate_gen"]["candidate_pair_cap"].as<unsigned int>();
        this->candidate_gen.topology_nodes_cap =
            mConfig["candidate_gen"]["topology_nodes_cap"].as<unsigned int>();
        this->candidate_gen.global_fallback_cap =
            mConfig["candidate_gen"]["global_fallback_cap"].as<unsigned int>();
        this->candidate_gen.weight_angle =
            mConfig["candidate_gen"]["weight_angle"].as<float>();
        this->candidate_gen.weight_extent =
            mConfig["candidate_gen"]["weight_extent"].as<float>();
        this->candidate_gen.weight_aperture =
            mConfig["candidate_gen"]["weight_aperture"].as<float>();
        this->candidate_gen.weight_topology =
            mConfig["candidate_gen"]["weight_topology"].as<float>();
        this->candidate_gen.angle_missing_penalty =
            mConfig["candidate_gen"]["angle_missing_penalty"].as<float>();
        this->candidate_gen.extent_missing_penalty =
            mConfig["candidate_gen"]["extent_missing_penalty"].as<float>();
        this->candidate_gen.aperture_missing_penalty =
            mConfig["candidate_gen"]["aperture_missing_penalty"].as<float>();
        this->candidate_gen.ambiguity_margin =
            mConfig["candidate_gen"]["ambiguity_margin"].as<float>();
        this->candidate_gen.angle_tolerance_rad =
            mConfig["candidate_gen"]["angle_tolerance_rad"].as<float>();
        this->candidate_gen.runtime_budget_ms =
            mConfig["candidate_gen"]["runtime_budget_ms"].as<float>();
        this->candidate_gen.descriptor_elements_cap =
            mConfig["candidate_gen"]["descriptor_elements_cap"]
                .as<unsigned int>();
        this->candidate_gen.topo_refinement_iters =
            mConfig["candidate_gen"]["topo_refinement_iters"]
                .as<unsigned int>();

        this->verification.max_normal_angle_deg =
            mConfig["verification"]["max_normal_angle_deg"].as<float>();
        this->verification.max_offset_m =
            mConfig["verification"]["max_offset_m"].as<float>();
        this->verification.max_support_dist_m =
            mConfig["verification"]["max_support_dist_m"].as<float>();
        this->verification.min_inlier_ratio =
            mConfig["verification"]["min_inlier_ratio"].as<float>();
        this->verification.max_condition_number =
            mConfig["verification"]["max_condition_number"].as<float>();
        this->verification.ambiguity_margin_inliers =
            mConfig["verification"]["ambiguity_margin_inliers"]
                .as<unsigned int>();
        this->verification.max_walls_per_room =
            mConfig["verification"]["max_walls_per_room"].as<unsigned int>();
        this->verification.max_hypotheses =
            mConfig["verification"]["max_hypotheses"].as<unsigned int>();
        this->verification.max_support_sample_per_wall =
            mConfig["verification"]["max_support_sample_per_wall"]
                .as<unsigned int>();
        this->verification.min_abs_cos_normal_angle =
            mConfig["verification"]["min_abs_cos_normal_angle"].as<float>();

        this->factor.sigma_theta_rad =
            mConfig["factor"]["sigma_theta_rad"].as<float>();
        this->factor.sigma_offset_m =
            mConfig["factor"]["sigma_offset_m"].as<float>();
        this->factor.huber_delta = mConfig["factor"]["huber_delta"].as<float>();
        this->factor.optimizer_iterations =
            mConfig["factor"]["optimizer_iterations"].as<unsigned int>();

        const room_seg::BoundaryTopology &boundaryTopology =
            room_seg.boundaryTopology;

        const sem_seg::PassageDetection &passageDetection =
            sem_seg.passageDetection;
        const room_seg::PassagePartition &passagePartition =
            room_seg.passagePartition;

        if (passageDetection.minimumSideDistance_m <= 0.0F ||
            passageDetection.minimumEdgeNormalAlignment <= 0.0F ||
            passageDetection.minimumEdgeNormalAlignment > 1.0F ||
            passageDetection.minimumOpeningRadius_m <= 0.0F ||
            passageDetection.wallBoundsMargin_m < 0.0F ||
            passageDetection.duplicatePassageDistance_m <= 0.0F ||
            passageDetection.duplicateNormalAlignment <= 0.0F ||
            passageDetection.duplicateNormalAlignment > 1.0F ||
            passageDetection.ambiguousDuplicateNormalAlignment <= 0.0F ||
            passageDetection.ambiguousDuplicateNormalAlignment >
                passageDetection.duplicateNormalAlignment ||
            passageDetection.ambiguousDuplicatePlaneSeparation_m < 0.30F ||
            passageDetection.crossingClusterDistance_m <= 0.0F ||
            passageDetection.minimumConfirmationSnapshots < 2U ||
            passageDetection.minimumCrossingClusterSize < 1U ||
            passageDetection.minimumHorizontalFlankExtent_m <= 0.0F ||
            passageDetection.minimumHorizontalFlankPointCount < 1U ||
            passagePartition.edgeVertexAssociationDistance_m <= 0.0F ||
            passagePartition.minimumGraphCoverageRatio <= 0.0F ||
            passagePartition.minimumGraphCoverageRatio > 1.0F ||
            passagePartition.openingMargin_m < 0.0F ||
            passagePartition.minimumSideDistance_m < 0.0F ||
            passagePartition.wallCentroidMinimumSideDistance_m < 0.0F ||
            room_seg.minimumUndefendedWallHoldCycles < 1U ||
            boundaryTopology.minimumWallCount < 3U ||
            boundaryTopology.minimumWallLength_m <= 0.0F ||
            boundaryTopology.maximumCornerGap_m < 0.0F ||
            boundaryTopology.maximumInteriorIntersection_m < 0.0F ||
            boundaryTopology.minimumEnclosedArea_m2 <= 0.0F ||
            boundaryTopology.endpointTrimRatio < 0.0F ||
            boundaryTopology.endpointTrimRatio > 0.45F ||
            boundaryTopology.decisiveConflictSupportRatio < 1.0F ||
            room_tracking.crossing_dwell_s <= 0.0F ||
            room_tracking.crossing_confidence < 0.0F ||
            room_tracking.crossing_confidence > 1.0F ||
            !std::isfinite(room_tracking.crossing_dwell_s) ||
            !std::isfinite(room_tracking.crossing_confidence) ||
            room_tracking.lost_timeout_s <= 0.0F ||
            !std::isfinite(room_tracking.lost_timeout_s) ||
            room_tracking.reacquire_timeout_s <= 0.0F ||
            !std::isfinite(room_tracking.reacquire_timeout_s) ||
            room_tracking.reacquire_retry_interval_s <= 0.0F ||
            !std::isfinite(room_tracking.reacquire_retry_interval_s) ||
            room_tracking.reacquire_max_retries < 1U ||
            room_tracking.reacquire_min_planes < 1U ||
            this->candidate_gen.top_k < 1U ||
            this->candidate_gen.candidate_pair_cap < 1U ||
            this->candidate_gen.topology_nodes_cap < 1U ||
            this->candidate_gen.global_fallback_cap < 1U ||
            this->candidate_gen.top_k >
                this->candidate_gen.candidate_pair_cap ||
            this->candidate_gen.global_fallback_cap >
                this->candidate_gen.candidate_pair_cap ||
            this->candidate_gen.descriptor_elements_cap < 1U ||
            this->candidate_gen.topo_refinement_iters < 1U ||
            this->candidate_gen.weight_angle < 0.0F ||
            this->candidate_gen.weight_extent < 0.0F ||
            this->candidate_gen.weight_aperture < 0.0F ||
            this->candidate_gen.weight_topology < 0.0F ||
            this->candidate_gen.ambiguity_margin < 0.0F ||
            this->candidate_gen.angle_missing_penalty < 0.0F ||
            this->candidate_gen.extent_missing_penalty < 0.0F ||
            this->candidate_gen.aperture_missing_penalty < 0.0F ||
            this->candidate_gen.angle_tolerance_rad < 0.0F ||
            this->candidate_gen.runtime_budget_ms < 0.0F ||
            !std::isfinite(this->candidate_gen.weight_angle) ||
            !std::isfinite(this->candidate_gen.weight_extent) ||
            !std::isfinite(this->candidate_gen.weight_aperture) ||
            !std::isfinite(this->candidate_gen.weight_topology) ||
            !std::isfinite(this->candidate_gen.angle_missing_penalty) ||
            !std::isfinite(this->candidate_gen.extent_missing_penalty) ||
            !std::isfinite(this->candidate_gen.aperture_missing_penalty) ||
            !std::isfinite(this->candidate_gen.ambiguity_margin) ||
            !std::isfinite(this->candidate_gen.angle_tolerance_rad) ||
            !std::isfinite(this->candidate_gen.runtime_budget_ms) ||
            (this->candidate_gen.weight_angle == 0.0F &&
             this->candidate_gen.weight_extent == 0.0F &&
             this->candidate_gen.weight_aperture == 0.0F &&
             this->candidate_gen.weight_topology == 0.0F) ||
            this->verification.max_normal_angle_deg <= 0.0F ||
            this->verification.max_offset_m <= 0.0F ||
            this->verification.max_support_dist_m <= 0.0F ||
            this->verification.min_inlier_ratio <= 0.0F ||
            this->verification.min_inlier_ratio > 1.0F ||
            this->verification.max_condition_number <= 0.0F ||
            this->verification.max_walls_per_room < 3U ||
            this->verification.max_hypotheses < 1U ||
            this->verification.max_support_sample_per_wall < 1U ||
            this->verification.min_abs_cos_normal_angle <= 0.0F ||
            this->verification.min_abs_cos_normal_angle > 1.0F ||
            !std::isfinite(this->verification.max_normal_angle_deg) ||
            !std::isfinite(this->verification.max_offset_m) ||
            !std::isfinite(this->verification.max_support_dist_m) ||
            !std::isfinite(this->verification.min_inlier_ratio) ||
            !std::isfinite(this->verification.max_condition_number) ||
            !std::isfinite(this->verification.min_abs_cos_normal_angle) ||
            this->factor.sigma_theta_rad <= 0.0F ||
            this->factor.sigma_offset_m <= 0.0F ||
            this->factor.huber_delta <= 0.0F ||
            this->factor.optimizer_iterations < 1U ||
            !std::isfinite(this->factor.sigma_theta_rad) ||
            !std::isfinite(this->factor.sigma_offset_m) ||
            !std::isfinite(this->factor.huber_delta))
        {
            throw std::invalid_argument(
                "passage, room-topology or room-tracking configuration "
                "contains an invalid value");
        }
    }
    catch (YAML::Exception &e)
    {
        VSLAM_LOG_ERROR("Error loading system parameters. Make sure all "
                        "parameters are defined properly: %s",
                        e.what());
        exit(1);
    }
    catch (const std::invalid_argument &exception)
    {
        VSLAM_LOG_ERROR("Error loading system parameters: %s",
                        exception.what());
        exit(1);
    }
}
} // namespace ORB_SLAM3
