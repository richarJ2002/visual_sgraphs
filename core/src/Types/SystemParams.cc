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

namespace vs_graphs
{
namespace core
{
namespace types
{
SystemParams *SystemParams::p_systemParams = nullptr;

SystemParams::SystemParams()
{
    p_systemParams = nullptr;
}

SystemParams *SystemParams::getParams()
{
    if (p_systemParams == nullptr)
        p_systemParams = new SystemParams();
    return p_systemParams;
}

void SystemParams::setParams(const std::string &configFilePath_in)
{
    VSLAM_LOG_INFO("[SysParams] Loading system parameters from %s",
                   configFilePath_in.c_str());
    try
    {
        config = YAML::LoadFile(configFilePath_in);
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
        general.envDatabase =
            config["general"]["env_database"].as<std::string>();
        general.modeOfOperation = static_cast<General::ModeOfOperation>(
            config["general"]["mode_of_operation"].as<int>());

        // Marker Parameters
        markers.impact = config["markers"]["impact"].as<float>();

        // Tracking Refinement Parameters
        refineMapPoints.enabled =
            config["refine_map_points"]["enabled"].as<bool>();
        refineMapPoints.maxDistanceForDelete =
            config["refine_map_points"]["max_distance_for_delete"].as<float>();
        refineMapPoints.octree.resolution =
            config["refine_map_points"]["octree"]["resolution"].as<float>();
        refineMapPoints.octree.searchRadius =
            config["refine_map_points"]["octree"]["search_radius"].as<float>();
        refineMapPoints.octree.minNeighbors =
            config["refine_map_points"]["octree"]["min_neighbors"]
                .as<unsigned int>();

        // Plane based Covisibility Parameters
        planeBasedCovisibility.enabled =
            config["plane_based_covisibility"]["enabled"].as<bool>();
        planeBasedCovisibility.maxKeyframes =
            config["plane_based_covisibility"]["max_keyframes"]
                .as<unsigned int>();
        planeBasedCovisibility.scorePerPlane =
            config["plane_based_covisibility"]["score_per_plane"]
                .as<unsigned int>();

        // Common Segmentation Parameters
        seg.pointcloudsThresh =
            config["seg"]["pointclouds_thresh"].as<unsigned int>();
        seg.planePointDistThresh =
            config["seg"]["plane_point_dist_thresh"].as<float>();
        seg.planeAssociation.ominusThresh =
            config["seg"]["plane_association"]["ominus_thresh"].as<float>();
        seg.planeAssociation.distanceThresh =
            config["seg"]["plane_association"]["distance_thresh"].as<float>();
        seg.planeAssociation.centroidThresh =
            config["seg"]["plane_association"]["centroid_thresh"].as<float>();
        seg.planeAssociation.clusterSeparation.enabled =
            config["seg"]["plane_association"]["cluster_separation"]["enabled"]
                .as<bool>();
        seg.planeAssociation.clusterSeparation.tolerance =
            config["seg"]["plane_association"]["cluster_separation"]
                   ["tolerance"]
                       .as<float>();
        seg.planeAssociation.clusterSeparation.downsample.leafSize =
            config["seg"]["plane_association"]["cluster_separation"]
                   ["downsample"]["leaf_size"]
                       .as<float>();
        seg.planeAssociation.clusterSeparation.downsample
            .minPointsPerVoxel =
            config["seg"]["plane_association"]["cluster_separation"]
                   ["downsample"]["min_points_per_voxel"]
                       .as<unsigned int>();
        seg.ransac.maxPlanes =
            config["seg"]["ransac"]["max_planes"].as<unsigned int>();
        seg.ransac.distanceThresh =
            config["seg"]["ransac"]["distance_thresh"].as<float>();
        seg.ransac.maxIterations =
            config["seg"]["ransac"]["max_iterations"].as<unsigned int>();

        // Optimization Parameters
        optimization.marginalizePlanes =
            config["optimization"]["marginalize_planes"].as<bool>();
        optimization.planeKf.enabled =
            config["optimization"]["plane_kf"]["enabled"].as<bool>();
        optimization.planeKf.informationGain =
            config["optimization"]["plane_kf"]["information_gain"].as<float>();
        optimization.planePoint.enabled =
            config["optimization"]["plane_point"]["enabled"].as<bool>();
        optimization.planePoint.informationGain =
            config["optimization"]["plane_point"]["information_gain"]
                .as<float>();
        optimization.planeMapPoint.enabled =
            config["optimization"]["plane_map_point"]["enabled"].as<bool>();
        optimization.planeMapPoint.informationGain =
            config["optimization"]["plane_map_point"]["information_gain"]
                .as<float>();

        // Geometric Segmentation Parameters
        geoSeg.pointcloud.downsample.leafSize =
            config["geo_seg"]["pointcloud"]["downsample"]["leaf_size"]
                .as<float>();
        geoSeg.pointcloud.downsample.minPointsPerVoxel =
            config["geo_seg"]["pointcloud"]["downsample"]
                   ["min_points_per_voxel"]
                       .as<unsigned int>();
        geoSeg.pointcloud.outlierRemoval.stdThreshold =
            config["geo_seg"]["pointcloud"]["outlier_removal"]["std_threshold"]
                .as<float>();
        geoSeg.pointcloud.outlierRemoval.meanThreshold =
            config["geo_seg"]["pointcloud"]["outlier_removal"]
                   ["mean_threshold"]
                       .as<unsigned int>();

        // Semantic Segmentation Parameters
        semSeg.minVotes     = config["sem_seg"]["min_votes"].as<float>();
        semSeg.probThresh   = config["sem_seg"]["prob_thresh"].as<float>();
        semSeg.confThresh   = config["sem_seg"]["conf_thresh"].as<float>();
        semSeg.maxTiltWall = config["sem_seg"]["max_tilt_wall"].as<float>();
        semSeg.maxDoorWidth =
            config["sem_seg"]["max_door_width"].as<float>();
        semSeg.maxDoorHeight =
            config["sem_seg"]["max_door_height"].as<float>();
        semSeg.maxTiltGround =
            config["sem_seg"]["max_tilt_ground"].as<float>();
        semSeg.passageKfWindow =
            config["sem_seg"]["passage_kf_window"].as<int>();
        semSeg.maxStepElevation =
            config["sem_seg"]["max_step_elevation"].as<float>();
        semSeg.maxWallDoorDistance =
            config["sem_seg"]["max_wall_door_distance"].as<float>();
        semSeg.maxKfPassageDistance =
            config["sem_seg"]["max_kf_passage_distance"].as<float>();
        semSeg.enablePassageDetection =
            config["sem_seg"]["enable_passage_detection"].as<bool>();
        semSeg.passageCentroidDistanceThresh =
            config["sem_seg"]["passage_centroid_distance_thresh"].as<float>();
        semSeg.passageDetection.minimumSideDistance_m =
            config["sem_seg"]["passage_detection"]["minimum_side_distance"]
                .as<float>();
        semSeg.passageDetection.minimumEdgeNormalAlignment =
            config["sem_seg"]["passage_detection"]
                   ["minimum_edge_normal_alignment"]
                       .as<float>();
        semSeg.passageDetection.minimumOpeningRadius_m =
            config["sem_seg"]["passage_detection"]["minimum_opening_radius"]
                .as<float>();
        semSeg.passageDetection.wallBoundsMargin_m =
            config["sem_seg"]["passage_detection"]["wall_bounds_margin"]
                .as<float>();
        semSeg.passageDetection.duplicatePassageDistance_m =
            config["sem_seg"]["passage_detection"]
                   ["duplicate_passage_distance"]
                       .as<float>();
        semSeg.passageDetection.duplicateNormalAlignment =
            config["sem_seg"]["passage_detection"]
                   ["duplicate_normal_alignment"]
                       .as<float>();
        semSeg.passageDetection.ambiguousDuplicateNormalAlignment =
            config["sem_seg"]["passage_detection"]
                   ["ambiguous_duplicate_normal_alignment"]
                       .as<float>();
        semSeg.passageDetection.ambiguousDuplicatePlaneSeparation_m =
            config["sem_seg"]["passage_detection"]
                   ["ambiguous_duplicate_plane_separation"]
                       .as<float>();
        semSeg.passageDetection.crossingClusterDistance_m =
            config["sem_seg"]["passage_detection"]["crossing_cluster_distance"]
                .as<float>();
        semSeg.passageDetection.minimumConfirmationSnapshots =
            config["sem_seg"]["passage_detection"]
                   ["minimum_confirmation_snapshots"]
                       .as<unsigned int>();
        semSeg.passageDetection.minimumCrossingClusterSize =
            config["sem_seg"]["passage_detection"]
                   ["minimum_crossing_cluster_size"]
                       .as<unsigned int>();
        semSeg.passageDetection.maximumMissedSnapshots =
            config["sem_seg"]["passage_detection"]["maximum_missed_snapshots"]
                .as<unsigned int>();
        semSeg.passageDetection.minimumHorizontalFlankExtent_m =
            config["sem_seg"]["passage_detection"]
                   ["minimum_horizontal_flank_extent"]
                       .as<float>();
        semSeg.passageDetection.minimumHorizontalFlankPointCount =
            config["sem_seg"]["passage_detection"]
                   ["minimum_horizontal_flank_points"]
                       .as<unsigned int>();
        semSeg.pointcloud.downsample.leafSize =
            config["sem_seg"]["pointcloud"]["downsample"]["leaf_size"]
                .as<float>();
        semSeg.pointcloud.downsample.minPointsPerVoxel =
            config["sem_seg"]["pointcloud"]["downsample"]
                   ["min_points_per_voxel"]
                       .as<unsigned int>();
        semSeg.pointcloud.outlierRemoval.stdThreshold =
            config["sem_seg"]["pointcloud"]["outlier_removal"]["std_threshold"]
                .as<float>();
        semSeg.pointcloud.outlierRemoval.meanThreshold =
            config["sem_seg"]["pointcloud"]["outlier_removal"]
                   ["mean_threshold"]
                       .as<unsigned int>();
        semSeg.wallCreation.minimumPointCount =
            config["sem_seg"]["wall_creation"]["minimum_point_count"]
                .as<unsigned int>();
        semSeg.wallCreation.minimumMajorExtent_m =
            config["sem_seg"]["wall_creation"]["minimum_major_extent"]
                .as<float>();
        semSeg.wallCreation.minimumMinorExtent_m =
            config["sem_seg"]["wall_creation"]["minimum_minor_extent"]
                .as<float>();
        semSeg.wallCreation.minimumArea_m2 =
            config["sem_seg"]["wall_creation"]["minimum_area"].as<float>();
        semSeg.wallCreation.connectivity.enabled =
            config["sem_seg"]["wall_creation"]["connectivity"]["enabled"]
                .as<bool>();
        semSeg.wallCreation.connectivity.clusterTolerance_m =
            config["sem_seg"]["wall_creation"]["connectivity"]
                   ["cluster_tolerance"]
                       .as<float>();
        semSeg.wallCreation.connectivity.minimumComponentPointCount =
            config["sem_seg"]["wall_creation"]["connectivity"]
                   ["minimum_component_point_count"]
                       .as<unsigned int>();
        semSeg.wallCreation.connectivity.minimumComponentRatio =
            config["sem_seg"]["wall_creation"]["connectivity"]
                   ["minimum_component_ratio"]
                       .as<float>();
        semSeg.wallPairing.minimumThickness_m =
            config["sem_seg"]["wall_pairing"]["minimum_thickness"].as<float>();
        semSeg.wallPairing.maximumThickness_m =
            config["sem_seg"]["wall_pairing"]["maximum_thickness"].as<float>();
        semSeg.wallPairing.minimumOverlapRatio =
            config["sem_seg"]["wall_pairing"]["minimum_overlap_ratio"]
                .as<float>();
        semSeg.reassociate.enabled =
            config["sem_seg"]["reassociate"]["enabled"].as<bool>();
        semSeg.reassociate.associationThresh =
            config["sem_seg"]["reassociate"]["association_thresh"].as<float>();
        semSeg.reassociate.wallExtension.enabled =
            config["sem_seg"]["reassociate"]["wall_extension"]["enabled"]
                .as<bool>();
        semSeg.reassociate.wallExtension.maximumInPlaneGap_m =
            config["sem_seg"]["reassociate"]["wall_extension"]
                   ["maximum_in_plane_gap"]
                       .as<float>();
        semSeg.reassociate.wallExtension.minimumOrthogonalOverlap_m =
            config["sem_seg"]["reassociate"]["wall_extension"]
                   ["minimum_orthogonal_overlap"]
                       .as<float>();

        // Room Segmentation Parameters
        roomSeg.gnnVersion =
            config["room_seg"]["gnn_based"]["gnn_version"].as<int>();
        roomSeg.method = static_cast<RoomSeg::Method>(
            config["room_seg"]["method"].as<int>());
        roomSeg.wallsParallelismThresh =
            config["room_seg"]["parallelism_thresh"].as<float>();
        roomSeg.centerDistanceThresh =
            config["room_seg"]["center_distance_thresh"].as<float>();
        roomSeg.planeFacingDotThresh =
            config["room_seg"]["plane_facing_dot_thresh"].as<float>();
        roomSeg.minWallDistanceThresh =
            config["room_seg"]["min_wall_distance_thresh"].as<float>();
        roomSeg.wallsPerpendicularityThresh =
            config["room_seg"]["perpendicularity_thresh"].as<float>();
        roomSeg.minClusterVertices =
            config["room_seg"]["skeleton_based"]["min_cluster_vertices"]
                .as<unsigned int>();
        roomSeg.markerWallDistanceThresh =
            config["room_seg"]["geo_based"]["marker_wall_distance_thresh"]
                .as<float>();
        roomSeg.clusterPointWallDistanceThresh =
            config["room_seg"]["skeleton_based"]
                   ["cluster_point_wall_distance_thresh"]
                       .as<float>();
        roomSeg.clusterCentroidWallCentroidDistanceThresh =
            config["room_seg"]["skeleton_based"]
                   ["cluster_centroid_wall_centroid_distance_thresh"]
                       .as<float>();
        roomSeg.minimumWallSupportPointCount =
            config["room_seg"]["skeleton_based"]["minimum_wall_support_points"]
                .as<unsigned int>();
        roomSeg.minimumWallObservationCount =
            config["room_seg"]["skeleton_based"]["minimum_wall_observations"]
                .as<unsigned int>();
        roomSeg.minimumUndefendedWallHoldCycles =
            config["room_seg"]["skeleton_based"]
                   ["minimum_undefended_wall_hold_cycles"]
                       .as<unsigned int>();
        roomSeg.minimumWallSupportRatio =
            config["room_seg"]["skeleton_based"]["minimum_wall_support_ratio"]
                .as<float>();
        roomSeg.finiteWallBoundsMargin_m =
            config["room_seg"]["skeleton_based"]["finite_wall_bounds_margin"]
                .as<float>();
        roomSeg.minimumFiniteWallExtent_m =
            config["room_seg"]["skeleton_based"]["minimum_finite_wall_extent"]
                .as<float>();
        roomSeg.boundaryTopology.enabled =
            config["room_seg"]["boundary_topology"]["enabled"].as<bool>();
        roomSeg.boundaryTopology.minimumWallCount =
            config["room_seg"]["boundary_topology"]["minimum_wall_count"]
                .as<unsigned int>();
        roomSeg.boundaryTopology.minimumWallLength_m =
            config["room_seg"]["boundary_topology"]["minimum_wall_length"]
                .as<float>();
        roomSeg.boundaryTopology.maximumCornerGap_m =
            config["room_seg"]["boundary_topology"]["maximum_corner_gap"]
                .as<float>();
        roomSeg.boundaryTopology.maximumInteriorIntersection_m =
            config["room_seg"]["boundary_topology"]
                   ["maximum_interior_intersection"]
                       .as<float>();
        roomSeg.boundaryTopology.minimumEnclosedArea_m2 =
            config["room_seg"]["boundary_topology"]["minimum_enclosed_area"]
                .as<float>();
        roomSeg.boundaryTopology.endpointTrimRatio =
            config["room_seg"]["boundary_topology"]["endpoint_trim_ratio"]
                .as<float>();
        roomSeg.boundaryTopology.decisiveConflictSupportRatio =
            config["room_seg"]["boundary_topology"]
                   ["decisive_conflict_support_ratio"]
                       .as<float>();
        roomSeg.passagePartition.enabled =
            config["room_seg"]["passage_partition"]["enabled"].as<bool>();
        roomSeg.passagePartition.edgeVertexAssociationDistance_m =
            config["room_seg"]["passage_partition"]
                   ["edge_vertex_association_distance"]
                       .as<float>();
        roomSeg.passagePartition.minimumGraphCoverageRatio =
            config["room_seg"]["passage_partition"]
                   ["minimum_graph_coverage_ratio"]
                       .as<float>();
        roomSeg.passagePartition.openingMargin_m =
            config["room_seg"]["passage_partition"]["opening_margin"]
                .as<float>();
        roomSeg.passagePartition.minimumSideDistance_m =
            config["room_seg"]["passage_partition"]["minimum_side_distance"]
                .as<float>();
        roomSeg.passagePartition.detachWallsBeyondPassages =
            config["room_seg"]["passage_partition"]
                   ["detach_walls_beyond_passages"]
                       .as<bool>();
        roomSeg.passagePartition.wallCentroidMinimumSideDistance_m =
            config["room_seg"]["passage_partition"]
                   ["wall_centroid_minimum_side_distance"]
                       .as<float>();

        // Map-merge and axiom thresholds.
        mapMerge.passageMatchTolerance_m =
            config["map_merge"]["passage_match_tolerance_m"].as<float>();
        mapMerge.wallCoplanarAngle_deg =
            config["map_merge"]["wall_coplanar_angle_deg"].as<float>();
        mapMerge.wallEdgeOverlap_m =
            config["map_merge"]["wall_edge_overlap_m"].as<float>();
        mapMerge.floorMatchTolerance_m =
            config["map_merge"]["floor_match_tolerance_m"].as<float>();
        mapMerge.observationRayCheckCap =
            config["map_merge"]["observation_ray_check_cap"]
                .as<unsigned int>();
        mapMerge.mergeCooldown_s =
            config["map_merge"]["merge_cooldown_s"].as<unsigned int>();
        mapMerge.minAnchorRooms =
            config["map_merge"]["min_anchor_rooms"].as<unsigned int>();
        mapMerge.minRoomsPerMap =
            config["map_merge"]["min_rooms_per_map"].as<unsigned int>();
        mapMerge.minWallsPerMap =
            config["map_merge"]["min_walls_per_map"].as<unsigned int>();
        mapMerge.roomCentroidTolerance_m =
            config["map_merge"]["room_centroid_tolerance_m"].as<float>();

        // Room-Tracking State Machine Parameters
        roomTracking.crossingDwell_s =
            config["room_tracking"]["crossing_dwell_s"].as<float>();
        roomTracking.crossingConfidence =
            config["room_tracking"]["crossing_confidence"].as<float>();
        roomTracking.lostTimeout_s =
            config["room_tracking"]["lost_timeout_s"].as<float>();
        roomTracking.reacquireTimeout_s =
            config["room_tracking"]["reacquire_timeout_s"].as<float>();
        roomTracking.reacquireRetryInterval_s =
            config["room_tracking"]["reacquire_retry_interval_s"].as<float>();
        roomTracking.reacquireMaxRetries =
            config["room_tracking"]["reacquire_max_retries"]
                .as<unsigned int>();
        roomTracking.reacquireMinPlanes =
            config["room_tracking"]["reacquire_min_planes"].as<unsigned int>();

        this->candidateGen.topK =
            config["candidate_gen"]["top_k"].as<unsigned int>();
        this->candidateGen.candidatePairCap =
            config["candidate_gen"]["candidate_pair_cap"].as<unsigned int>();
        this->candidateGen.topologyNodesCap =
            config["candidate_gen"]["topology_nodes_cap"].as<unsigned int>();
        this->candidateGen.globalFallbackCap =
            config["candidate_gen"]["global_fallback_cap"].as<unsigned int>();
        this->candidateGen.weightAngle =
            config["candidate_gen"]["weight_angle"].as<float>();
        this->candidateGen.weightExtent =
            config["candidate_gen"]["weight_extent"].as<float>();
        this->candidateGen.weightAperture =
            config["candidate_gen"]["weight_aperture"].as<float>();
        this->candidateGen.weightTopology =
            config["candidate_gen"]["weight_topology"].as<float>();
        this->candidateGen.angleMissingPenalty =
            config["candidate_gen"]["angle_missing_penalty"].as<float>();
        this->candidateGen.extentMissingPenalty =
            config["candidate_gen"]["extent_missing_penalty"].as<float>();
        this->candidateGen.apertureMissingPenalty =
            config["candidate_gen"]["aperture_missing_penalty"].as<float>();
        this->candidateGen.ambiguityMargin =
            config["candidate_gen"]["ambiguity_margin"].as<float>();
        this->candidateGen.angleTolerance_rad =
            config["candidate_gen"]["angle_tolerance_rad"].as<float>();
        this->candidateGen.runtimeBudget_ms =
            config["candidate_gen"]["runtime_budget_ms"].as<float>();
        this->candidateGen.descriptorElementsCap =
            config["candidate_gen"]["descriptor_elements_cap"]
                .as<unsigned int>();
        this->candidateGen.topoRefinementIters =
            config["candidate_gen"]["topo_refinement_iters"]
                .as<unsigned int>();

        this->verification.maxNormalAngle_deg =
            config["verification"]["max_normal_angle_deg"].as<float>();
        this->verification.maxOffset_m =
            config["verification"]["max_offset_m"].as<float>();
        this->verification.maxSupportDist_m =
            config["verification"]["max_support_dist_m"].as<float>();
        this->verification.minInlierRatio =
            config["verification"]["min_inlier_ratio"].as<float>();
        this->verification.maxConditionNumber =
            config["verification"]["max_condition_number"].as<float>();
        this->verification.ambiguityMarginInliers =
            config["verification"]["ambiguity_margin_inliers"]
                .as<unsigned int>();
        this->verification.maxWallsPerRoom =
            config["verification"]["max_walls_per_room"].as<unsigned int>();
        this->verification.maxHypotheses =
            config["verification"]["max_hypotheses"].as<unsigned int>();
        this->verification.maxSupportSamplePerWall =
            config["verification"]["max_support_sample_per_wall"]
                .as<unsigned int>();
        this->verification.minAbsCosNormalAngle =
            config["verification"]["min_abs_cos_normal_angle"].as<float>();

        this->factor.sigmaTheta_rad =
            config["factor"]["sigma_theta_rad"].as<float>();
        this->factor.sigmaOffset_m =
            config["factor"]["sigma_offset_m"].as<float>();
        this->factor.huberDelta = config["factor"]["huber_delta"].as<float>();
        this->factor.optimizerIterations =
            config["factor"]["optimizer_iterations"].as<unsigned int>();

        const RoomSeg::BoundaryTopology &boundaryTopology =
            roomSeg.boundaryTopology;

        const SemSeg::PassageDetection &passageDetection =
            semSeg.passageDetection;
        const RoomSeg::PassagePartition &passagePartition =
            roomSeg.passagePartition;

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
            roomSeg.minimumUndefendedWallHoldCycles < 1U ||
            boundaryTopology.minimumWallCount < 3U ||
            boundaryTopology.minimumWallLength_m <= 0.0F ||
            boundaryTopology.maximumCornerGap_m < 0.0F ||
            boundaryTopology.maximumInteriorIntersection_m < 0.0F ||
            boundaryTopology.minimumEnclosedArea_m2 <= 0.0F ||
            boundaryTopology.endpointTrimRatio < 0.0F ||
            boundaryTopology.endpointTrimRatio > 0.45F ||
            boundaryTopology.decisiveConflictSupportRatio < 1.0F ||
            roomTracking.crossingDwell_s <= 0.0F ||
            roomTracking.crossingConfidence < 0.0F ||
            roomTracking.crossingConfidence > 1.0F ||
            !std::isfinite(roomTracking.crossingDwell_s) ||
            !std::isfinite(roomTracking.crossingConfidence) ||
            roomTracking.lostTimeout_s <= 0.0F ||
            !std::isfinite(roomTracking.lostTimeout_s) ||
            roomTracking.reacquireTimeout_s <= 0.0F ||
            !std::isfinite(roomTracking.reacquireTimeout_s) ||
            roomTracking.reacquireRetryInterval_s <= 0.0F ||
            !std::isfinite(roomTracking.reacquireRetryInterval_s) ||
            roomTracking.reacquireMaxRetries < 1U ||
            roomTracking.reacquireMinPlanes < 1U ||
            this->candidateGen.topK < 1U ||
            this->candidateGen.candidatePairCap < 1U ||
            this->candidateGen.topologyNodesCap < 1U ||
            this->candidateGen.globalFallbackCap < 1U ||
            this->candidateGen.topK >
                this->candidateGen.candidatePairCap ||
            this->candidateGen.globalFallbackCap >
                this->candidateGen.candidatePairCap ||
            this->candidateGen.descriptorElementsCap < 1U ||
            this->candidateGen.topoRefinementIters < 1U ||
            this->candidateGen.weightAngle < 0.0F ||
            this->candidateGen.weightExtent < 0.0F ||
            this->candidateGen.weightAperture < 0.0F ||
            this->candidateGen.weightTopology < 0.0F ||
            this->candidateGen.ambiguityMargin < 0.0F ||
            this->candidateGen.angleMissingPenalty < 0.0F ||
            this->candidateGen.extentMissingPenalty < 0.0F ||
            this->candidateGen.apertureMissingPenalty < 0.0F ||
            this->candidateGen.angleTolerance_rad < 0.0F ||
            this->candidateGen.runtimeBudget_ms < 0.0F ||
            !std::isfinite(this->candidateGen.weightAngle) ||
            !std::isfinite(this->candidateGen.weightExtent) ||
            !std::isfinite(this->candidateGen.weightAperture) ||
            !std::isfinite(this->candidateGen.weightTopology) ||
            !std::isfinite(this->candidateGen.angleMissingPenalty) ||
            !std::isfinite(this->candidateGen.extentMissingPenalty) ||
            !std::isfinite(this->candidateGen.apertureMissingPenalty) ||
            !std::isfinite(this->candidateGen.ambiguityMargin) ||
            !std::isfinite(this->candidateGen.angleTolerance_rad) ||
            !std::isfinite(this->candidateGen.runtimeBudget_ms) ||
            (this->candidateGen.weightAngle == 0.0F &&
             this->candidateGen.weightExtent == 0.0F &&
             this->candidateGen.weightAperture == 0.0F &&
             this->candidateGen.weightTopology == 0.0F) ||
            this->verification.maxNormalAngle_deg <= 0.0F ||
            this->verification.maxOffset_m <= 0.0F ||
            this->verification.maxSupportDist_m <= 0.0F ||
            this->verification.minInlierRatio <= 0.0F ||
            this->verification.minInlierRatio > 1.0F ||
            this->verification.maxConditionNumber <= 0.0F ||
            this->verification.maxWallsPerRoom < 3U ||
            this->verification.maxHypotheses < 1U ||
            this->verification.maxSupportSamplePerWall < 1U ||
            this->verification.minAbsCosNormalAngle <= 0.0F ||
            this->verification.minAbsCosNormalAngle > 1.0F ||
            !std::isfinite(this->verification.maxNormalAngle_deg) ||
            !std::isfinite(this->verification.maxOffset_m) ||
            !std::isfinite(this->verification.maxSupportDist_m) ||
            !std::isfinite(this->verification.minInlierRatio) ||
            !std::isfinite(this->verification.maxConditionNumber) ||
            !std::isfinite(this->verification.minAbsCosNormalAngle) ||
            this->factor.sigmaTheta_rad <= 0.0F ||
            this->factor.sigmaOffset_m <= 0.0F ||
            this->factor.huberDelta <= 0.0F ||
            this->factor.optimizerIterations < 1U ||
            !std::isfinite(this->factor.sigmaTheta_rad) ||
            !std::isfinite(this->factor.sigmaOffset_m) ||
            !std::isfinite(this->factor.huberDelta))
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
} // namespace types
} // namespace core
} // namespace vs_graphs
