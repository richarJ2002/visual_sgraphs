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

/*!
 * @file            SystemParams.h
 *
 * @brief           Declares the SystemParams YAML-backed calibration store.
 *
 *  @note           WP-01C/WP-04/WP-05 deferred by pilot scope (§17.4): member
 *                  identifiers (snake_case, m_ prefix, Get/Set PascalCase),
 *                  unscoped enums without underlying types, raw-new singleton,
 *                  and void-plus-exit(1) failure handling stay as-is here.
 *                  Renaming or status-converting them cascades to consumers
 *                  repo-wide (ROS/YAML keys frozen per §16.3) and needs the
 *                  group-level WP-02/WP-04 step with maintainer sign-off.
 */

#include <iostream>
#include <yaml-cpp/yaml.h>

#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

namespace vs_graphs
{
namespace core
{
namespace types
{
class SystemParams
{
  public:
    static SystemParams *GetParams();
    void                 SetParams(const std::string &strConfigFile);

    // Common struct definitions
    struct Constraint
    {
        bool  enabled          = false;
        float information_gain = 0.1f;
    };
    struct Downsample
    {
        float        leaf_size            = 0.03f;
        unsigned int min_points_per_voxel = 5;
    };
    struct OutlierRemoval
    {
        float        std_threshold  = 1.0;
        unsigned int mean_threshold = 50;
    };

    // Structs for different modules
    struct general
    {
        // enum for mode of operation
        enum ModeOfOperation
        {
            SEM_GEO = 0,
            SEM     = 1,
            GEO     = 2
        };
        ModeOfOperation mode_of_operation = SEM_GEO;
        std::string     env_database      = "";
    } general;

    struct markers
    {
        float impact = 0.1f;
    } markers;

    struct pointcloud
    {
        std::pair<float, float> distance_thresh = std::make_pair(0.2f, 10.0f);
    } pointcloud;

    struct optimization
    {
        bool       marginalize_planes = false;
        Constraint plane_map_point;
        Constraint plane_kf;
        Constraint plane_point;
    } optimization;

    struct refine_map_points
    {
        bool  enabled                 = false;
        float max_distance_for_delete = 0.5f;
        struct octree
        {
            float        resolution    = 0.1f;
            float        search_radius = 0.5f;
            unsigned int min_neighbors = 2;
        } octree;
    } refine_map_points;

    struct plane_based_covisibility
    {
        bool         enabled         = true;
        unsigned int max_keyframes   = 75;
        unsigned int score_per_plane = 60;
    } plane_based_covisibility;

    struct seg
    {
        unsigned int pointclouds_thresh      = 200;
        float        plane_point_dist_thresh = 0.2f;

        struct plane_association
        {
            /*!
             *@brief        Maximum angular difference between associated planes
             *              in radians.
             */
            float ominus_thresh = 0.18f;

            /*!
             * @brief       Maximum perpendicular separation between associated
             *              planes.
             */
            float distance_thresh = 0.12f;

            /*!
             * @brief       Maximum centroid distance before finite cloud
             *              compatibility is required.
             */
            float centroid_thresh = 2.5f;

            struct cluster_separation
            {
                /*!
                 * @brief       Enables finite point-cloud compatibility
                 *              checking.
                 */
                bool enabled = true;

                /*!
                 * @brief       Maximum point-cloud gap for neighbouring
                 *              fragments of one plane.
                 */
                float tolerance = 0.35f;

                Downsample downsample;
            } cluster_separation;

        } plane_association;

        struct ransac
        {
            unsigned int max_planes      = 2;
            float        distance_thresh = 0.04f;
            unsigned int max_iterations  = 600;
        } ransac;
    } seg;

    struct geo_seg
    {
        struct pointcloud
        {
            Downsample     downsample;
            OutlierRemoval outlier_removal;
        } pointcloud;
    } geo_seg;

    struct sem_seg
    {
        float min_votes          = 1.0f;
        float prob_thresh        = 0.5f;
        float conf_thresh        = 0.5f;
        float max_tilt_wall      = 0.3f;
        float max_tilt_ground    = 0.2f;
        float max_step_elevation = 0.2f;

        int   passage_kf_window                = 7;
        float max_door_width                   = 1.5f;
        float max_door_height                  = 2.0f;
        float max_wall_door_distance           = 0.5f;
        float max_kf_passage_distance          = 1.0f;
        bool  enable_passage_detection         = true;
        float passage_centroid_distance_thresh = 1.0f;

        /*!
         * @brief Configures persistent open-passage evidence extraction.
         *
         *        Candidate openings must be finite wall breaches supported on
         *        both sides by mapped wall material and confirmed in distinct
         *        Voxblox skeleton snapshots.
         */
        struct PassageDetection
        {
            /*! @brief Minimum edge-endpoint distance from the wall, in metres.
             */
            float        minimumSideDistance_m = 0.15f;
            /*! @brief Minimum absolute edge-to-wall-normal dot product. */
            float        minimumEdgeNormalAlignment = 0.50f;
            /*! @brief Minimum empty in-plane aperture radius, in metres. */
            float        minimumOpeningRadius_m = 0.30f;
            /*! @brief Passage-test expansion of finite wall bounds, in metres.
             */
            float        wallBoundsMargin_m = 0.10f;
            /*! @brief Maximum duplicate-opening separation, in metres. */
            float        duplicatePassageDistance_m = 1.00f;
            /*! @brief Minimum absolute normal alignment for duplicate tracks.
             */
            float        duplicateNormalAlignment = 0.90f;
            /*! @brief Alignment above which a nearby opening is ambiguous. */
            float        ambiguousDuplicateNormalAlignment = 0.70f;
            /*! @brief Maximum ambiguous plane offset, in metres. */
            float        ambiguousDuplicatePlaneSeparation_m = 0.75f;
            /*! @brief Maximum crossing-cluster separation, in metres. */
            float        crossingClusterDistance_m = 0.65f;
            /*! @brief Distinct skeleton snapshots required for confirmation.
             *         No longer gates passage CREATION (see
             *         minimumCrossingClusterSize) -- retained only to cap the
             *         temporal-evidence smoothing weight. */
            unsigned int minimumConfirmationSnapshots = 4U;
            /*! @brief Individual skeleton-edge crossings that must cluster
             *         into one opening in a SINGLE cycle before a passage is
             *         created for it -- the same-cycle evidence-quantity gate
             *         a wall's own admission uses (cluster point count /
             *         connectivity ratio), applied at the same semantic
             *         level and priority: enough of the free-space skeleton
             *         breaking through the wall this cycle is sufficient on
             *         its own, with no separate multi-cycle waiting period. */
            unsigned int minimumCrossingClusterSize = 2U;
            /*! @brief Missed snapshots retained before discarding evidence. */
            unsigned int maximumMissedSnapshots = 4U;
            /*! @brief Required wall extent beside each opening, in metres. */
            float        minimumHorizontalFlankExtent_m = 0.35f;
            /*! @brief Required mapped wall points on each opening flank. */
            unsigned int minimumHorizontalFlankPointCount = 12U;
        } passageDetection;

        struct pointcloud
        {
            Downsample     downsample;
            OutlierRemoval outlier_removal;
        } pointcloud;

        /*!
         * @brief Controls whether a semantic wall observation is substantial
         *        enough to create a persistent mapped plane.
         */
        struct WallCreation
        {
            /*! @brief Minimum finite points in a new wall observation. */
            unsigned int minimumPointCount = 350U;
            /*! @brief Minimum largest in-plane dimension, in metres. */
            float        minimumMajorExtent_m = 0.80f;
            /*! @brief Minimum smallest in-plane dimension, in metres. */
            float        minimumMinorExtent_m = 0.30f;
            /*! @brief Minimum finite observed wall area, in square metres. */
            float        minimumArea_m2 = 0.40f;

            /*!
             * @brief Requires most wall evidence to form one spatially
             *        connected component.
             */
            struct Connectivity
            {
                /*! @brief Enables connected-component validation. */
                bool         enabled = true;
                /*! @brief Maximum neighbour separation, in metres. */
                float        clusterTolerance_m = 0.15f;
                /*! @brief Minimum points in the largest component. */
                unsigned int minimumComponentPointCount = 300U;
                /*! @brief Minimum fraction belonging to that component.
                 *
                 * A wall's own doorway/window cuts its single-frame mask
                 * into two large, independently-valid fragments of near
                 * equal size (ratio ~0.50), not one dominant blob -- so the
                 * gate must clear ~0.50 with margin while still catching
                 * genuinely scattered/sparse noise below it. */
                float        minimumComponentRatio = 0.40f;
            } connectivity;
        } wallCreation;

        /*!
         * @brief Thresholds deciding when two WALL Planes are plausibly the
         *        two opposite-facing observations of one physical wall.
         */
        struct WallPairing
        {
            /*! @brief Thinnest plausible physical wall, in metres. */
            float minimumThickness_m = 0.05f;
            /*! @brief Thickest plausible physical wall, in metres. */
            float maximumThickness_m = 0.60f;
            /*! @brief Required in-plane footprint overlap ratio. */
            float minimumOverlapRatio = 0.30f;
        } wallPairing;

        struct reassociate
        {
            bool  enabled            = false;
            float association_thresh = 0.2f;

            /*!
             * @brief Permits adjacent finite fragments to be fused when they
             *        extend the same physical wall.
             */
            struct WallExtension
            {
                /*! @brief Enables finite-extent wall-fragment fusion. */
                bool  enabled = true;
                /*! @brief Maximum gap along either in-plane axis, in metres. */
                float maximumInPlaneGap_m = 1.25f;
                /*! @brief Minimum overlap on the orthogonal axis, in metres. */
                float minimumOrthogonalOverlap_m = 0.30f;
            } wallExtension;
        } reassociate;
    } sem_seg;

    struct room_seg
    {
        enum Method
        {
            GEOMETRIC  = 0,
            FREE_SPACE = 1,
            GNN        = 2
        };
        Method method = FREE_SPACE;

        float center_distance_thresh        = 1.5f;
        float plane_facing_dot_thresh       = -0.8f;
        float min_wall_distance_thresh      = 1.0f;
        float walls_parallelism_thresh      = 10.0f;
        float walls_perpendicularity_thresh = 10.0f;

        unsigned int min_cluster_vertices                           = 5;
        float        marker_wall_distance_thresh                    = 3.0f;
        float        cluster_point_wall_distance_thresh             = 0.5f;
        float        cluster_centroid_wall_centroid_distance_thresh = 5.0f;

        unsigned int minimumWallSupportPointCount    = 2;
        unsigned int minimumWallObservationCount     = 3;
        unsigned int minimumUndefendedWallHoldCycles = 5;
        float        minimumWallSupportRatio         = 0.5f;
        float        finiteWallBoundsMargin_m        = 0.75f;
        float        minimumFiniteWallExtent_m       = 1.0f;

        struct BoundaryTopology
        {
            bool         enabled                       = true;
            unsigned int minimumWallCount              = 3;
            float        minimumWallLength_m           = 0.75f;
            float        maximumCornerGap_m            = 0.75f;
            float        maximumInteriorIntersection_m = 0.30f;
            float        minimumEnclosedArea_m2        = 2.0f;
            float        endpointTrimRatio             = 0.02f;
            float        decisiveConflictSupportRatio  = 1.5f;
        } boundaryTopology;

        /*!
         * @brief Configures semantic room partitioning at confirmed passages.
         *
         *        Voxblox free space remains connected for ESDF planning; only
         *        the room-level interpretation cuts graph edges through a
         *        confirmed finite opening.
         */
        struct PassagePartition
        {
            /*! @brief Enables passage-aware semantic free-space partitioning.
             */
            bool  enabled = true;
            /*! @brief Edge-to-cluster vertex association limit, in metres. */
            float edgeVertexAssociationDistance_m = 0.15f;
            /*! @brief Minimum reconstructed graph-vertex coverage ratio. */
            float minimumGraphCoverageRatio = 0.65f;
            /*! @brief Expansion applied around a passage aperture, in metres.
             */
            float openingMargin_m = 0.20f;
            /*! @brief Minimum edge-endpoint passage-plane distance, in metres.
             */
            float minimumSideDistance_m = 0.10f;
            /*! @brief Enables repair of pre-confirmation wall associations. */
            bool  detachWallsBeyondPassages = true;
            /*! @brief Room/wall side-test distance threshold, in metres. */
            float wallCentroidMinimumSideDistance_m = 0.30f;
        } passagePartition;

        int gnn_version = 1;
    } room_seg;

    /*!
     * @brief Room-tracking state machine configuration (WP13 Section 18.4).
     *
     *        All values are calibration-dependent initial values.
     */
    struct room_tracking
    {
        /*! Minimum continuous dwell in the crossing guard before the
         *  CONFIRMED_ROOM <-> CROSSING_PASSAGE transitions commit (seconds).
         */
        float        crossing_dwell_s = 2.0f;
        /*! Minimum traversal confidence (0..1) for a crossing to count. */
        float        crossing_confidence = 0.7f;
        /*! Maximum time in LOST_WITH_LAST_ROOM before decay to
         *  LOST_WITHOUT_ROOM (seconds). */
        float        lost_timeout_s = 30.0f;
        /*! Maximum time in REACQUIRING_IN_NEW_MAP before decay to
         *  LOST_WITHOUT_ROOM (seconds). */
        float        reacquire_timeout_s = 60.0f;
        /*! Retry backoff between failed reacquire attempts (seconds). */
        float        reacquire_retry_interval_s = 5.0f;
        /*! Maximum failed reacquire attempts before timeout applies. */
        unsigned int reacquire_max_retries = 3U;
        /*! Minimum planes required to attempt a reacquire. */
        unsigned int reacquire_min_planes = 3U;
    } room_tracking;

    struct candidate_gen
    {
        unsigned int top_k                    = 10U;
        unsigned int candidate_pair_cap       = 1000U;
        unsigned int topology_nodes_cap       = 128U;
        unsigned int global_fallback_cap      = 1000U;
        float        weight_angle             = 1.0F;
        float        weight_extent            = 1.0F;
        float        weight_aperture          = 1.0F;
        float        weight_topology          = 1.0F;
        float        angle_missing_penalty    = 1.0F;
        float        extent_missing_penalty   = 1.0F;
        float        aperture_missing_penalty = 1.0F;
        float        ambiguity_margin         = 0.05F;
        float        angle_tolerance_rad      = 1.0e-9F;
        float        runtime_budget_ms        = 0.0F;
        unsigned int descriptor_elements_cap  = 4096U;
        unsigned int topo_refinement_iters    = 3U;
    } candidate_gen;

    /** WP13 Phase 4 (Section 9.3): plane-gated geometric verification gates.
     * Initial values are the plan's own explicit figures; all
     * calibration-dependent. */
    struct verification
    {
        float        max_normal_angle_deg        = 10.0F;
        float        max_offset_m                = 0.35F;
        float        max_support_dist_m          = 0.25F;
        float        min_inlier_ratio            = 0.6F;
        float        max_condition_number        = 100.0F;
        unsigned int ambiguity_margin_inliers    = 1U;
        unsigned int max_walls_per_room          = 16U;
        unsigned int max_hypotheses              = 2000U;
        unsigned int max_support_sample_per_wall = 64U;
        /** Section 19.5's explicit |cos(theta)| gate, distinct from
         * max_normal_angle_deg above. */
        float        min_abs_cos_normal_angle = 0.85F;
    } verification;

    /** WP13 Phase 4 (Section 17.4): EdgePlaneTransformSE3 factor noise model
     * and robust threshold. No plan-given initial values beyond the Huber
     * constant; the sigma defaults below are conservative literal choices,
     * calibration-dependent like the rest of this section. */
    struct factor
    {
        float        sigma_theta_rad      = 0.05F;
        float        sigma_offset_m       = 0.05F;
        float        huber_delta          = 1.345F;
        unsigned int optimizer_iterations = 20U;
    } factor;

    /*!
     * @brief Map-merge and axiom configuration thresholds.
     *
     *      Currently wired as YAML params; merge/axiom logic adopts them in
     *      later work packages.
     */
    struct map_merge
    {
        /*! @brief Fixed spherical tolerance for passage association across maps
         * (metres). */
        float        passage_match_tolerance_m = 0.20f;
        /*! @brief Wall coplanarity angle threshold (degrees); planes within
         * this angle are considered coplanar and merge-eligible. */
        float        wall_coplanar_angle_deg = 5.0f;
        /*! @brief Edges within this distance count as overlapping / the same
         * wall (metres). */
        float        wall_edge_overlap_m = 0.50f;
        /*! @brief Floor match tolerance for through-doorway wall axiom
         * (metres). */
        float        floor_match_tolerance_m = 0.10f;
        /*! @brief Max first-to-latest observation origins checked by the
         * through-doorway wall axiom. */
        unsigned int observation_ray_check_cap = 5U;
        /*! @brief Cooldown between consecutive-map merge attempts for the
         * same old map (seconds). */
        unsigned int merge_cooldown_s = 30U;
        /*! @brief Minimum tag-matched anchor rooms required to estimate
         * alignment. */
        unsigned int min_anchor_rooms = 2U;
        /*! @brief Minimum rooms required in each map before attempting a
         * match. */
        unsigned int min_rooms_per_map = 1U;
        /*! @brief Minimum walls required in each map before attempting a
         * match. */
        unsigned int min_walls_per_map = 3U;
        /*! @brief Maximum anchor-room centroid distance after alignment
         * (metres). Rooms pair by tag; this only bounds residual drift. */
        float        room_centroid_tolerance_m = 0.50F;
    } map_merge;

  private:
    SystemParams();
    static SystemParams *mSystemParams;
    YAML::Node           mConfig;
};
} // namespace types
} // namespace core
} // namespace vs_graphs

#endif
