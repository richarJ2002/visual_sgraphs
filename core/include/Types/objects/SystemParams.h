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
 * @file            SystemParams.h
 *
 * @brief           Declares the SystemParams YAML-backed calibration store.
 *
 *  @note           Naming convention applied: lowerCamelCase members
 *                  (`p_` prefix for the singleton pointer),
 *                  `getParams`/`setParams` accessors, scoped enums
 *                  with fixed values and underlying types. YAML key
 *                  string literals are frozen (ros_freeze.csv) and
 *                  unchanged.
 */

#include <cstdint>
#include <string>
#include <utility>

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
    static SystemParams *getParams();
    void                 setParams(const std::string &configurationFilePath_in);

    // Common struct definitions
    struct Constraint
    {
        bool  enabled         = false;
        float informationGain = 0.1f;
    };
    struct Downsample
    {
        float        leafSize          = 0.03f;
        unsigned int minPointsPerVoxel = 5;
    };
    struct OutlierRemoval
    {
        float        stdThreshold  = 1.0;
        unsigned int meanThreshold = 50;
    };

    // Structs for different modules
    struct General
    {
        // enum for mode of operation
        enum class ModeOfOperation : std::uint8_t
        {
            SEM_GEO = 0U,
            SEM     = 1U,
            GEO     = 2U
        };
        ModeOfOperation modeOfOperation = ModeOfOperation::SEM_GEO;
        std::string     envDatabase     = "";
    } general;

    struct Markers
    {
        float impact = 0.1f;
    } markers;

    struct Pointcloud
    {
        std::pair<float, float> distanceThresh = std::make_pair(0.2f, 10.0f);
    } pointcloud;

    struct Optimization
    {
        bool       shouldMarginalizePlanes = false;
        Constraint planeMapPoint;
        Constraint planeKf;
        Constraint planePoint;
    } optimization;

    struct RefineMapPoints
    {
        bool  enabled              = false;
        float maxDistanceForDelete = 0.5f;
        struct Octree
        {
            float        resolution   = 0.1f;
            float        searchRadius = 0.5f;
            unsigned int minNeighbors = 2;
        } octree;
    } refineMapPoints;

    struct PlaneBasedCovisibility
    {
        bool         enabled       = true;
        unsigned int maxKeyframes  = 75;
        unsigned int scorePerPlane = 60;
    } planeBasedCovisibility;

    struct Seg
    {
        unsigned int pointcloudsThresh    = 200;
        float        planePointDistThresh = 0.2f;

        struct PlaneAssociation
        {
            /*!
             *@brief        Maximum angular difference between associated planes
             *              in radians.
             */
            float ominusThresh = 0.18f;

            /*!
             * @brief       Maximum perpendicular separation between associated
             *              planes.
             */
            float distanceThresh = 0.12f;

            /*!
             * @brief       Maximum centroid distance before finite cloud
             *              compatibility is required.
             */
            float centroidThresh = 2.5f;

            struct ClusterSeparation
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
            } clusterSeparation;

        } planeAssociation;

        struct Ransac
        {
            unsigned int maxPlanes      = 2;
            float        distanceThresh = 0.04f;
            unsigned int maxIterations  = 600;
        } ransac;
    } seg;

    struct GeoSeg
    {
        struct Pointcloud
        {
            Downsample     downsample;
            OutlierRemoval outlierRemoval;
        } pointcloud;
    } geoSeg;

    struct SemSeg
    {
        float minVotes         = 1.0f;
        float probThresh       = 0.5f;
        float confThresh       = 0.5f;
        float maxTiltWall      = 0.3f;
        float maxTiltGround    = 0.2f;
        float maxStepElevation = 0.2f;

        int   passageKfWindow               = 7;
        float maxDoorWidth                  = 1.5f;
        float maxDoorHeight                 = 2.0f;
        float maxWallDoorDistance           = 0.5f;
        float maxKfPassageDistance          = 1.0f;
        bool  enablePassageDetection        = true;
        float passageCentroidDistanceThresh = 1.0f;

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

        struct Pointcloud
        {
            Downsample     downsample;
            OutlierRemoval outlierRemoval;
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

        struct Reassociate
        {
            bool  enabled           = false;
            float associationThresh = 0.2f;

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
    } semSeg;

    struct RoomSeg
    {
        enum class Method : std::uint8_t
        {
            GEOMETRIC  = 0U,
            FREE_SPACE = 1U,
            GNN        = 2U
        };
        Method method = Method::FREE_SPACE;

        float centerDistanceThresh        = 1.5f;
        float planeFacingDotThresh        = -0.8f;
        float minWallDistanceThresh       = 1.0f;
        float wallsParallelismThresh      = 10.0f;
        float wallsPerpendicularityThresh = 10.0f;

        unsigned int minClusterVertices                        = 5;
        float        markerWallDistanceThresh                  = 3.0f;
        float        clusterPointWallDistanceThresh            = 0.5f;
        float        clusterCentroidWallCentroidDistanceThresh = 5.0f;

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
            bool  shouldDetachWallsBeyondPassages = true;
            /*! @brief Room/wall side-test distance threshold, in metres. */
            float wallCentroidMinimumSideDistance_m = 0.30f;
        } passagePartition;

        int gnnVersion = 1;
    } roomSeg;

    /*!
     * @brief        Room-tracking state machine configuration.
     *
     *               All values are calibration-dependent initial
     *               values.
     */
    struct RoomTracking
    {
        /*! Minimum continuous dwell in the crossing guard before the
         *  CONFIRMED_ROOM <-> CROSSING_PASSAGE transitions commit (seconds).
         */
        float        crossingDwell_s = 2.0f;
        /*! Minimum traversal confidence (0..1) for a crossing to count. */
        float        crossingConfidence = 0.7f;
        /*! Maximum time in LOST_WITH_LAST_ROOM before decay to
         *  LOST_WITHOUT_ROOM (seconds). */
        float        lostTimeout_s = 30.0f;
        /*! Maximum time in REACQUIRING_IN_NEW_MAP before decay to
         *  LOST_WITHOUT_ROOM (seconds). */
        float        reacquireTimeout_s = 60.0f;
        /*! Retry backoff between failed reacquire attempts (seconds). */
        float        reacquireRetryInterval_s = 5.0f;
        /*! Maximum failed reacquire attempts before timeout applies. */
        unsigned int reacquireMaxRetries = 3U;
        /*! Minimum planes required to attempt a reacquire. */
        unsigned int reacquireMinPlanes = 3U;
    } roomTracking;

    struct CandidateGen
    {
        unsigned int topK                   = 10U;
        unsigned int candidatePairCap       = 1000U;
        unsigned int topologyNodesCap       = 128U;
        unsigned int globalFallbackCap      = 1000U;
        float        weightAngle            = 1.0F;
        float        weightExtent           = 1.0F;
        float        weightAperture         = 1.0F;
        float        weightTopology         = 1.0F;
        float        angleMissingPenalty    = 1.0F;
        float        extentMissingPenalty   = 1.0F;
        float        apertureMissingPenalty = 1.0F;
        float        ambiguityMargin        = 0.05F;
        float        angleTolerance_rad     = 1.0e-9F;
        float        runtimeBudget_ms       = 0.0F;
        unsigned int descriptorElementsCap  = 4096U;
        unsigned int topoRefinementIters    = 3U;
    } candidateGen;

    /*! Plane-gated geometric verification gates. Initial values are
     * explicit figures; all calibration-dependent. */
    struct Verification
    {
        float        maxNormalAngle_deg      = 10.0F;
        float        maxOffset_m             = 0.35F;
        float        maxSupportDist_m        = 0.25F;
        float        minInlierRatio          = 0.6F;
        float        maxConditionNumber      = 100.0F;
        unsigned int ambiguityMarginInliers  = 1U;
        unsigned int maxWallsPerRoom         = 16U;
        unsigned int maxHypotheses           = 2000U;
        unsigned int maxSupportSamplePerWall = 64U;
        /*! Explicit |cos(theta)| gate, distinct from
         * maxNormalAngle_deg above. */
        float        minAbsCosNormalAngle = 0.85F;
    } verification;

    /*! EdgePlaneTransformSE3 factor noise model and robust threshold.
     * No given initial values beyond the Huber constant; the sigma
     * defaults below are conservative literal choices,
     * calibration-dependent like the rest of this section. */
    struct Factor
    {
        float        sigmaTheta_rad      = 0.05F;
        float        sigmaOffset_m       = 0.05F;
        float        huberDelta          = 1.345F;
        unsigned int optimizerIterations = 20U;
    } factor;

    /*!
     * @brief        Map-merge and axiom configuration thresholds.
     *
     *               Currently wired as YAML params; merge/axiom
     *               logic adopts them as it is connected.
     */
    struct MapMerge
    {
        /*! @brief Fixed spherical tolerance for passage association across maps
         * (metres). */
        float        passageMatchTolerance_m = 0.20f;
        /*! @brief Wall coplanarity angle threshold (degrees); planes within
         * this angle are considered coplanar and merge-eligible. */
        float        wallCoplanarAngle_deg = 5.0f;
        /*! @brief Edges within this distance count as overlapping / the same
         * wall (metres). */
        float        wallEdgeOverlap_m = 0.50f;
        /*! @brief Floor match tolerance for through-doorway wall axiom
         * (metres). */
        float        floorMatchTolerance_m = 0.10f;
        /*! @brief Max first-to-latest observation origins checked by the
         * through-doorway wall axiom. */
        unsigned int observationRayCheckCap = 5U;
        /*! @brief Cooldown between consecutive-map merge attempts for the
         * same old map (seconds). */
        unsigned int mergeCooldown_s = 30U;
        /*! @brief Minimum tag-matched anchor rooms required to estimate
         * alignment. */
        unsigned int minAnchorRooms = 2U;
        /*! @brief Minimum rooms required in each map before attempting a
         * match. */
        unsigned int minRoomsPerMap = 1U;
        /*! @brief Minimum walls required in each map before attempting a
         * match. */
        unsigned int minWallsPerMap = 3U;
        /*! @brief Maximum anchor-room centroid distance after alignment
         * (metres). Rooms pair by tag; this only bounds residual drift. */
        float        roomCentroidTolerance_m = 0.50F;
    } mapMerge;

  private:
    /*!
     * @brief           Creates the parameter store (private; accessed via
     *                  getParams()).
     */
    SystemParams()
    {
        p_systemParams = nullptr;
    }
    static SystemParams *p_systemParams;
    YAML::Node           config;
};
} // namespace types
} // namespace core
} // namespace vs_graphs

#endif
