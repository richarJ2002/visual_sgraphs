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
 * @note            Naming convention applied: lowerCamelCase members
 *                  (`p_` prefix for the singleton pointer),
 *                  `getParams`/`setParams` accessors, scoped enums
 *                  with fixed values and underlying types. YAML key
 *                  string literals are frozen (ros_freeze.csv) and
 *                  unchanged.
 */

#include "Types/objects/SystemParamsStatus.h"
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
/*!
 * @brief           Process-wide store of the tunable settings read from the
 *                  YAML parameter file (config/system_params.yaml). One
 *                  instance exists, obtained through getParams(); setParams()
 *                  fills it once at start-up.
 */
class SystemParams
{
  public:
    /*!
     * @brief           Returns the single parameter store, creating it with the
     *                  built-in defaults on the first call. The store is never
     *                  deleted.
     *
     * @param[out]      p_params_out
     *                  Receives a borrowed pointer to the store; never null
     *                  after the call.
     *
     * @return          SYSTEM_PARAMS_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SystemParamsStatus
        getParams(SystemParams *&p_params_out);
    /*!
     * @brief           Loads every setting from the YAML file, replacing the
     *                  defaults. A missing file, a missing or mistyped key, or
     *                  a value that fails the range checks is logged and ends
     *                  the process with exit(1) instead of returning.
     *
     * @param[in]       configurationFilePath_in
     *                  Path of the YAML parameter file.
     *
     * @return          SYSTEM_PARAMS_STATUS_SUCCESS when everything loaded;
     *                  failures never return.
     */
    [[nodiscard]] SystemParamsStatus
        setParams(const std::string &configurationFilePath_in);

    // Common struct definitions
    /*!
     * @brief           Switch and weight of one optional plane-based
     *                  optimisation factor.
     */
    struct Constraint
    {
        /*!
         * @brief           True when the optimiser adds this factor.
         */
        bool  enabled = false;
        /*!
         * @brief           Scale applied to the factor's information matrix,
         *                  together with the observation confidence. Unitless;
         *                  larger values make the optimiser trust the factor
         *                  more.
         */
        float informationGain = 0.1f;
    };
    /*!
     * @brief           Voxel-grid settings for thinning a point cloud before it
     *                  is used.
     */
    struct Downsample
    {
        /*!
         * @brief           Edge length of one voxel, metres.
         */
        float        leafSize = 0.03f;
        /*!
         * @brief           Fewest points a voxel must hold to be kept; sparser
         *                  voxels are dropped.
         */
        unsigned int minPointsPerVoxel = 5;
    };
    // Structs for different modules
    /*!
     * @brief           Run mode and environment-database settings.
     */
    struct General
    {
        /*!
         * @brief           Which segmentation pipelines run. SEM_GEO runs
         *                  semantic segmentation together with geometric
         *                  segmentation, SEM runs semantic alone, GEO runs
         *                  geometric alone. The values match the YAML integers
         *                  0, 1 and 2.
         */
        enum class ModeOfOperation : std::uint8_t
        {
            SEM_GEO = 0U,
            SEM     = 1U,
            GEO     = 2U
        };
        /*!
         * @brief           Selected pipeline, read from the YAML key
         *                  mode_of_operation as a raw integer (not
         *                  range-checked).
         */
        ModeOfOperation modeOfOperation = ModeOfOperation::SEM_GEO;
        /*!
         * @brief           Path of the JSON file holding the high-level
         *                  semantic description of the environment;
         *                  System::initialize parses it.
         */
        std::string     envDatabase = "";
    };
    /*!
     * @brief           Run mode and environment-database settings.
     */
    General general;

    /*!
     * @brief           Fiducial-marker settings.
     */
    struct Markers
    {
        /*!
         * @brief           Trust weight of marker observations, unitless.
         *                  Loaded from YAML but no optimisation step reads it
         *                  yet.
         */
        float impact = 0.1f;
    };
    /*!
     * @brief           Fiducial-marker settings.
     */
    Markers markers;

    /*!
     * @brief           Depth range accepted from the RGB-D sensor.
     */
    struct Pointcloud
    {
        /*!
         * @brief           Near and far depth limits, metres, along the sensor
         *                  z axis: first is near, second is far. The defaults
         *                  are replaced by the camera settings RGBD.NearThresh
         *                  and RGBD.FarThresh when those are read.
         */
        std::pair<float, float> distanceThresh = std::make_pair(0.2f, 10.0f);
    };
    /*!
     * @brief           Depth range accepted from the RGB-D sensor.
     */
    Pointcloud pointcloud;

    /*!
     * @brief           Switches and gains of the plane-based optimisation
     *                  factors.
     */
    struct Optimization
    {
        /*!
         * @brief           True when bundle adjustment marginalises the plane
         *                  vertices out of the problem. The plane to map point
         *                  factor works only when this is false.
         */
        bool       shouldMarginalizePlanes = false;
        /*!
         * @brief           Factor linking a plane to the map points observed on
         *                  it.
         */
        Constraint planeMapPoint;
        /*!
         * @brief           Factor linking a plane equation to the key frame
         *                  that observes it.
         */
        Constraint planeKf;
        /*!
         * @brief           Factor linking a plane to the distance of a point
         *                  lying on it.
         */
        Constraint planePoint;
    };
    /*!
     * @brief           Switches and gains of the plane-based optimisation
     *                  factors.
     */
    Optimization optimization;

    /*!
     * @brief           Settings for removing map points that lie behind a
     *                  semantic plane.
     */
    struct RefineMapPoints
    {
        /*!
         * @brief           True when pose optimisation checks map points
         *                  against semantic planes.
         */
        bool  enabled = false;
        /*!
         * @brief           Signed distance behind a plane, metres, beyond which
         *                  a map point is treated as lying behind that plane
         *                  and is checked against the plane cloud.
         */
        float maxDistanceForDelete = 0.5f;
        /*!
         * @brief           Octree search settings for testing whether a point
         *                  lies in a plane's point cloud.
         */
        struct Octree
        {
            /*!
             * @brief           Leaf edge length of the plane-cloud octree,
             *                  metres.
             */
            float        resolution = 0.1f;
            /*!
             * @brief           Radius of the neighbour search around the
             *                  queried point, metres.
             */
            float        searchRadius = 0.5f;
            /*!
             * @brief           Number of plane-cloud points that must lie
             *                  within the search radius for the point to count
             *                  as part of the plane cloud.
             */
            unsigned int minNeighbors = 2;
        };
        /*!
         * @brief           Octree search settings for testing whether a point
         *                  lies in a plane cloud.
         */
        Octree octree;
    };
    /*!
     * @brief           Settings for removing map points that lie behind a
     *                  semantic plane.
     */
    RefineMapPoints refineMapPoints;

    /*!
     * @brief           Plane-based covisibility graph settings.
     */
    struct PlaneBasedCovisibility
    {
        /*!
         * @brief           True when key frames that see the same planes are
         *                  used as local bundle adjustment neighbours.
         */
        bool         enabled = true;
        /*!
         * @brief           Number of best covisible key frames used as
         *                  neighbours of a key frame in local bundle
         *                  adjustment.
         */
        unsigned int maxKeyframes = 75;
        /*!
         * @brief           Covisibility score each plane shared by two key
         *                  frames adds to their connection weight.
         */
        unsigned int scorePerPlane = 60;
    };
    /*!
     * @brief           Plane-based covisibility graph settings.
     */
    PlaneBasedCovisibility planeBasedCovisibility;

    /*!
     * @brief           Segmentation settings shared by the geometric and
     *                  semantic pipelines.
     */
    struct Seg
    {
        /*!
         * @brief           Number of points a filtered cloud must exceed before
         *                  a plane is searched in it.
         */
        unsigned int pointcloudsThresh = 200;
        /*!
         * @brief           Largest distance from a map point to a plane,
         *                  metres, for the point to count as lying on the
         *                  plane.
         */
        float        planePointDistThresh = 0.2f;

        /*!
         * @brief           Thresholds that decide whether two plane
         *                  observations belong to the same plane.
         */
        struct PlaneAssociation
        {
            /*!
             * @brief           Maximum angular difference between associated
             *                  planes in radians.
             */
            float ominusThresh = 0.18f;

            /*!
             * @brief           Maximum perpendicular separation between
             *                  associated planes.
             */
            float distanceThresh = 0.12f;

            /*!
             * @brief           Maximum centroid distance before finite cloud
             *                  compatibility is required.
             */
            float centroidThresh = 2.5f;

            /*!
             * @brief           Splits the cloud of one plane into spatially
             *                  separate fragments.
             */
            struct ClusterSeparation
            {
                /*!
                 * @brief           Enables finite point-cloud compatibility
                 *                  checking.
                 */
                bool enabled = true;

                /*!
                 * @brief           Maximum point-cloud gap for neighbouring
                 *                  fragments of one plane.
                 */
                float tolerance = 0.35f;

                /*!
                 * @brief           Voxel filter intended for the plane cloud
                 *                  before fragments are separated. Loaded from
                 *                  YAML but not read anywhere yet.
                 */
                Downsample downsample;
            };
            /*!
             * @brief           Settings for splitting one plane's cloud into
             *                  separate fragments.
             */
            ClusterSeparation clusterSeparation;
        };
        /*!
         * @brief           Thresholds that decide whether two plane
         *                  observations are the same plane.
         */
        PlaneAssociation planeAssociation;

        /*!
         * @brief           RANSAC plane-fitting limits.
         */
        struct Ransac
        {
            /*!
             * @brief           Largest number of planes fitted in one point
             *                  cloud.
             */
            unsigned int maxPlanes = 2;
            /*!
             * @brief           Largest distance from a point to the fitted
             *                  plane, metres, for the point to be an inlier.
             */
            float        distanceThresh = 0.04f;
            /*!
             * @brief           Largest number of RANSAC iterations per plane.
             */
            unsigned int maxIterations = 600;
        };
        /*!
         * @brief           RANSAC plane-fitting limits.
         */
        Ransac ransac;
    };
    /*!
     * @brief           Segmentation settings shared by the geometric and
     *                  semantic pipelines.
     */
    Seg seg;

    /*!
     * @brief           Geometric segmentation settings.
     */
    struct GeoSeg
    {
        /*!
         * @brief           Point-cloud settings of geometric segmentation.
         */
        struct Pointcloud
        {
            /*!
             * @brief           Voxel filter intended for the geometric point
             *                  cloud. Loaded from YAML but not read anywhere
             *                  yet.
             */
            Downsample downsample;
        };
        /*!
         * @brief           Point-cloud settings of geometric segmentation.
         */
        Pointcloud pointcloud;
    };
    /*!
     * @brief           Geometric segmentation settings.
     */
    GeoSeg geoSeg;

    /*!
     * @brief           Semantic segmentation settings.
     */
    struct SemSeg
    {
        /*!
         * @brief           Weighted vote total a plane needs before it receives
         *                  a semantic label.
         */
        float minVotes = 1.0f;
        /*!
         * @brief           Smallest per-pixel class probability, 0 to 1, that
         *                  is kept in a class cloud.
         */
        float probThresh = 0.5f;
        /*!
         * @brief           Smallest segmentation confidence, 0 to 1, that is
         *                  kept; it is scaled by 255 to compare with the 8-bit
         *                  confidence values.
         */
        float confThresh = 0.5f;
        /*!
         * @brief           Largest absolute value of coefficient 1 of a wall
         *                  plane's equation, expressed in the ground-reference
         *                  frame, for the plane to stay a wall. Unitless; 0 is
         *                  perfectly vertical.
         */
        float maxTiltWall = 0.3f;
        /*!
         * @brief           Largest absolute value of coefficient 0 of a ground
         *                  plane's equation, expressed in the ground-reference
         *                  frame, for the plane to stay ground. Unitless; 0 is
         *                  perfectly horizontal.
         */
        float maxTiltGround = 0.2f;
        /*!
         * @brief           Depth, metres, below the main ground plane's median
         *                  height that other ground planes may reach before
         *                  they are dropped as not being stepped ground.
         */
        float maxStepElevation = 0.2f;

        /*!
         * @brief           Number of key frames meant to be checked when
         *                  deciding whether the camera passed through a
         *                  passage. Loaded from YAML but not read anywhere yet.
         */
        int   passageKfWindow = 7;
        /*!
         * @brief           Largest width of a door or passage, metres; wider
         *                  measured openings are limited or rejected.
         */
        float maxDoorWidth = 1.5f;
        /*!
         * @brief           Largest height of a door or passage, metres.
         */
        float maxDoorHeight = 2.0f;
        /*!
         * @brief           Largest distance, metres, between a door and its
         *                  supporting wall for the door to be attached to that
         *                  wall.
         */
        float maxWallDoorDistance = 0.5f;
        /*!
         * @brief           Largest key frame to passage distance, metres, meant
         *                  for detecting a pass through a passage. Loaded from
         *                  YAML but not read anywhere yet.
         */
        float maxKfPassageDistance = 1.0f;
        /*!
         * @brief           True when doors and passages are detected and
         *                  updated.
         */
        bool  enablePassageDetection = true;
        /*!
         * @brief           Largest distance between two passage centroids,
         *                  metres, for them to be treated as the same passage.
         */
        float passageCentroidDistanceThresh = 1.0f;

        /*!
         * @brief           Configures persistent open-passage evidence
         *                  extraction.
         *
         *                  Candidate openings must be finite wall breaches
         *                  supported on both sides by mapped wall material and
         *                  confirmed in distinct Voxblox skeleton snapshots.
         */
        struct PassageDetection
        {
            /*!
             * @brief           Minimum edge-endpoint distance from the wall, in
             *                  metres.
             */
            float        minimumSideDistance_m = 0.15f;
            /*!
             * @brief           Minimum absolute edge-to-wall-normal dot
             *                  product.
             */
            float        minimumEdgeNormalAlignment = 0.50f;
            /*!
             * @brief           Minimum empty in-plane aperture radius, in
             *                  metres.
             */
            float        minimumOpeningRadius_m = 0.30f;
            /*!
             * @brief           Passage-test expansion of finite wall bounds, in
             *                  metres.
             */
            float        wallBoundsMargin_m = 0.10f;
            /*!
             * @brief           Maximum duplicate-opening separation, in metres.
             */
            float        duplicatePassageDistance_m = 1.00f;
            /*!
             * @brief           Minimum absolute normal alignment for duplicate
             *                  tracks.
             */
            float        duplicateNormalAlignment = 0.90f;
            /*!
             * @brief           Alignment above which a nearby opening is
             *                  ambiguous.
             */
            float        ambiguousDuplicateNormalAlignment = 0.70f;
            /*!
             * @brief           Maximum ambiguous plane offset, in metres.
             */
            float        ambiguousDuplicatePlaneSeparation_m = 0.75f;
            /*!
             * @brief           Maximum crossing-cluster separation, in metres.
             */
            float        crossingClusterDistance_m = 0.65f;
            /*!
             * @brief           Distinct skeleton snapshots required for
             *                  confirmation. No longer gates passage CREATION
             *                  (see minimumCrossingClusterSize) -- retained
             *                  only to cap the temporal-evidence smoothing
             *                  weight.
             */
            unsigned int minimumConfirmationSnapshots = 4U;
            /*!
             * @brief           Individual skeleton-edge crossings that must
             *                  cluster into one opening in a SINGLE cycle
             *                  before a passage is created for it -- the
             *                  same-cycle evidence-quantity gate a wall's own
             *                  admission uses (cluster point count /
             *                  connectivity ratio), applied at the same
             *                  semantic level and priority: enough of the
             *                  free-space skeleton breaking through the wall
             *                  this cycle is sufficient on its own, with no
             *                  separate multi-cycle waiting period.
             */
            unsigned int minimumCrossingClusterSize = 2U;
            /*!
             * @brief           Missed snapshots retained before discarding
             *                  evidence.
             */
            unsigned int maximumMissedSnapshots = 4U;
            /*!
             * @brief           Required wall extent beside each opening, in
             *                  metres.
             */
            float        minimumHorizontalFlankExtent_m = 0.35f;
            /*!
             * @brief           Required mapped wall points on each opening
             *                  flank.
             */
            unsigned int minimumHorizontalFlankPointCount = 12U;
        };
        /*!
         * @brief           Open-passage evidence extraction settings.
         */
        PassageDetection passageDetection;

        /*!
         * @brief           Point-cloud settings of semantic segmentation.
         */
        struct Pointcloud
        {
            /*!
             * @brief           Voxel filter applied to each semantic class
             *                  cloud before planes are extracted.
             */
            Downsample downsample;
        };
        /*!
         * @brief           Point-cloud settings of semantic segmentation.
         */
        Pointcloud pointcloud;

        /*!
         * @brief           Controls whether a semantic wall observation is
         *                  substantial enough to create a persistent mapped
         *                  plane.
         */
        struct WallCreation
        {
            /*!
             * @brief           Minimum finite points in a new wall observation.
             */
            unsigned int minimumPointCount = 350U;
            /*!
             * @brief           Minimum largest in-plane dimension, in metres.
             */
            float        minimumMajorExtent_m = 0.80f;
            /*!
             * @brief           Minimum smallest in-plane dimension, in metres.
             */
            float        minimumMinorExtent_m = 0.30f;
            /*!
             * @brief           Minimum finite observed wall area, in square
             *                  metres.
             */
            float        minimumArea_m2 = 0.40f;

            /*!
             * @brief           Requires most wall evidence to form one
             *                  spatially connected component.
             */
            struct Connectivity
            {
                /*!
                 * @brief           Enables connected-component validation.
                 */
                bool         enabled = true;
                /*!
                 * @brief           Maximum neighbour separation, in metres.
                 */
                float        clusterTolerance_m = 0.15f;
                /*!
                 * @brief           Minimum points in the largest component.
                 */
                unsigned int minimumComponentPointCount = 300U;
                /*!
                 * @brief           Minimum fraction belonging to that
                 *                  component.
                 *
                 *                  A wall's own doorway/window cuts its
                 *                  single-frame mask into two large,
                 *                  independently-valid fragments of near equal
                 *                  size (ratio ~0.50), not one dominant blob --
                 *                  so the gate must clear ~0.50 with margin
                 *                  while still catching genuinely
                 *                  scattered/sparse noise below it.
                 */
                float        minimumComponentRatio = 0.40f;
            };
            /*!
             * @brief           Connected-component validation of a new wall
             *                  observation.
             */
            Connectivity connectivity;
        };
        /*!
         * @brief           Admission thresholds for creating a persistent wall
         *                  plane.
         */
        WallCreation wallCreation;

        /*!
         * @brief           Thresholds deciding when two WALL Planes are
         *                  plausibly the two opposite-facing observations of
         *                  one physical wall.
         */
        struct WallPairing
        {
            /*!
             * @brief           Thinnest plausible physical wall, in metres.
             */
            float minimumThickness_m = 0.05f;
            /*!
             * @brief           Thickest plausible physical wall, in metres.
             */
            float maximumThickness_m = 0.60f;
            /*!
             * @brief           Required in-plane footprint overlap ratio.
             */
            float minimumOverlapRatio = 0.30f;
        };
        /*!
         * @brief           Thresholds for pairing the two faces of one physical
         *                  wall.
         */
        WallPairing wallPairing;

        /*!
         * @brief           Re-association of semantic planes that became
         *                  duplicates after loop closure or map merge.
         */
        struct Reassociate
        {
            /*!
             * @brief           True when duplicate semantic planes are
             *                  reconciled.
             */
            bool  enabled = false;
            /*!
             * @brief           Association threshold passed to associatePlanes
             *                  when a plane is matched against the others
             *                  during re-association.
             */
            float associationThresh = 0.2f;

            /*!
             * @brief           Permits adjacent finite fragments to be fused
             *                  when they extend the same physical wall.
             */
            struct WallExtension
            {
                /*!
                 * @brief           Enables finite-extent wall-fragment fusion.
                 */
                bool  enabled = true;
                /*!
                 * @brief           Maximum gap along either in-plane axis, in
                 *                  metres.
                 */
                float maximumInPlaneGap_m = 1.25f;
                /*!
                 * @brief           Minimum overlap on the orthogonal axis, in
                 *                  metres.
                 */
                float minimumOrthogonalOverlap_m = 0.30f;
            };
            /*!
             * @brief           Settings for fusing adjacent finite fragments of
             *                  one wall.
             */
            WallExtension wallExtension;
        };
        /*!
         * @brief           Re-association settings for duplicate semantic
         *                  planes.
         */
        Reassociate reassociate;
    };
    /*!
     * @brief           Semantic segmentation settings.
     */
    SemSeg semSeg;

    /*!
     * @brief           Room segmentation settings.
     */
    struct RoomSeg
    {
        /*!
         * @brief           Algorithm that detects rooms. GEOMETRIC groups the
         *                  closest facing walls, FREE_SPACE clusters the
         *                  Voxblox free-space skeleton, and GNN uses a graph
         *                  neural network.
         */
        enum class Method : std::uint8_t
        {
            GEOMETRIC  = 0U,
            FREE_SPACE = 1U,
            GNN        = 2U
        };
        /*!
         * @brief           Room detection algorithm, read from the YAML key
         *                  method as a raw integer (not range-checked).
         */
        Method method = Method::FREE_SPACE;

        /*!
         * @brief           Largest distance between two room centres, metres,
         *                  for them to be treated as the same room.
         */
        float centerDistanceThresh = 1.5f;
        /*!
         * @brief           Smallest alignment (dot product of unit normals) two
         *                  walls need to count as facing each other. Only its
         *                  absolute value is used. Unitless.
         */
        float planeFacingDotThresh = -0.8f;
        /*!
         * @brief           Smallest gap between two facing walls, metres, for
         *                  the space between them to count as a room or
         *                  corridor.
         */
        float minWallDistanceThresh = 1.0f;
        /*!
         * @brief           Largest angle between two wall normals, degrees, for
         *                  the walls to count as parallel.
         */
        float wallsParallelismThresh = 10.0f;
        /*!
         * @brief           Largest deviation from 90 degrees, in degrees,
         *                  between two wall normals for the walls to count as
         *                  perpendicular.
         */
        float wallsPerpendicularityThresh = 10.0f;

        /*!
         * @brief           Smallest number of skeleton vertices that form a
         *                  free-space cluster.
         */
        unsigned int minClusterVertices = 5;
        /*!
         * @brief           Largest marker to wall distance, metres, for the
         *                  geometric room method. Deprecated; loaded from YAML
         *                  but not read anywhere.
         */
        float        markerWallDistanceThresh = 3.0f;
        /*!
         * @brief           Largest distance from a cluster point to a wall,
         *                  metres, for the wall to belong to the room.
         */
        float        clusterPointWallDistanceThresh = 0.5f;
        /*!
         * @brief           Largest distance from the cluster centroid to a wall
         *                  centroid, metres, for the wall to belong to the
         *                  room.
         */
        float        clusterCentroidWallCentroidDistanceThresh = 5.0f;

        /*!
         * @brief           Fewest skeleton points that must support a wall
         *                  before it is linked to a room.
         */
        unsigned int minimumWallSupportPointCount = 2;
        /*!
         * @brief           Fewest key frame observations a wall needs before it
         *                  is created as a structural element.
         */
        unsigned int minimumWallObservationCount = 3;
        /*!
         * @brief           Number of semantic cycles an unused wall is kept
         *                  before it is retired. Must be at least 1.
         */
        unsigned int minimumUndefendedWallHoldCycles = 5;
        /*!
         * @brief           Fraction of nearby skeleton points, 0 to 1, that
         *                  must project inside a wall's observed bounds.
         */
        float        minimumWallSupportRatio = 0.5f;
        /*!
         * @brief           Tolerance added around a wall's observed finite
         *                  bounds, metres.
         */
        float        finiteWallBoundsMargin_m = 0.75f;
        /*!
         * @brief           Smallest observed extent along each of a wall's two
         *                  tangent axes, metres.
         */
        float        minimumFiniteWallExtent_m = 1.0f;

        /*!
         * @brief           Finite-wall topology validation: repair of clashing
         *                  walls and classification of closed room boundaries.
         */
        struct BoundaryTopology
        {
            /*!
             * @brief           True when wall-clash repair and closed-boundary
             *                  classification run.
             */
            bool         enabled = true;
            /*!
             * @brief           Fewest finite wall segments needed to close a
             *                  room boundary; at least 3.
             */
            unsigned int minimumWallCount = 3;
            /*!
             * @brief           Smallest robust horizontal wall length used by
             *                  the boundary model, metres.
             */
            float        minimumWallLength_m = 0.75f;
            /*!
             * @brief           Largest unsupported extension from an observed
             *                  wall end to a corner, metres.
             */
            float        maximumCornerGap_m = 0.75f;
            /*!
             * @brief           Largest distance an intersection may lie inside
             *                  a wall before it is treated as a clash or
             *                  T-junction, metres.
             */
            float        maximumInteriorIntersection_m = 0.30f;
            /*!
             * @brief           Smallest horizontal area enclosed by a complete
             *                  room boundary, square metres.
             */
            float        minimumEnclosedArea_m2 = 2.0f;
            /*!
             * @brief           Fraction trimmed from each end of a wall
             *                  projection to reject outliers; 0 to 0.45.
             */
            float        endpointTrimRatio = 0.02f;
            /*!
             * @brief           Ratio by which the stronger wall's observation
             *                  score must exceed the weaker one's before the
             *                  weaker one is removed in a clash; otherwise both
             *                  are kept and the conflict is flagged. At least
             *                  1.
             */
            float        decisiveConflictSupportRatio = 1.5f;
        };
        /*!
         * @brief           Finite-wall topology validation settings.
         */
        BoundaryTopology boundaryTopology;

        /*!
         * @brief           Configures semantic room partitioning at confirmed
         *                  passages.
         *
         *                  Voxblox free space remains connected for ESDF
         *                  planning; only the room-level interpretation cuts
         *                  graph edges through a confirmed finite opening.
         */
        struct PassagePartition
        {
            /*!
             * @brief           Enables passage-aware semantic free-space
             *                  partitioning.
             */
            bool  enabled = true;
            /*!
             * @brief           Edge-to-cluster vertex association limit, in
             *                  metres.
             */
            float edgeVertexAssociationDistance_m = 0.15f;
            /*!
             * @brief           Minimum reconstructed graph-vertex coverage
             *                  ratio.
             */
            float minimumGraphCoverageRatio = 0.65f;
            /*!
             * @brief           Expansion applied around a passage aperture, in
             *                  metres.
             */
            float openingMargin_m = 0.20f;
            /*!
             * @brief           Minimum edge-endpoint passage-plane distance, in
             *                  metres.
             */
            float minimumSideDistance_m = 0.10f;
            /*!
             * @brief           Enables repair of pre-confirmation wall
             *                  associations.
             */
            bool  shouldDetachWallsBeyondPassages = true;
            /*!
             * @brief           Room/wall side-test distance threshold, in
             *                  metres.
             */
            float wallCentroidMinimumSideDistance_m = 0.30f;
        };
        /*!
         * @brief           Settings for cutting room free space at confirmed
         *                  passages.
         */
        PassagePartition passagePartition;

        /*!
         * @brief           Version of the GNN-based room segmentation: 1 is the
         *                  S-Graphs one, 2 the vS-Graphs one. Loaded from YAML
         *                  but not read anywhere yet.
         */
        int gnnVersion = 1;
    };
    /*!
     * @brief           Room segmentation settings.
     */
    RoomSeg roomSeg;

    /*!
     * @brief           Room-tracking state machine configuration.
     *
     *                  All values are calibration-dependent initial
     *                  values.
     */
    struct RoomTracking
    {
        /*!
         * @brief           Minimum continuous dwell in the crossing guard
         *                  before the CONFIRMED_ROOM <-> CROSSING_PASSAGE
         *                  transitions commit (seconds).
         */
        float        crossingDwell_s = 2.0f;
        /*!
         * @brief           Minimum traversal confidence (0..1) for a crossing
         *                  to count.
         */
        float        crossingConfidence = 0.7f;
        /*!
         * @brief           Maximum time in LOST_WITH_LAST_ROOM before decay to
         *                  LOST_WITHOUT_ROOM (seconds).
         */
        float        lostTimeout_s = 30.0f;
        /*!
         * @brief           Maximum time in REACQUIRING_IN_NEW_MAP before decay
         *                  to LOST_WITHOUT_ROOM (seconds).
         */
        float        reacquireTimeout_s = 60.0f;
        /*!
         * @brief           Retry backoff between failed reacquire attempts
         *                  (seconds).
         */
        float        reacquireRetryInterval_s = 5.0f;
        /*!
         * @brief           Maximum failed reacquire attempts before timeout
         *                  applies.
         */
        unsigned int reacquireMaxRetries = 3U;
        /*!
         * @brief           Minimum planes required to attempt a reacquire.
         */
        unsigned int reacquireMinPlanes = 3U;
    };
    /*!
     * @brief           Room-tracking state machine settings.
     */
    RoomTracking roomTracking;

    /*!
     * @brief           Limits and weights for generating room-match candidates
     *                  across maps.
     */
    struct CandidateGen
    {
        /*!
         * @brief           Largest number of candidates returned, best first.
         */
        unsigned int topK = 10U;
        /*!
         * @brief           Largest number of room pairs scored in the
         *                  adjacency-prioritised pass.
         */
        unsigned int candidatePairCap = 1000U;
        /*!
         * @brief           Largest number of topology graph nodes used per
         *                  room.
         */
        unsigned int topologyNodesCap = 128U;
        /*!
         * @brief           Largest number of room pairs scored in the global
         *                  fallback pass.
         */
        unsigned int globalFallbackCap = 1000U;
        /*!
         * @brief           Weight of the wall-angle cue in the combined
         *                  distance, unitless.
         */
        float        weightAngle = 1.0F;
        /*!
         * @brief           Weight of the wall-extent cue in the combined
         *                  distance, unitless.
         */
        float        weightExtent = 1.0F;
        /*!
         * @brief           Weight of the passage-aperture cue in the combined
         *                  distance, unitless.
         */
        float        weightAperture = 1.0F;
        /*!
         * @brief           Weight of the topology cue in the combined distance,
         *                  unitless.
         */
        float        weightTopology = 1.0F;
        /*!
         * @brief           Value used in place of a missing entry when two
         *                  rooms' wall-angle lists differ in length.
         */
        float        angleMissingPenalty = 1.0F;
        /*!
         * @brief           Value used in place of a missing entry when two
         *                  rooms' wall-extent lists differ in length.
         */
        float        extentMissingPenalty = 1.0F;
        /*!
         * @brief           Penalty distance for a passage aperture that has no
         *                  partner in the other room.
         */
        float        apertureMissingPenalty = 1.0F;
        /*!
         * @brief           A candidate whose distance is within this margin of
         *                  the best one is flagged as ambiguous.
         */
        float        ambiguityMargin = 0.05F;
        /*!
         * @brief           Angle differences at or below this value, radians,
         *                  count as zero.
         */
        float        angleTolerance_rad = 1.0e-9F;
        /*!
         * @brief           Runtime budget, milliseconds, kept as profiling
         *                  metadata only; the generator never reads a clock. 0
         *                  means no budget.
         */
        float        runtimeBudget_ms = 0.0F;
        /*!
         * @brief           Largest number of elements in one room descriptor.
         */
        unsigned int descriptorElementsCap = 4096U;
        /*!
         * @brief           Number of refinement rounds when building a room's
         *                  topology signature.
         */
        unsigned int topoRefinementIters = 3U;
    };
    /*!
     * @brief           Room-match candidate generation settings.
     */
    CandidateGen candidateGen;

    /*!
     * @brief           Plane-gated geometric verification gates. Initial values
     *                  are explicit figures; all calibration-dependent.
     */
    struct Verification
    {
        /*!
         * @brief           Largest angle between two matched wall normals,
         *                  degrees, for the walls to agree.
         */
        float        maxNormalAngle_deg = 10.0F;
        /*!
         * @brief           Largest difference between matched wall plane
         *                  offsets after alignment, metres.
         */
        float        maxOffset_m = 0.35F;
        /*!
         * @brief           Largest symmetric support distance between two
         *                  matched walls after alignment, metres.
         */
        float        maxSupportDist_m = 0.25F;
        /*!
         * @brief           Smallest inlier ratio, 0 to 1, for a hypothesis to
         *                  be accepted.
         */
        float        minInlierRatio = 0.6F;
        /*!
         * @brief           Largest condition number of the translation fit;
         *                  above it a hypothesis is discarded. Unitless.
         */
        float        maxConditionNumber = 100.0F;
        /*!
         * @brief           Number of inliers by which the best hypothesis must
         *                  beat the runner-up; otherwise the match is rejected
         *                  as ambiguous.
         */
        unsigned int ambiguityMarginInliers = 1U;
        /*!
         * @brief           Largest number of walls collected per room.
         */
        unsigned int maxWallsPerRoom = 16U;
        /*!
         * @brief           Largest number of wall-triple hypotheses evaluated.
         */
        unsigned int maxHypotheses = 2000U;
        /*!
         * @brief           Largest number of support points sampled per wall.
         */
        unsigned int maxSupportSamplePerWall = 64U;
        /*!
         * @brief           Explicit |cos(theta)| gate, distinct from
         *                  maxNormalAngle_deg above.
         */
        float        minAbsCosNormalAngle = 0.85F;
    };
    /*!
     * @brief           Geometric verification gates for a room match.
     */
    Verification verification;

    /*!
     * @brief           EdgePlaneTransformSE3 factor noise model and robust
     *                  threshold. No given initial values beyond the Huber
     *                  constant; the sigma defaults below are conservative
     *                  literal choices, calibration-dependent like the rest of
     *                  this section.
     */
    struct Factor
    {
        /*!
         * @brief           Standard deviation of the wall normal angle in the
         *                  edge-plane factor, radians.
         */
        float        sigmaTheta_rad = 0.05F;
        /*!
         * @brief           Standard deviation of the wall plane offset in the
         *                  edge-plane factor, metres.
         */
        float        sigmaOffset_m = 0.05F;
        /*!
         * @brief           Huber kernel threshold of the edge-plane factor, in
         *                  standard deviations; residuals beyond it are
         *                  weighted down.
         */
        float        huberDelta = 1.345F;
        /*!
         * @brief           Number of optimiser iterations used by the
         *                  verification step.
         */
        unsigned int optimizerIterations = 20U;
    };
    /*!
     * @brief           Noise model and robust threshold of the edge-plane
     *                  factor.
     */
    Factor factor;

    /*!
     * @brief           Map-merge and axiom configuration thresholds.
     *
     *                  Currently wired as YAML params; merge/axiom
     *                  logic adopts them as it is connected.
     */
    struct MapMerge
    {
        /*!
         * @brief           Fixed spherical tolerance for passage association
         *                  across maps (metres).
         */
        float        passageMatchTolerance_m = 0.20f;
        /*!
         * @brief           Wall coplanarity angle threshold (degrees); planes
         *                  within this angle are considered coplanar and
         *                  merge-eligible.
         */
        float        wallCoplanarAngle_deg = 5.0f;
        /*!
         * @brief           Edges within this distance count as overlapping /
         *                  the same wall (metres).
         */
        float        wallEdgeOverlap_m = 0.50f;
        /*!
         * @brief           Floor match tolerance for through-doorway wall axiom
         *                  (metres).
         */
        float        floorMatchTolerance_m = 0.10f;
        /*!
         * @brief           Max first-to-latest observation origins checked by
         *                  the through-doorway wall axiom.
         */
        unsigned int observationRayCheckCap = 5U;
        /*!
         * @brief           Cooldown between consecutive-map merge attempts for
         *                  the same old map (seconds).
         */
        unsigned int mergeCooldown_s = 30U;
        /*!
         * @brief           Minimum tag-matched anchor rooms required to
         *                  estimate alignment.
         */
        unsigned int minAnchorRooms = 2U;
        /*!
         * @brief           Minimum rooms required in each map before attempting
         *                  a match.
         */
        unsigned int minRoomsPerMap = 1U;
        /*!
         * @brief           Minimum walls required in each map before attempting
         *                  a match.
         */
        unsigned int minWallsPerMap = 3U;
        /*!
         * @brief           Maximum anchor-room centroid distance after
         *                  alignment (metres). Rooms pair by tag; this only
         *                  bounds residual drift.
         */
        float        roomCentroidTolerance_m = 0.50F;
    };
    /*!
     * @brief           Map-merge and axiom thresholds.
     */
    MapMerge mapMerge;

  private:
    /*!
     * @brief           Creates the parameter store (private; accessed via
     *                  getParams()).
     */
    SystemParams()
    {
        p_systemParams = nullptr;
    }
    /*!
     * @brief           The single parameter store, created by getParams() on
     *                  first use and never deleted; null until then.
     */
    static SystemParams *p_systemParams;
    /*!
     * @brief           Root of the parsed YAML parameter file, kept by
     *                  setParams().
     */
    YAML::Node           config;
};
} // namespace types
} // namespace core
} // namespace vs_graphs

#endif
