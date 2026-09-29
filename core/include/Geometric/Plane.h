/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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

#ifndef PLANE_H
#define PLANE_H

#include "Geometric/PlaneGeometryMetadataSnapshot.h"
#include "Geometric/PlaneStatus.h"
#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Marker.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"
#include "Types/objects/SystemParams.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_search.h>
#include <set>

namespace vs_graphs
{
namespace core
{
class Map;
class MapPoint;
namespace semantic
{
class Marker;
}
namespace geometric
{

class Plane
{
  public:
    enum class PlaneVariant : std::int8_t
    {
        /*!
         * @brief       Plane has not yet received a semantic classification.
         */
        UNDEFINED = -1,

        /*!
         * @brief       Structural vertical surface that bounds a room.
         */
        WALL = 0,

        /*!
         * @brief       Traversable horizontal surface supporting the map.
         */
        GROUND = 1,

        /*!
         * @brief       Door surface or doorway observation in a wall.
         */
        DOOR = 2,

        /*!
         * @brief       Window surface observed in a wall.
         */
        WINDOW = 3
    };

    /*!
     * @brief       Represents a single observation of a plane from a local
     *              keyframe or sensor frame.
     *
     *              The observation stores the locally expressed plane equation,
     *              the aggregated point-cloud constraint information, the
     *              confidence of the measurement, and its semantic
     *              classification.
     */
    struct Observation
    {
        /*!
         * @brief       Plane equation expressed in the local observation frame.
         */
        g2o::Plane3D localPlane;

        /*!
         * @brief       Aggregated point-cloud matrix used to evaluate the
         *              point-to-plane fitting error for this observation.
         *
         *              Each homogeneous supporting point p = [x, y, z, 1]^T
         *              contributes the outer product p * p^T. For plane
         *              coefficients pi = [a, b, c, d]^T, the accumulated
         *              squared point-to-plane residual is evaluated as:
         *
         *                  error = pi^T *
         *                          pointPlaneConstraintMatrix *
         *                          pi
         *
         *              This provides a compact representation of the supporting
         *              cloud and avoids processing every point repeatedly
         *              during optimisation.
         */
        Eigen::Matrix4d pointPlaneConstraintMatrix;

        /*!
         * @brief       Confidence associated with this plane observation.
         */
        double confidence;

        /*!
         * @brief       Semantic classification assigned to this plane
         * observation.
         */
        PlaneVariant semanticType = PlaneVariant::UNDEFINED;

        /*! @brief Semantic evidence retained when observations are fused. */
        std::map<PlaneVariant, double> semanticEvidence;
    };

    /*! Immutable copy of one generation of finite plane geometry. */
    struct GeometrySnapshot
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr supportCloud;
        Eigen::Vector4d equation_World{Eigen::Vector4d::Zero()};
        Eigen::Vector3d centroid_World_m{Eigen::Vector3d::Zero()};
        double          minPlaneU_m{0.0};
        double          maxPlaneU_m{0.0};
        double          minPlaneV_m{0.0};
        double          maxPlaneV_m{0.0};
        std::size_t     finiteSupportCount{0U};
        std::size_t     observationCount{0U};
        std::uint64_t   cloudGeneration{0U};
        std::uint64_t   successfulRefitGeneration{0U};
    };

    struct ObservationSideSnapshot
    {
        enum class Face
        {
            UNKNOWN,
            POSITIVE,
            NEGATIVE,
            AMBIGUOUS
        };

        Face                  face{Face::UNKNOWN};
        std::size_t           evidenceCount{0U};
        double                consensusRatio{0.0};
        std::optional<double> medianSignedDistance_m;
    };
    /* ---------------------------------------------------------------------- *
     * VARIABLES FOR BUNDLE ADJUSTMENT
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The first keyframe that observed the plane is the  reference
     *              keyframe
     */
    KeyFrame *p_refKeyFrame;

    /*!
     * @brief       The reference keyframe ID for the Global BA the plane was
     *              part of
     */
    unsigned long int baGlobalKeyFrameId;

    /*!
     * @brief       The plane equation in the global map after the Global BA
     */
    g2o::Plane3D planeGBA;

    /* ---------------------------------------------------------------------- *
     * VARIABLES OF PLANE GEOMETRY
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The maximum distance of the plane in the U axis tangental
     *              to plane.
     *
     * @frame       Plane tangental
     * @unit        meters
     */
    double maxPlaneU;

    /*!
     * @brief       The maximum distance of the plane in the U axis tangental
     *              to plane.
     *
     * @frame       Plane tangental
     * @unit        meters
     */
    double minPlaneU;

    /*!
     * @brief       The maximum distance of the plane in the U axis tangental
     *              to plane.
     *
     * @frame       Plane tangental
     * @unit        meters
     */
    double maxPlaneV;

    /*!
     * @brief       The maximum distance of the plane in the U axis tangental
     *              to plane.
     *
     * @frame       Plane tangental
     * @unit        meters
     */
    double minPlaneV;

  private:
    /*!
     * @brief       The plane's identifier
     */
    int id;

    /*!
     * @brief       The plane's identifier in the local optimizer
     */
    int opId;

    /*!
     * @brief       The plane's identifier in the global optimizer
     */
    int opIdG;

    /*!
     * @brief       Marks the plane as bad (if true, the plane will not be used)
     */
    bool isFlaggedBad;

    /*!
     * @brief       Number of unique keyframes which have observed the plane.
     */
    std::size_t observationCount{0};

    /*!
     * @brief       The plane's semantic type (e.g., wall, ground, etc.)s
     */
    PlaneVariant planeType;

    /*!
     * @brief       The centroid of the plane
     */
    Eigen::Vector3d centroid;

    /*!
     * @brief       World-frame camera position of the observation that first
     *              created this plane face.
     *
     * @note        A physical wall has TWO faces, and a camera can only ever
     *              observe the one turned toward it. This point is what makes
     *              those two faces distinguishable: the side of the plane this
     *              position falls on IS the face's identity, and it is stamped
     *              once, at creation, from the observing keyframe. It is stored
     *              as a POINT rather than a sign or a boolean deliberately --
     *              the global equation may be refit (and its normal re-signed)
     *              over the plane's lifetime, which would silently invert a
     *              stored sign, whereas re-deriving the sign from this point
     *              against the current equation stays correct.
     */
    std::optional<Eigen::Vector3d> observationOrigin_World_m;

    /*!
     * @brief       Non-owning link to the opposite-facing Plane hypothesis
     *              believed to be the other face of the same physical wall,
     *              when one has been identified.
     *
     * @note        Symmetric by convention: if A's twin is B, B's twin is A.
     *              Populated and re-validated by
     *              SemanticsManager::reconcileWallFacePairs(); nullptr when
     *              no plausible twin has been found (or a prior one stopped
     *              being plausible, e.g. after a refit).
     */
    Plane *p_twinFace{nullptr};

    /*!
     * @brief       A color devoted for visualization
     */
    std::vector<uint8_t> color;

    /*!
     * @brief       The plane equation in the local map
     */
    g2o::Plane3D localEquation;

    /*!
     * @brief       The plane equation in the global map
     */
    g2o::Plane3D globalEquation;

    /*!
     * @brief       The unique set of map points lying on the plane
     */
    std::set<MapPoint *> mapPoints;

    /*!
     * @brief       The votes for the semantic type of the plane
     */
    std::map<PlaneVariant, double> semanticVotes;

    /*!
     * @brief       Plane's observations in keyFrames
     */
    std::map<KeyFrame *, Observation> observations;

    /*!
     * @brief       The point cloud of the plane
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;

    /*! @brief Incremented whenever finite cloud coordinates change. */
    std::uint64_t cloudGeneration{0U};

    /*! @brief Last cloud generation claimed by a fitting attempt. */
    std::uint64_t lastRefitAttemptGeneration{0U};

    /*! @brief Cloud generation used by the latest successful fit. */
    std::uint64_t successfulRefitGeneration{0U};

    /*! @brief Finite support count used by the most recent successful fit. */
    std::size_t lastSuccessfulRefitFinitePointCount{0};

    /*!
     * @brief       The octree for the plane cloud
     */
    boost::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>>
        p_octree;

    /*!
     * @brief Recomputes the finite plane bounds while the geometry mutexes are
     *        already held by the caller.
     *
     * @note This helper must not acquire a mutex. It exists to prevent the
     *       cloud mutation methods from recursively locking featuresMutex.
     */
    [[nodiscard]] PlaneStatus updatePlaneBoundsWithoutLock(void);

    /*! @brief Rebuilds semantic votes from observations with both locks held.
     */
    [[nodiscard]] PlaneStatus rebuildSemanticVotesWithoutLock(void);

  public:
    Plane(void)
    {
        id    = -1;
        opId  = -1;
        opIdG = -1;

        isFlaggedBad = false;
        planeType    = Plane::PlaneVariant::UNDEFINED;

        centroid.setZero();

        p_map = nullptr;

        p_refKeyFrame      = nullptr;
        baGlobalKeyFrameId = 0;

        planeCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

        types::SystemParams *p_params = nullptr;
        if (types::SystemParams::getParams(p_params) !=
            types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
        {
            // getParams cannot fail; continue as before.
        }
        p_octree = boost::make_shared<
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>>(
            p_params->refineMapPoints.octree.resolution);

        minPlaneU = std::numeric_limits<double>::max();
        maxPlaneU = std::numeric_limits<double>::lowest();
        minPlaneV = std::numeric_limits<double>::max();
        maxPlaneV = std::numeric_limits<double>::lowest();
    }
    ~Plane() {}

    /*!
     * @brief       Apply a rigid/similarity transform to the plane geometry.
     *
     *              Updates the centroid, point cloud and plane equations so
     *              that the plane remains consistent with the merged map frame.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the current plane frame to the new map frame.
     */
    [[nodiscard]] PlaneStatus
        applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    /*!
     * @brief Aligns the complete finite plane geometry with an optimized
     *        world-frame equation.
     *
     * A minimal rigid correction is applied to the centroid and supporting
     * cloud before the target equation is stored. This keeps all plane
     * representations mutually consistent after graph optimization.
     *
     * @param[in] targetEquation_NewWorld_in Optimized plane equation in the
     *            active map frame.
     */
    [[nodiscard]] PlaneStatus
        alignGeometryToEquation(const g2o::Plane3D &targetEquation_NewWorld_in);

    /*!
     * @brief       Tranforms the plane equation from an old world frame to a
     *              new world frame.
     *
     * @param[in]   plane_in
     *              The plane to be transfromed, passed by reference.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              The transform from the old world frame to the new world
     *              frame, passed by reference.
     *
     * @param[out] transformedEquation_out Returns the updated plane equation in
     * the new frame.
     * @return PLANE_STATUS_SUCCESS.
     */
    [[nodiscard]] PlaneStatus
        transformPlaneEquation(const g2o::Plane3D &plane_in,
                               const g2o::Sim3 &transform_oldWorldToNewWorld_in,
                               g2o::Plane3D    &transformedEquation_out);

    /*!
     * @brief       Returns the atlas-assigned plane identifier.
     */
    [[nodiscard]] PlaneStatus getId(int &getId_out) const;

    /*!
     * @brief       Sets the atlas-assigned plane identifier.
     */
    [[nodiscard]] PlaneStatus setId(int id_in);

    /*!
     * @brief       Returns the local optimizer vertex identifier.
     */
    [[nodiscard]] PlaneStatus getOpId(int &getOpId_out) const;

    /*!
     * @brief       Sets the local optimizer vertex identifier.
     */
    [[nodiscard]] PlaneStatus setOpId(int opId_in);

    /*!
     * @brief       Returns the global optimizer vertex identifier.
     */
    [[nodiscard]] PlaneStatus getOpIdG(int &getOpIdG_out) const;

    /*!
     * @brief       Sets the global optimizer vertex identifier.
     */
    [[nodiscard]] PlaneStatus setOpIdG(int opIdG_in);

    /*!
     * @brief       Reports whether this plane has been invalidated.
     */
    [[nodiscard]] PlaneStatus isBad(bool &isBad_out);

    /*!
     * @brief       Marks this plane as invalid for subsequent processing.
     */
    [[nodiscard]] PlaneStatus setBad(void);

    /*!
     * @brief       Assigns a visualization color from the semantic type.
     */
    [[nodiscard]] PlaneStatus setColor(void);

    /*!
     * @brief       Returns the plane's RGB visualization color.
     */
    [[nodiscard]] PlaneStatus
        getColor(std::vector<uint8_t> &getColor_out) const;

    /*!
     * @brief       Returns the accepted semantic classification.
     */
    [[nodiscard]] PlaneStatus
        getPlaneType(Plane::PlaneVariant &planeType_out);

    /*!
     * @brief       Returns the leading classification from weighted votes.
     */
    [[nodiscard]] PlaneStatus
        getExpectedPlaneType(Plane::PlaneVariant &expectedPlaneType_out);

    /*!
     * @brief       Sets the accepted semantic classification.
     */
    [[nodiscard]] PlaneStatus setPlaneType(PlaneVariant planeType_in);

    /*!
     * @brief       Associates a non-owning map point with the plane.
     */
    [[nodiscard]] PlaneStatus setMapPoints(MapPoint *p_mapPoint_in);

    /*!
     * @brief       Returns the map points associated with the plane.
     */
    [[nodiscard]] PlaneStatus
        getMapPoints(std::set<core::MapPoint *> &mapPoints_out);

    /*!
     * @brief       Returns the plane centroid in the active map frame.
     */
    [[nodiscard]] PlaneStatus
        getCentroid(Eigen::Vector3d &getCentroid_out) const;

    /*!
     * @brief       Method which takes the points in the map and calculates the
     *              bounds of the plane. Assumes that the plane already has
     *              points associated with it. Assumes that the plane is a
     *              rectangle.
     */
    [[nodiscard]] PlaneStatus updateSizeOfPlane(void);

    /*!
     * @brief       Sets the plane centroid in the active map frame.
     */
    [[nodiscard]] PlaneStatus setCentroid(const Eigen::Vector3d &centroid_in);

    /*!
     * @brief       Stamps the world-frame camera position this face was first
     *              observed from. Intended to be called once, at creation.
     */
    [[nodiscard]] PlaneStatus
        setObservationOrigin_World(const Eigen::Vector3d &origin_World_m_in);

    /*!
     * @brief       Returns the world-frame camera position this face was first
     *              observed from, when one was stamped.
     */
    [[nodiscard]] PlaneStatus getObservationOrigin_World(std::optional<Eigen::Vector3d> &getObservationOrigin_World_out)
        const;

    /*!
     * @brief       Returns the linked opposite-facing Plane hypothesis for
     *              this wall's other side, or nullptr when none is set.
     */
    [[nodiscard]] PlaneStatus getTwinFace(Plane *&p_getTwinFace_out) const;

    /*!
     * @brief       Sets the linked opposite-facing Plane hypothesis. Caller
     *              is responsible for setting the reverse link symmetrically
     *              (see SemanticsManager::reconcileWallFacePairs()).
     */
    [[nodiscard]] PlaneStatus setTwinFace(Plane *p_twinFace_in);

    /*!
     * @brief       Clears the linked opposite-facing Plane hypothesis.
     */
    [[nodiscard]] PlaneStatus clearTwinFace(void);

    /*!
     * @brief       Returns the plane equation in its observation frame.
     */
    [[nodiscard]] PlaneStatus
        getLocalEquation(g2o::Plane3D &getLocalEquation_out) const;

    /*!
     * @brief       Sets the plane equation in its observation frame.
     */
    [[nodiscard]] PlaneStatus
        setLocalEquation(const g2o::Plane3D &localEquation_in);

    /*!
     * @brief       Returns the plane equation in the active map frame.
     */
    [[nodiscard]] PlaneStatus
        getGlobalEquation(g2o::Plane3D &getGlobalEquation_out) const;

    /*!
     * @brief       Sets the plane equation in the active map frame.
     */
    [[nodiscard]] PlaneStatus
        setGlobalEquation(const g2o::Plane3D &globalEquation_in);

    /*!
     * @brief       Records or replaces an observation from a keyframe.
     */
    [[nodiscard]] PlaneStatus addObservation(KeyFrame *p_keyFrame_inout,
                                             const Observation &observation_in);

    /*! Inserts or fuses a same-keyframe observation and rebuilds semantics. */
    [[nodiscard]] PlaneStatus
        mergeObservation(KeyFrame          *p_keyFrame_inout,
                         const Observation &observation_in);

    /*!
     * @brief       Removes the observation associated with a keyframe.
     */
    [[nodiscard]] PlaneStatus eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       Gets the unique keyframes which observed the plane.
     *
     * @param[out] getObservations_out List of keyframes that observed the
     * frame.
     * @return PLANE_STATUS_SUCCESS.
     */
    [[nodiscard]] PlaneStatus getObservations(std::map<core::KeyFrame *, Plane::Observation> &getObservations_out)
        const;

    /*!
     * @brief       Gets the number of unique keyframes which observed the
     *              plane.
     *
     * @param[out] getObservationCount_out Number of unique observations.
     * @return PLANE_STATUS_SUCCESS.
     */
    [[nodiscard]] PlaneStatus
        getObservationCount(std::size_t &getObservationCount_out) const;

    /*!
     * @brief       Returns an independent deep copy of the accumulated
     *              world-frame plane point cloud.
     *
     *              The copy is produced under the plane position and feature
     *              locks so callers cannot race with concurrent writers.
     */
    [[nodiscard]] PlaneStatus getMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mapClouds_out);

    /*! Returns a deep immutable copy of the current finite geometry. */
    [[nodiscard]] PlaneStatus getGeometrySnapshot(Plane::GeometrySnapshot &getGeometrySnapshot_out) const;

    /*!
     * @brief       Returns the cheap scalar plane-geometry metadata
     *              without deep-copying the support point cloud.
     *
     *              Reads exactly the fields getGeometrySnapshot() also
     *              reads (equation, centroid, bounds, evidence counts,
     *              cloud/refit generation numbers), under the identical
     *              std::scoped_lock(positionMutex, featuresMutex) critical
     *              section, but omits the cloud copy. Use this whenever a
     *              caller does not need the support cloud itself.
     *
     * @note        Thread-safe; self-locking, so callers must not already
     *              hold positionMutex or featuresMutex on this thread.
     */
    [[nodiscard]] PlaneStatus getGeometryMetadataSnapshot(PlaneGeometryMetadataSnapshot &getGeometryMetadataSnapshot_out)
        const;

    /*! Applies the association path's 75% observation-side consensus rule. */
    [[nodiscard]] PlaneStatus getObservationSideSnapshot(
        const Eigen::Vector4d          &normalizedEquation_World_in,
        Plane::ObservationSideSnapshot &observationSideSnapshot_out) const;

    /*!
     * @brief       Appends points to the accumulated plane cloud.
     *
     *              Use replaceMapClouds() to substitute the whole cloud.
     */
    [[nodiscard]] PlaneStatus setMapClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_additionalCloud_in);

    /*!
     * @brief       Replaces the accumulated plane cloud contents.
     */
    [[nodiscard]] PlaneStatus replaceMapClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_replacementCloud_in);

    /*! Claims and returns an immutable snapshot of a new cloud generation. */
    [[nodiscard]] PlaneStatus beginMapCloudRefit(std::optional<Plane::GeometrySnapshot> &geometrySnapshot_out);

    /*!
     * @brief Publishes geometry from a successful whole-cloud fit.
     *
     * The centroid, equation and finite bounds are updated under one lock so
     * readers cannot observe partially refitted geometry.
     */
    [[nodiscard]] PlaneStatus
        completeMapCloudRefit(std::uint64_t          sourceCloudGeneration_in,
                              const Eigen::Vector3d &centroid_World_m_in,
                              const g2o::Plane3D    &equation_World_in,
                              std::size_t            finitePointCount_in,
                              bool                  &wasRefitPublished_out);

    /*!
     * @brief       Tests whether a world-frame point belongs to the plane
     *              cloud within the configured association tolerance.
     */
    [[nodiscard]] PlaneStatus
        isPointinPlaneCloud(const Eigen::Vector3d &queryPoint_in,
                            bool                  &isPointinPlaneCloud_out);

    /*!
     * @brief       Adds weighted evidence for a semantic classification.
     */
    [[nodiscard]] PlaneStatus castWeightedVote(PlaneVariant semanticType_in,
                                               double       voteWeight_in);

    /*!
     * @brief       Clears semantic votes and restores undefined semantics.
     *
     * @note        Geometric support and observation constraints are preserved
     *              so that a temporarily rejected classification can recover
     *              on later observations.
     */
    [[nodiscard]] PlaneStatus resetPlaneSemantics(void);

    /*!
     * @brief       Returns the map that owns this plane.
     */
    [[nodiscard]] PlaneStatus getMap(core::Map *&p_map_out);

    /*!
     * @brief       Assigns this plane to a map.
     */
    [[nodiscard]] PlaneStatus setMap(Map *p_map_in);

  protected:
    /*!
     * @brief       Non-owning pointer to the map that owns this plane.
     */
    Map *p_map{nullptr};

    /*!
     * @brief       Protects the owning-map pointer and semantic type.
     */
    std::mutex mapMutex, typeMutex;

    /*!
     * @brief       Protects feature associations and geometric state.
     */
    mutable std::mutex featuresMutex, positionMutex;
};
} // namespace geometric
} // namespace core
} // namespace vs_graphs

#endif
