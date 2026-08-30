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

#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Marker.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"
#include "Types/SystemParams.h"

#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_search.h>
#include <set>

namespace ORB_SLAM3
{
class Map;
class Marker;
class MapPoint;

class Plane
{
  public:
    enum planeVariant
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
        planeVariant semanticType = UNDEFINED;

        /*! @brief Semantic evidence retained when observations are fused. */
        std::map<planeVariant, double> semanticEvidence;
    };

    /** Immutable copy of one generation of finite plane geometry. */
    struct GeometrySnapshot
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr supportCloud;
        Eigen::Vector4d equation_World{Eigen::Vector4d::Zero()};
        Eigen::Vector3d centroid_World_m{Eigen::Vector3d::Zero()};
        double minPlaneU_m{0.0};
        double maxPlaneU_m{0.0};
        double minPlaneV_m{0.0};
        double maxPlaneV_m{0.0};
        std::size_t finiteSupportCount{0U};
        std::size_t observationCount{0U};
        std::uint64_t cloudGeneration{0U};
        std::uint64_t successfulRefitGeneration{0U};
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

        Face face{Face::UNKNOWN};
        std::size_t evidenceCount{0U};
        double consensusRatio{0.0};
        std::optional<double> medianSignedDistance_m;
    };
    /* ---------------------------------------------------------------------- *
     * VARIABLES FOR BUNDLE ADJUSTMENT
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The first keyframe that observed the plane is the  reference
     *              keyframe
     */
    KeyFrame *refKeyFrame;

    /*!
     * @brief       The reference keyframe ID for the Global BA the plane was
     *              part of
     */
    unsigned long int mnBAGlobalForKF;

    /*!
     * @brief       The plane equation in the global map after the Global BA
     */
    g2o::Plane3D mPlaneGBA;

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
    bool mbBad;

    /*!
     * @brief       Number of unique keyframes which have observed the plane.
     */
    std::size_t observationCount{0};

    /*!
     * @brief       The plane's semantic type (e.g., wall, ground, etc.)s
     */
    planeVariant planeType;

    /*!
     * @brief       The centroid of the plane
     */
    Eigen::Vector3d centroid;

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
    std::map<planeVariant, double> semanticVotes;

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
        octree;

    /*!
     * @brief Recomputes the finite plane bounds while the geometry mutexes are
     *        already held by the caller.
     *
     * @note This helper must not acquire a mutex. It exists to prevent the
     *       cloud mutation methods from recursively locking mMutexFeatures.
     */
    void updatePlaneBoundsWithoutLock(void);

    /*! @brief Rebuilds semantic votes from observations with both locks held. */
    void rebuildSemanticVotesWithoutLock(void);

  public:
    Plane(void);
    ~Plane(void);

    /*!
     * @brief       Apply a rigid/similarity transform to the plane geometry.
     *
     *              Updates the centroid, point cloud and plane equations so
     *              that the plane remains consistent with the merged map frame.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the current plane frame to the new map frame.
     */
    void applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    /**
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
    void
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
     * @return      Returns the updated plane equation in the new frame.
     */
    g2o::Plane3D transformPlaneEquation(
        const g2o::Plane3D &plane_in,
        const g2o::Sim3    &transform_oldWorldToNewWorld_in);

    /*!
     * @brief       Returns the atlas-assigned plane identifier.
     */
    int getId(void) const;

    /*!
     * @brief       Sets the atlas-assigned plane identifier.
     */
    void setId(int value);

    /*!
     * @brief       Returns the local optimizer vertex identifier.
     */
    int getOpId(void) const;

    /*!
     * @brief       Sets the local optimizer vertex identifier.
     */
    void setOpId(int value);

    /*!
     * @brief       Returns the global optimizer vertex identifier.
     */
    int getOpIdG(void) const;

    /*!
     * @brief       Sets the global optimizer vertex identifier.
     */
    void setOpIdG(int value);

    /*!
     * @brief       Reports whether this plane has been invalidated.
     */
    bool isBad(void);

    /*!
     * @brief       Marks this plane as invalid for subsequent processing.
     */
    void setBad(void);

    /*!
     * @brief       Assigns a visualization color from the semantic type.
     */
    void setColor(void);

    /*!
     * @brief       Returns the plane's RGB visualization color.
     */
    std::vector<uint8_t> getColor(void) const;

    /*!
     * @brief       Returns the accepted semantic classification.
     */
    planeVariant getPlaneType(void);

    /*!
     * @brief       Returns the leading classification from weighted votes.
     */
    planeVariant getExpectedPlaneType(void);

    /*!
     * @brief       Sets the accepted semantic classification.
     */
    void setPlaneType(planeVariant newType);

    /*!
     * @brief       Associates a non-owning map point with the plane.
     */
    void setMapPoints(MapPoint *value);

    /*!
     * @brief       Returns the map points associated with the plane.
     */
    std::set<MapPoint *> getMapPoints(void);

    /*!
     * @brief       Returns the plane centroid in the active map frame.
     */
    Eigen::Vector3d getCentroid(void) const;

    /*!
     * @brief       Method which takes the points in the map and calculates the
     *              bounds of the plane. Assumes that the plane already has
     *              points associated with it. Assumes that the plane is a
     *              rectangle.
     */
    void updateSizeOfPlane(void);

    /*!
     * @brief       Sets the plane centroid in the active map frame.
     */
    void setCentroid(const Eigen::Vector3d &value);

    /*!
     * @brief       Returns the plane equation in its observation frame.
     */
    g2o::Plane3D getLocalEquation(void) const;

    /*!
     * @brief       Sets the plane equation in its observation frame.
     */
    void setLocalEquation(const g2o::Plane3D &value);

    /*!
     * @brief       Returns the plane equation in the active map frame.
     */
    g2o::Plane3D getGlobalEquation(void) const;

    /*!
     * @brief       Sets the plane equation in the active map frame.
     */
    void setGlobalEquation(const g2o::Plane3D &value);

    /*!
     * @brief       Records or replaces an observation from a keyframe.
     */
    void addObservation(KeyFrame          *p_keyFrame_in,
                        const Observation &observation_in);

    /** Inserts or fuses a same-keyframe observation and rebuilds semantics. */
    void mergeObservation(KeyFrame          *p_keyFrame_in,
                          const Observation &observation_in);

    /*!
     * @brief       Removes the observation associated with a keyframe.
     */
    void eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       Gets the unique keyframes which observed the plane.
     *
     * @return      List of keyframes that observed the frame.
     */
    std::map<KeyFrame *, Observation> getObservations(void) const;

    /*!
     * @brief       Gets the number of unique keyframes which observed the
     *              plane.
     *
     * @return      Number of unique observations.
     */
    std::size_t getObservationCount(void) const;

    /*!
     * @brief       Returns the accumulated world-frame plane point cloud.
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getMapClouds(void);

    /** Returns a deep immutable copy of the current finite geometry. */
    GeometrySnapshot getGeometrySnapshot(void) const;

    /** Applies the association path's 75% observation-side consensus rule. */
    ObservationSideSnapshot getObservationSideSnapshot(
        const Eigen::Vector4d &normalizedEquation_World_in) const;

    /*!
     * @brief       Appends points to the accumulated plane cloud.
     *
     *              Use replaceMapClouds() to substitute the whole cloud.
     */
    void setMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in);

    /*!
     * @brief       Replaces the accumulated plane cloud contents.
     */
    void replaceMapClouds(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in);

    /** Claims and returns an immutable snapshot of a new cloud generation. */
    std::optional<GeometrySnapshot> beginMapCloudRefit(void);

    /**
     * @brief Publishes geometry from a successful whole-cloud fit.
     *
     * The centroid, equation and finite bounds are updated under one lock so
     * readers cannot observe partially refitted geometry.
     */
    bool completeMapCloudRefit(std::uint64_t sourceCloudGeneration_in,
                               const Eigen::Vector3d &centroid_World_m_in,
                               const g2o::Plane3D    &equation_World_in,
                               std::size_t finitePointCount_in);

    /*!
     * @brief       Tests whether a world-frame point belongs to the plane
     *              cloud within the configured association tolerance.
     */
    bool isPointinPlaneCloud(const Eigen::Vector3d &point);

    /*!
     * @brief       Adds weighted evidence for a semantic classification.
     */
    void castWeightedVote(planeVariant semanticType, double voteWeight);

    /*!
     * @brief       Clears semantic votes and restores undefined semantics.
     *
     * @note        Geometric support and observation constraints are preserved
     *              so that a temporarily rejected classification can recover
     *              on later observations.
     */
    void resetPlaneSemantics(void);

    /*!
     * @brief       Returns the map that owns this plane.
     */
    Map *GetMap(void);

    /*!
     * @brief       Assigns this plane to a map.
     */
    void SetMap(Map *pMap);

  protected:
    /*!
     * @brief       Non-owning pointer to the map that owns this plane.
     */
    Map *mpMap;

    /*!
     * @brief       Protects the owning-map pointer and semantic type.
     */
    std::mutex mMutexMap, mMutexType;

    /*!
     * @brief       Protects feature associations and geometric state.
     */
    mutable std::mutex mMutexFeatures, mMutexPos;
};
} // namespace ORB_SLAM3

#endif
