/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VS_GRAPHS_CORE_OPTIMIZABLETYPES_H
#define VS_GRAPHS_CORE_OPTIMIZABLETYPES_H

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/types/isometry3d_mappings.h"

#include <Eigen/Geometry>
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/vertex_plane.h>

namespace vs_graphs
{
namespace core
{
/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (SE3) [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read(std::istream &inputStream_inout);

    bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error = observation - p_camera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d                                  Xw;
    camera_models::geometriccamera::GeometricCamera *p_camera;
};

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (SE3) using depth [Note]: it creates constraint for one measurement, i.e.,
 * depth (z) For RGB-D: adds depth residual to pose optimization
 */
class EdgeSE3ProjectXYZDepth
    : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZDepth() {}

    bool read(std::istream &inputStream_inout);

    bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // Transform 3D point to camera frame
        Eigen::Vector3d Xc = v1->estimate().map(Xw);
        // Depth measurement
        double          observation = _measurement;
        // Error is difference between measured depth and estimated depth
        _error[0] = observation - Xc(2);
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d                                  Xw;
    camera_models::geometriccamera::GeometricCamera *p_camera;
};

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (SE3) [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZOnlyPoseToBody
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody() {}

    bool read(std::istream &inputStream_inout);

    bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error =
            observation - p_camera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d                                  Xw;
    camera_models::geometriccamera::GeometricCamera *p_camera;

    g2o::SE3Quat mTrl;
};

/*!
 * The edge used to connect a MapPoint vertex (XYZ) to a KeyFrame vertex (SE3)
 * [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge<2,
                                                     Eigen::Vector2d,
                                                     g2o::VertexSBAPointXYZ,
                                                     g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ() :
        BaseBinaryEdge<2,
                       Eigen::Vector2d,
                       g2o::VertexSBAPointXYZ,
                       g2o::VertexSE3Expmap>()
    {}

    bool read(std::istream &inputStream_inout);

    bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error =
            observation - p_camera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
    }

    virtual void linearizeOplus();

    camera_models::geometriccamera::GeometricCamera *p_camera;
};

/*!
 * The edge used to connect a MapPoint vertex (XYZ) to a KeyFrame vertex (SE3)
 * [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZToBody
    : public g2o::BaseBinaryEdge<2,
                                 Eigen::Vector2d,
                                 g2o::VertexSBAPointXYZ,
                                 g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody() :
        BaseBinaryEdge<2,
                       Eigen::Vector2d,
                       g2o::VertexSBAPointXYZ,
                       g2o::VertexSE3Expmap>()
    {}

    bool read(std::istream &inputStream_inout);

    bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error = observation -
                 p_camera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
    }

    virtual void linearizeOplus();

    camera_models::geometriccamera::GeometricCamera *p_camera;
    g2o::SE3Quat                                     mTrl;
};

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (Sim3) [Note]: it creates constraint for seven measurements, i.e., (u, v, z,
 * roll, pitch, yaw, scale)
 */
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap() :
        BaseVertex<7, g2o::Sim3>()
    {
        _marginalized = false;
        isScaleFixed  = false;
    }
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    virtual void setToOriginImpl()
    {
        _estimate = g2o::Sim3();
    }

    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double *>(p_update_in));

        if (isScaleFixed)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    camera_models::geometriccamera::GeometricCamera *p_firstCamera,
        *p_secondCamera;

    bool isScaleFixed;
};

class EdgeSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2,
                                 Eigen::Vector2d,
                                 g2o::VertexSBAPointXYZ,
                                 vs_graphs::core::VertexSim3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ() :
        g2o::BaseBinaryEdge<2,
                            Eigen::Vector2d,
                            g2o::VertexSBAPointXYZ,
                            VertexSim3Expmap>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const vs_graphs::core::VertexSim3Expmap *v1 =
            static_cast<const vs_graphs::core::VertexSim3Expmap *>(
                _vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d observation(_measurement);
        _error = observation -
                 v1->p_firstCamera->project(v1->estimate().map(v2->estimate()));
    }
};

class EdgeInverseSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2,
                                 Eigen::Vector2d,
                                 g2o::VertexSBAPointXYZ,
                                 VertexSim3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ() :
        g2o::BaseBinaryEdge<2,
                            Eigen::Vector2d,
                            g2o::VertexSBAPointXYZ,
                            VertexSim3Expmap>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        const vs_graphs::core::VertexSim3Expmap *v1 =
            static_cast<const vs_graphs::core::VertexSim3Expmap *>(
                _vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d observation(_measurement);
        _error =
            observation - v1->p_secondCamera->project(
                              (v1->estimate().inverse().map(v2->estimate())));
    }
};

/*!
 * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
 * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll,
 * pitch, yaw) 🚧 [vS-Graphs v1.5] Deprecated, but extended for other
 * optimization constraints.
 */
class EdgeSE3ProjectSE3 : public g2o::BaseBinaryEdge<6,
                                                     g2o::Isometry3D,
                                                     g2o::VertexSE3Expmap,
                                                     g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectSE3() :
        g2o::BaseBinaryEdge<6,
                            g2o::Isometry3D,
                            g2o::VertexSE3Expmap,
                            g2o::VertexSE3Expmap>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;
    virtual void setMeasurement(const g2o::Isometry3D &m_in) override
    {
        _measurement = m_in;
    }

    void computeError()
    {
        // Marker's global pose
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // KeyFrame's global pose
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        // Calculate the local pose of the marker w.r.t. the keyframe
        g2o::SE3Quat markerLp =
            p_keyFrameGpVertex->estimate() * p_markerGpVertex->estimate();

        g2o::Isometry3D markerLpIso = g2o::Isometry3D::Identity();
        markerLpIso.matrix()        = markerLp.to_homogeneous_matrix();
        // Calculating the transformation between the measuremenent and the
        // marker's local pose
        g2o::Isometry3D delta = _measurement.inverse() * markerLpIso;

        // Calculating the final error
        _error = g2o::internal::toVectorMQT(delta);
    }
};

/*!
 * The edge used to connect a Room vertex (SE3) to a Passage vertex (SE3)
 * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll,
 * pitch, yaw)
 */
class EdgeSE3DoorwayProjectSE3Room : public EdgeSE3ProjectSE3
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3DoorwayProjectSE3Room() :
        EdgeSE3ProjectSE3()
    {}

    void computeError()
    {
        // Room's global pose
        const g2o::VertexSE3Expmap *p_roomGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // Passage's global pose
        const g2o::VertexSE3Expmap *p_doorwayGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        // Calculate the local pose of the doorway w.r.t. the keyframe
        g2o::SE3Quat doorwayLp = p_roomGpVertex->estimate().inverse() *
                                 p_doorwayGpVertex->estimate();

        g2o::Isometry3D doorwayLpIso = g2o::Isometry3D::Identity();
        doorwayLpIso.matrix()        = doorwayLp.to_homogeneous_matrix();
        // Calculating the transformation between the measuremenent and the
        // doorway's local pose
        g2o::Isometry3D delta = _measurement.inverse() * doorwayLpIso;

        // Calculating the final error
        _error = g2o::internal::toVectorMQT(delta);
    }
};

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame point
 * (SE3) [Note]: it creates constraint connecting the points in a plane
 * observation to the plane
 */
class EdgeSE3KFPointToPlane : public g2o::BaseBinaryEdge<1,
                                                         Eigen::Matrix4d,
                                                         g2o::VertexSE3Expmap,
                                                         g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3KFPointToPlane() :
        g2o::BaseBinaryEdge<1,
                            Eigen::Matrix4d,
                            g2o::VertexSE3Expmap,
                            g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void setMeasurement(const Eigen::Matrix4d &m_in) override
    {
        _measurement = m_in;
    }

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *v2 =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        Eigen::Matrix4d Ti  = v1->estimate().inverse().to_homogeneous_matrix();
        Eigen::Vector4d Pj  = v2->estimate().coeffs();
        Eigen::Matrix4d Gij = _measurement;
        _error              = Pj.transpose() * Ti * Gij * Ti.transpose() * Pj;
    }

    // Checks if the plane distance d is in the correct direction
    bool isDistanceCorrect()
    {
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        // local plane equation
        Eigen::Isometry3d keyFramePose = p_keyFrameGpVertex->estimate();
        g2o::Plane3D localPlane = keyFramePose * p_planeGpVertex->estimate();

        return (localPlane.coeffs()(3) > 0);
    }
};

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame vertex
 * (SE3) [Note]: it creates constraint for three measurements, i.e., (x, y, z)
 */
class EdgeVertexPlaneProjectSE3KF
    : public g2o::BaseBinaryEdge<3,
                                 g2o::Plane3D,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexPlaneProjectSE3KF() :
        g2o::BaseBinaryEdge<3,
                            g2o::Plane3D,
                            g2o::VertexSE3Expmap,
                            g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void setMeasurement(const g2o::Plane3D &m_in) override
    {
        _measurement = m_in;
    }

    void computeError()
    {
        // KeyFrame's global pose
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // Plane's global pose
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        // Calculating poses (in global frame)
        Eigen::Isometry3d keyFramePose = p_keyFrameGpVertex->estimate();
        g2o::Plane3D localPlane = keyFramePose * p_planeGpVertex->estimate();

        // Calculating the error
        _error = localPlane.ominus(_measurement);
    }

    // Checks if the plane distance d is in the correct direction
    bool isDistanceCorrect()
    {
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        // local plane equation
        Eigen::Isometry3d keyFramePose = p_keyFrameGpVertex->estimate();
        g2o::Plane3D localPlane = keyFramePose * p_planeGpVertex->estimate();

        return (localPlane.coeffs()(3) > 0);
    }
};

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Plane vertex
 * (VertexPlane)
 */
class EdgeVertexPlaneProjectPointXYZ
    : public g2o::
          BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexPlaneProjectPointXYZ() :
        g2o::BaseBinaryEdge<1,
                            double,
                            g2o::VertexSBAPointXYZ,
                            g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        // Map Point's position
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        // Plane's global pose
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        // Calculating the error
        // plane equation is already normalized -> D = n.x + d
        _error[0] = p_planeGpVertex->estimate().coeffs().head(3).dot(
                        p_pointVertex->estimate().head(3)) +
                    p_planeGpVertex->estimate().coeffs()(3);
    }
};

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a Marker vertex
 * (SE3) [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
 * 🚧 [vS-Graphs v1.5] Unused.
 */
class EdgeVertexPlaneProjectSE3M
    : public g2o::BaseBinaryEdge<4,
                                 Eigen::Vector4d,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexPlaneProjectSE3M() :
        g2o::BaseBinaryEdge<4,
                            Eigen::Vector4d,
                            g2o::VertexSE3Expmap,
                            g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        // Marker's global pose
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // Plane's global pose
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        // Calculating poses (in global frame)
        g2o::Isometry3D markerPose  = p_markerGpVertex->estimate();
        g2o::Vector4D   planeCoeffs = p_planeGpVertex->estimate().coeffs();

        // Normalize the plane vector if necessary
        if (planeCoeffs(3) < 0)
            planeCoeffs *= -1;

        // Create the plane plane in global frame
        g2o::Plane3D plane_g(planeCoeffs);

        // Calculate the plane in marker's frame
        g2o::Plane3D plane_m = markerPose.inverse() * plane_g;

        // Calculate the normal of the marker in marker's frame
        g2o::Plane3D markerNormal_m =
            markerPose.inverse() * markerPose.matrix().col(2);

        // Calculate the difference of the planes
        Eigen::Vector3d planeDiff =
            plane_m.coeffs().head(3) - markerNormal_m.coeffs().head(3);

        _error[0] = planeDiff(0);
        _error[1] = planeDiff(1);
        _error[2] = planeDiff(2);
        _error[3] = plane_m.coeffs()(3); // Distance (d) of the cacmera
    }
};

/*!
 * The edge used to enforce parallelism between two Plane vertices (VertexPlane)
 * [Note]: it creates constraint for one measurement, i.e., (angle difference)
 */
class EdgeVertexPlaneParallelism
    : public g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexPlaneParallelism() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError() override
    {
        // Planes
        const g2o::VertexPlane *p_plane1Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[0]);
        const g2o::VertexPlane *p_plane2Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        Eigen::Vector3d n1 = p_plane1Vertex->estimate().normal().normalized();
        Eigen::Vector3d n2 = p_plane2Vertex->estimate().normal().normalized();

        // Compute deviation from parallelism
        double error = std::fabs(n1.dot(n2));
        _error[0]    = 1 - error;
    }
};

/*!
 * The edge used to enforce perpendicularity between two Plane vertices
 * (VertexPlane) [Note]: it creates constraint for one measurement, i.e., (angle
 * difference)
 */
class EdgeVertexPlanePerpendicularity
    : public g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexPlanePerpendicularity() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError() override
    {
        // Planes
        const g2o::VertexPlane *p_plane1Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[0]);
        const g2o::VertexPlane *p_plane2Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        Eigen::Vector3d n1 = p_plane1Vertex->estimate().normal().normalized();
        Eigen::Vector3d n2 = p_plane2Vertex->estimate().normal().normalized();

        // Compute deviation from perpendicularity
        double error = std::fabs(n1.dot(n2));
        _error[0]    = error;
    }
};

/*!
 * The edge used to connect a Two-wall Room's center (SE3) to Wall vertices
 * (VertexPlane) [Note]: it creates constraint for three measurements, i.e., (x,
 * y, z) 🚧 [vS-Graphs v1.5] Deprecated with the introduction of n-wall rooms.
 */
class EdgeVertex2PlaneProjectSE3Room
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertex2PlaneProjectSE3Room() :
        g2o::BaseMultiEdge<3, Eigen::Vector3d>()
    {
        resize(3);
    }
    EdgeVertex2PlaneProjectSE3Room(Eigen::Vector3d position_in) :
        g2o::BaseMultiEdge<3, Eigen::Vector3d>()
    {
        // markerPosition = position;
        resize(3);
    }

    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *v2 =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);
        const g2o::VertexPlane *v3 =
            static_cast<const g2o::VertexPlane *>(_vertices[2]);

        Eigen::Vector3d roomPose = v1->estimate().translation();
        Eigen::Vector4d wall1    = v2->estimate().coeffs();
        Eigen::Vector4d wall2    = v3->estimate().coeffs();

        correctPlaneDirection(wall1);
        correctPlaneDirection(wall2);

        Eigen::Vector3d vector;
        if (fabs(wall1(3)) > fabs(wall2(3)))
        {
            vector = (0.5 * (fabs(wall1(3)) * wall1.head(3) -
                             fabs(wall2(3)) * wall2.head(3))) +
                     fabs(wall2(3)) * wall2.head(3);
        }
        else
        {
            vector = (0.5 * (fabs(wall2(3)) * wall2.head(3) -
                             fabs(wall1(3)) * wall1.head(3))) +
                     fabs(wall1(3)) * wall1.head(3);
        }

        Eigen::Vector3d normal = vector / vector.norm();
        // Eigen::Vector3d finalPose = vec + (markerPosition -
        // (markerPosition.dot(normal)) * normal);

        _error = roomPose - vector;
    }

  protected:
    virtual void correctPlaneDirection(Eigen::Vector4d &plane_inout)
    {
        if (plane_inout(3) > 0)
            plane_inout *= -1;
    }
};

/*!
 * The edge used to connect a Four-wall Room's centroid (SE3) to Wall vertices
 * (VertexPlane) [Note]: it creates constraint for three measurements, i.e., (x,
 * y, z) 🚧 [vS-Graphs v1.5] Deprecated with the introduction of n-wall rooms.
 */
class EdgeVertex4PlaneProjectSE3Room
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertex4PlaneProjectSE3Room() :
        g2o::BaseMultiEdge<3, Eigen::Vector3d>()
    {
        resize(5);
    }
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *v2 =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);
        const g2o::VertexPlane *v3 =
            static_cast<const g2o::VertexPlane *>(_vertices[2]);
        const g2o::VertexPlane *v4 =
            static_cast<const g2o::VertexPlane *>(_vertices[3]);
        const g2o::VertexPlane *v5 =
            static_cast<const g2o::VertexPlane *>(_vertices[4]);

        Eigen::Vector3d roomPose = v1->estimate().translation();
        Eigen::Vector4d xPlane1  = v2->estimate().coeffs();
        Eigen::Vector4d xPlane2  = v3->estimate().coeffs();
        Eigen::Vector4d yPlane1  = v4->estimate().coeffs();
        Eigen::Vector4d yPlane2  = v5->estimate().coeffs();

        correctPlaneDirection(xPlane1);
        correctPlaneDirection(xPlane2);
        correctPlaneDirection(yPlane1);
        correctPlaneDirection(yPlane2);

        Eigen::Vector3d vectorX, vectorY;
        if (fabs(xPlane1(3)) > fabs(xPlane2(3)))
            vectorX = (0.5 * (fabs(xPlane1(3)) * xPlane1.head(3) -
                              fabs(xPlane2(3)) * xPlane2.head(3))) +
                      fabs(xPlane2(3)) * xPlane2.head(3);
        else
            vectorX = (0.5 * (fabs(xPlane2(3)) * xPlane2.head(3) -
                              fabs(xPlane1(3)) * xPlane1.head(3))) +
                      fabs(xPlane1(3)) * xPlane1.head(3);

        if (fabs(yPlane1(3)) > fabs(yPlane2(3)))
            vectorY = (0.5 * (fabs(yPlane1(3)) * yPlane1.head(3) -
                              fabs(yPlane2(3)) * yPlane2.head(3))) +
                      fabs(yPlane2(3)) * yPlane2.head(3);
        else
            vectorY = (0.5 * (fabs(yPlane2(3)) * yPlane2.head(3) -
                              fabs(yPlane1(3)) * yPlane1.head(3))) +
                      fabs(yPlane1(3)) * yPlane1.head(3);

        Eigen::Vector3d finalPose = vectorX + vectorY;
        _error                    = roomPose - finalPose;
    }

  protected:
    virtual void correctPlaneDirection(Eigen::Vector4d &plane_inout)
    {
        if (plane_inout(3) > 0)
            plane_inout *= -1;
    }
};

/*!
 * The edge used to connect an N-wall Room's centroid (SE3) to its Wall vertices
 * (VertexPlane) [Note]: it creates constraint for three measurements, i.e., (x,
 * y, z)
 */
class EdgeVertexNPlaneProjectSE3Room
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;
    EdgeVertexNPlaneProjectSE3Room()
    {
        // Dynamically sized edge: at least one SE3 (room center) + N planes
        resize(1);
    }

    void computeError() override
    {
        // First vertex is always the room pose (SE3)
        const g2o::VertexSE3Expmap *p_roomVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d roomPose = p_roomVertex->estimate().translation();

        // Remaining vertices are walls
        std::vector<Eigen::Vector4d> walls;
        for (size_t vertexIndex = 1; vertexIndex < _vertices.size();
             ++vertexIndex)
        {
            const g2o::VertexPlane *p_wallVertex =
                static_cast<const g2o::VertexPlane *>(_vertices[vertexIndex]);
            Eigen::Vector4d plane = p_wallVertex->estimate().coeffs();
            correctPlaneDirection(plane);
            walls.push_back(plane);
        }

        // Compute representative position from all walls enclosing the cluster
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto &wall : walls)
        {
            // Each wall equation ax+by+cz+d=0 → normal = (a,b,c), offset = d
            Eigen::Vector3d normal = wall.head<3>();
            double          d      = wall(3);

            // Approximate contribution: project along the normal
            Eigen::Vector3d contrib = -d * normal;
            centroid += contrib;
        }

        if (!walls.empty())
            centroid /= static_cast<double>(walls.size());

        // Final error
        _error = roomPose - centroid;
    }

  protected:
    void correctPlaneDirection(Eigen::Vector4d &plane_inout)
    {
        if (plane_inout(3) > 0)
            plane_inout *= -1;
    }
};

/*!
 * The edge used to connect a Floor centroid (SE3) to its Room vertices (SE3)
 * [Note]: it creates constraint for three measurements, i.e., (x, y, z)
 */
class EdgeVertexNSE3RoomProjectSE3Floor
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;
    EdgeVertexNSE3RoomProjectSE3Floor()
    {
        // Dynamically sized edge: at least one SE3 (floor center) + N rooms
        resize(1);
    }

    void computeError() override
    {
        // First vertex is always the floor pose (SE3)
        const g2o::VertexSE3Expmap *p_floorVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d floorPose = p_floorVertex->estimate().translation();

        // Remaining vertices are rooms
        std::vector<Eigen::Vector3d> rooms;
        for (size_t vertexIndex = 1; vertexIndex < _vertices.size();
             ++vertexIndex)
        {
            const g2o::VertexSE3Expmap *p_roomVertex =
                static_cast<const g2o::VertexSE3Expmap *>(
                    _vertices[vertexIndex]);
            Eigen::Vector3d room = p_roomVertex->estimate().translation();
            rooms.push_back(room);
        }

        // Compute representative position from all rooms in the floor
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto &room : rooms)
            centroid += room;

        if (!rooms.empty())
            centroid /= static_cast<double>(rooms.size());

        // Final error
        _error = floorPose - centroid;
    }
};

/*!
 * The edge used to connect a Room's centroid (SE3) to a Marker vertex (SE3)
 * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
 */
class EdgeVertexSE3RoomProjectSE3Marker
    : public g2o::BaseBinaryEdge<4,
                                 Eigen::Vector4d,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVertexSE3RoomProjectSE3Marker() :
        g2o::BaseBinaryEdge<4,
                            Eigen::Vector4d,
                            g2o::VertexSE3Expmap,
                            g2o::VertexSE3Expmap>()
    {}
    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_inout) const;

    void computeError()
    {
        // Marker's global pose
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        // Room's center point in global pose
        const g2o::VertexSE3Expmap *p_roomCenterGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        // Calculating poses (in global frame)
        g2o::Isometry3D markerPose = p_markerGpVertex->estimate();
        Eigen::Vector3d roomPose =
            p_roomCenterGpVertex->estimate().translation();

        // Calculating the error
        _error[0] = markerPose.translation()(0) - roomPose(0);
        _error[1] = markerPose.translation()(1) - roomPose(1);
        _error[2] = markerPose.translation()(2) - roomPose(2);
        _error[3] = markerPose.translation().norm();
    }
};

/*!
 * A fixed source/target plane-pair measurement for the inter-map
 * transform factor below. Both plane equations and the sign
 * hypothesis are fixed at construction; the surviving plane is not
 * a graph variable.
 */
struct PlanePairMeasurement
{
    Eigen::Vector3d n_A;   // source plane normal (unit, absorbed map frame)
    double          d_A;   // source plane offset: n_A^T x + d_A = 0
    Eigen::Vector3d n_B;   // target plane normal (unit, surviving map frame)
    double          d_B;   // target plane offset: n_B^T x + d_B = 0
    int             sigma; // fixed sign hypothesis for the source plane (+1/-1)

    PlanePairMeasurement() :
        n_A(Eigen::Vector3d::Zero()),
        d_A(0.0),
        n_B(Eigen::Vector3d::Zero()),
        d_B(0.0),
        sigma(1)
    {}
};

/*!
 * One unary factor on the inter-map SE(3) transform per accepted wall
 * correspondence. Residual is [orientation (2D tangent at n_B), offset
 * (1D)]. Only computeError() is provided: this codebase's
 * VertexSE3Expmap::oplusImpl perturbs on the LEFT
 * (setEstimate(SE3Quat::exp(update) * estimate())), so no analytical
 * Jacobian is derived under the opposite convention; this edge relies
 * on g2o::BaseUnaryEdge's own numerical linearizeOplus()
 * (base_unary_edge.hpp), which probes the vertex's actual oplus() and
 * is therefore correct regardless of the perturbation-side convention
 * (numerical differentiation with central differences is the explicit
 * fallback).
 */
class EdgePlaneTransformSE3
    : public g2o::BaseUnaryEdge<3, PlanePairMeasurement, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePlaneTransformSE3() {}

    bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const g2o::VertexSE3Expmap *v =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::SE3Quat    estimate = v->estimate();
        const Eigen::Matrix3d R        = estimate.rotation().toRotationMatrix();
        const Eigen::Vector3d t        = estimate.translation();

        // Plane.cc:transformPlaneEquation law (scale fixed at 1 here):
        // n' = R n ; d' = sigma*d - n'^T t
        const Eigen::Vector3d predCount =
            static_cast<double>(_measurement.sigma) * (R * _measurement.n_A);
        const double predictedDistance =
            static_cast<double>(_measurement.sigma) * _measurement.d_A -
            predCount.dot(t);

        // Minimal rotation vector (axis-angle) that rotates n_pred onto
        // n_B, i.e. Log_SO3(n_pred, n_B).
        const Eigen::Vector3d &n_B      = _measurement.n_B;
        const Eigen::Vector3d  axis     = predCount.cross(n_B);
        const double           sinAngle = axis.norm();
        const double           cosAngle =
            std::max(-1.0, std::min(1.0, predCount.dot(n_B)));
        const double    angle     = std::atan2(sinAngle, cosAngle);
        Eigen::Vector3d logVector = Eigen::Vector3d::Zero();
        if (sinAngle > 1e-12)
        {
            logVector = (axis / sinAngle) * angle;
        }
        else if (cosAngle < 0.0)
        {
            // Antipodal singularity: axis/sinAngle is 0/0 exactly where
            // the true orientation error is at its maximum (angle ~ pi),
            // not zero -- leaving logVec at zero here would silently
            // mask a 180 degree misalignment as a perfect fit. Any unit
            // vector orthogonal to n_pred is a valid rotation axis at
            // this isolated point; unitOrthogonal() picks one
            // deterministically.
            logVector = predCount.unitOrthogonal() * angle;
        }

        // Orthonormal 2D basis for the tangent plane at n_B (B_B).
        const Eigen::Vector3d referenceAxis = (std::abs(n_B.x()) <= 0.9)
                                                  ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d axisU = n_B.cross(referenceAxis).normalized();
        const Eigen::Vector3d axisV = n_B.cross(axisU).normalized();

        _error[0] = axisU.dot(logVector);
        _error[1] = axisV.dot(logVector);
        _error[2] = predictedDistance - _measurement.d_B;
    }
};
} // namespace core
} // namespace vs_graphs
#endif
