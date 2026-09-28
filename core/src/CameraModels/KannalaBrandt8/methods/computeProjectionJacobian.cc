/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            computeProjectionJacobian.cc
 *
 * @brief           Implements
 *                  KannalaBrandt8::computeProjectionJacobian(), declared
 *                  in CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
Eigen::Matrix<double, 2, 3>
    KannalaBrandt8::computeProjectionJacobian(const Eigen::Vector3d &point3d_in)
{
    /* Declare loval variables */
    double pointXSquared       = point3d_in[0] * point3d_in[0];
    double pointYSquared       = point3d_in[1] * point3d_in[1];
    double pointZSquared       = point3d_in[2] * point3d_in[2];
    double planarRadiusSquared = pointXSquared + pointYSquared;
    double planarRadius        = sqrt(planarRadiusSquared);
    double planarRadiusCubed   = planarRadiusSquared * planarRadius;
    double incidenceAngle      = atan2(planarRadius, point3d_in[2]);

    double incidenceAngleSquared = incidenceAngle * incidenceAngle;
    double incidenceAngleCubed   = incidenceAngleSquared * incidenceAngle;
    double incidenceAnglePow4 = incidenceAngleSquared * incidenceAngleSquared;
    double incidenceAnglePow5 = incidenceAnglePow4 * incidenceAngle;
    double incidenceAnglePow6 = incidenceAngleSquared * incidenceAnglePow4;
    double incidenceAnglePow7 = incidenceAnglePow6 * incidenceAngle;
    double incidenceAnglePow8 = incidenceAnglePow4 * incidenceAnglePow4;
    double incidenceAnglePow9 = incidenceAnglePow8 * incidenceAngle;

    double distortedIncidenceAngle =
        incidenceAngle + incidenceAngleCubed * parameters[4] +
        incidenceAnglePow5 * parameters[5] +
        incidenceAnglePow7 * parameters[6] + incidenceAnglePow9 * parameters[7];
    double distortedIncidenceAngleDerivative =
        1 + 3 * parameters[4] * incidenceAngleSquared +
        5 * parameters[5] * incidenceAnglePow4 +
        7 * parameters[6] * incidenceAnglePow6 +
        9 * parameters[7] * incidenceAnglePow8;

    Eigen::Matrix<double, 2, 3> projectionJacobian;
    projectionJacobian(0, 0) =
        parameters[0] *
        (distortedIncidenceAngleDerivative * point3d_in[2] * pointXSquared /
             (planarRadiusSquared * (planarRadiusSquared + pointZSquared)) +
         distortedIncidenceAngle * pointYSquared / planarRadiusCubed);
    projectionJacobian(1, 0) =
        parameters[1] *
        (distortedIncidenceAngleDerivative * point3d_in[2] * point3d_in[1] *
             point3d_in[0] /
             (planarRadiusSquared * (planarRadiusSquared + pointZSquared)) -
         distortedIncidenceAngle * point3d_in[1] * point3d_in[0] /
             planarRadiusCubed);

    projectionJacobian(0, 1) =
        parameters[0] *
        (distortedIncidenceAngleDerivative * point3d_in[2] * point3d_in[1] *
             point3d_in[0] /
             (planarRadiusSquared * (planarRadiusSquared + pointZSquared)) -
         distortedIncidenceAngle * point3d_in[1] * point3d_in[0] /
             planarRadiusCubed);
    projectionJacobian(1, 1) =
        parameters[1] *
        (distortedIncidenceAngleDerivative * point3d_in[2] * pointYSquared /
             (planarRadiusSquared * (planarRadiusSquared + pointZSquared)) +
         distortedIncidenceAngle * pointXSquared / planarRadiusCubed);

    projectionJacobian(0, 2) =
        -parameters[0] * distortedIncidenceAngleDerivative * point3d_in[0] /
        (planarRadiusSquared + pointZSquared);
    projectionJacobian(1, 2) =
        -parameters[1] * distortedIncidenceAngleDerivative * point3d_in[1] /
        (planarRadiusSquared + pointZSquared);

    return projectionJacobian;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
