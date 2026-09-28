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

#include "OptimizableTypes.h"

namespace vs_graphs
{
namespace core
{

bool VertexSim3Expmap::read(std::istream &inputStream_inout)
{
    g2o::Vector7d cam2world;
    for (int parameterIndex = 0; parameterIndex < 6; parameterIndex++)
        inputStream_inout >> cam2world[parameterIndex];

    inputStream_inout >> cam2world[6];

    float  cameraParameterValue;
    size_t firstCameraSize{};
    if (p_firstCamera->size(firstCameraSize) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // size cannot fail; continue as before.
    }
    for (size_t parameterIndex = 0; parameterIndex < firstCameraSize;
         parameterIndex++)
    {
        inputStream_inout >> cameraParameterValue;
        if (p_firstCamera->setParameter(cameraParameterValue, parameterIndex) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            // setParameter cannot fail; continue as before.
        }
    }

    size_t secondCameraSize{};
    if (p_secondCamera->size(secondCameraSize) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // size cannot fail; continue as before.
    }
    for (size_t parameterIndex = 0; parameterIndex < secondCameraSize;
         parameterIndex++)
    {
        inputStream_inout >> cameraParameterValue;
        if (p_secondCamera->setParameter(cameraParameterValue,
                                         parameterIndex) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            // setParameter cannot fail; continue as before.
        }
    }

    setEstimate(g2o::Sim3(cam2world).inverse());
    return true;
}

} // namespace core
} // namespace vs_graphs
