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

#include "Atlas.h"

namespace vs_graphs
{
namespace core
{

camera_models::geometriccamera::GeometricCamera *Atlas::addCamera(
    camera_models::geometriccamera::GeometricCamera *p_camera_in)
{
    // Check if the camera already exists
    bool isAlreadyInMap     = false;
    int  matchedCameraIndex = -1;
    for (size_t cameraIndex = 0; cameraIndex < cameras.size(); ++cameraIndex)
    {
        camera_models::geometriccamera::GeometricCamera *p_existingCamera =
            cameras[cameraIndex];
        if (!p_camera_in)
            std::cout << "Not pCam" << std::endl;
        if (!p_existingCamera)
            std::cout << "Not pCam_i" << std::endl;
        if (p_camera_in->getType() != p_existingCamera->getType())
            continue;

        if (p_camera_in->getType() ==
            camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE)
        {
            if (((camera_models::pinhole::Pinhole *)p_existingCamera)
                    ->isEqual(p_camera_in))
            {
                isAlreadyInMap     = true;
                matchedCameraIndex = cameraIndex;
            }
        }
        else if (p_camera_in->getType() ==
                 camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE)
        {
            if (((camera_models::kannalabrandt8::KannalaBrandt8 *)
                     p_existingCamera)
                    ->isEqual(p_camera_in))
            {
                isAlreadyInMap     = true;
                matchedCameraIndex = cameraIndex;
            }
        }
    }

    if (isAlreadyInMap)
    {
        return cameras[matchedCameraIndex];
    }
    else
    {
        cameras.push_back(p_camera_in);
        return p_camera_in;
    }
}

} // namespace core
} // namespace vs_graphs
