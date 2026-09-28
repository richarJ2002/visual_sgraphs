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

#include "GeometricTools.h"

#include "KeyFrame.h"

namespace vs_graphs
{
namespace core
{

GeometricToolsStatus GeometricTools::computeF12(KeyFrame       *&keyFrame1_in,
                                                KeyFrame       *&keyFrame2_in,
                                                Eigen::Matrix3f &f12_out)
{
    Sophus::SE3<float>                    Tc1w = keyFrame1_in->getPose();
    Sophus::Matrix3<float>                Rc1w = Tc1w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

    Sophus::SE3<float>                    Tc2w = keyFrame2_in->getPose();
    Sophus::Matrix3<float>                Rc2w = Tc2w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

    Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
    Eigen::Vector3f        tc1c2 = -Rc1c2 * tc2w + tc1w;

    Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

    const Eigen::Matrix3f K1 = keyFrame1_in->p_camera->toK_();
    const Eigen::Matrix3f K2 = keyFrame2_in->p_camera->toK_();

    f12_out = K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
    return GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
