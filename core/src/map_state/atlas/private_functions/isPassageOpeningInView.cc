/*!
 * @file            isPassageOpeningInView.cc
 *
 * @brief           Implements isPassageOpeningInView(), declared in
 *                  private_functions.h.
 */

#include "private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"

#include <array>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool isPassageOpeningInView(semantic::Passage             *p_passage_in,
                            keyframe::KeyFrame            *p_keyFrame_in,
                            const Eigen::Vector3d         &upDirection_world_in,
                            const std::pair<float, float> &depthRange_m_in,
                            double                        &openingDepth_m_out)
{
    openingDepth_m_out = 0.0;
    Eigen::Vector3d passageCentre_world{};
    g2o::Plane3D    passagePlane_world{};
    double          openingWidth_m{};
    Sophus::SE3f    cameraPose_worldToCamera{};
    if (p_passage_in->getCentroid(passageCentre_world) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_in->getGlobalEquation(passagePlane_world) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_in->getWidth(openingWidth_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_keyFrame_in->getPose(cameraPose_worldToCamera) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a passage or key frame getter returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* The opening runs across the passage plane, perpendicular to up. */
    const Eigen::Vector3d passageNormal_world =
        passagePlane_world.coeffs().head<3>();
    const Eigen::Vector3d openingAxis_world =
        upDirection_world_in.cross(passageNormal_world);
    if (!passageCentre_world.allFinite() || !openingAxis_world.allFinite() ||
        openingAxis_world.norm() < 1e-6 || !(openingWidth_m > 0.0))
    {
        return false;
    }
    const Eigen::Vector3d halfOpening_world =
        0.5 * openingWidth_m * openingAxis_world.normalized();
    const std::array<Eigen::Vector3d, 3U> openingPoints_world{
        passageCentre_world - halfOpening_world,
        passageCentre_world,
        passageCentre_world + halfOpening_world};

    /* Pinhole projection u = fx * x / z + cx, with the camera z axis along
     * the optical axis; every point must lie in the image columns and in the
     * sensor's depth range. */
    for (const Eigen::Vector3d &openingPoint_world : openingPoints_world)
    {
        const Eigen::Vector3f openingPoint_camera =
            cameraPose_worldToCamera * openingPoint_world.cast<float>();
        const float pointDepth_m = openingPoint_camera.z();
        if (!(pointDepth_m > 0.0f) || pointDepth_m < depthRange_m_in.first ||
            pointDepth_m > depthRange_m_in.second)
        {
            return false;
        }
        const float imageColumn_px =
            p_keyFrame_in->fx * openingPoint_camera.x() / pointDepth_m +
            p_keyFrame_in->cx;
        if (imageColumn_px < static_cast<float>(p_keyFrame_in->gridMinX) ||
            imageColumn_px > static_cast<float>(p_keyFrame_in->gridMaxX))
        {
            return false;
        }
    }
    openingDepth_m_out = static_cast<double>(
        (cameraPose_worldToCamera * passageCentre_world.cast<float>()).z());
    return true;
}

} /* namespace core */
} /* namespace vs_graphs */
