/*!
 * @file            copyPassageContext.cc
 *
 * @brief           Implements copyPassageContext(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <cmath>
#include <optional>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageContext copyPassageContext(const Passage *p_passage_in)
{
    PassageContext context;
    int            passageId{};
    if (p_passage_in->getId(passageId) != PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    context.id = passageId;
    bool passageIsPassable{};
    if (p_passage_in->isPassable(passageIsPassable) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    context.isPassable = passageIsPassable;
    Eigen::Vector3d passageCentroid{};
    if (p_passage_in->getCentroid(passageCentroid) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    context.passageCentroid_world = passageCentroid;
    double passageWidth{};
    if (p_passage_in->getWidth(passageWidth) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    context.width_m = passageWidth;
    double passageHeight{};
    if (p_passage_in->getHeight(passageHeight) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    context.height_m = passageHeight;
    g2o::Plane3D passageEquation{};
    if (p_passage_in->getGlobalEquation(passageEquation) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d passagePlane      = passageEquation.coeffs();
    const double    passageNormalNorm = passagePlane.head<3>().norm();
    if (passagePlane.allFinite() && passageNormalNorm > 1e-8)
    {
        context.hasPlane            = true;
        context.planeEquation_world = passagePlane / passageNormalNorm;
    }
    bool passageIsRecoveryProxy{};
    if (p_passage_in->isRecoveryProxy(passageIsRecoveryProxy) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRecoveryProxy returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    context.isRecoveryProxy = passageIsRecoveryProxy;
    context.isApertureValid = std::isfinite(context.width_m) &&
                              std::isfinite(context.height_m) &&
                              context.width_m > 0.0 && context.height_m > 0.0;
    Passage::KnownSideProvenance knownSide{};
    if (p_passage_in->getKnownSideProvenance(knownSide) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    context.hasKnownSideRoom = knownSide.p_room != nullptr;
    if (context.hasKnownSideRoom)
    {
        int id2{};
        if (knownSide.p_room->getId(id2) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        context.knownSideRoomId = id2;
    }
    bool knownSideHasDirection{};
    if (knownSide.hasDirection(knownSideHasDirection) !=
        KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    context.hasKnownSideDirection = knownSideHasDirection;
    if (context.hasKnownSideDirection)
    {
        context.knownSideDirection_world = knownSide.knownSideDirection_world;
    }
    std::optional<int> farSideRoomId{};
    if (p_passage_in->getProspectiveRoomId(farSideRoomId) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    context.hasFarSideRoom = farSideRoomId.has_value();
    if (farSideRoomId.has_value())
    {
        context.secondaryRoomId = *farSideRoomId;
    }
    return context;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
