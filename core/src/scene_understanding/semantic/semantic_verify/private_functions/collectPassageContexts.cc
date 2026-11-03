/*!
 * @file            collectPassageContexts.cc
 *
 * @brief           Implements collectPassageContexts(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void collectPassageContexts(const std::vector<Passage *> &passages_in,
                            std::vector<Passage *>       &admittedPassages_out,
                            std::vector<PassageContext>  &contexts_out)
{
    admittedPassages_out.clear();
    contexts_out.clear();
    for (Passage *p_passage : passages_in)
    {
        if (p_passage == nullptr)
        {
            continue;
        }
        bool passageIsBad{};
        if (p_passage->isBad(passageIsBad) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (passageIsBad)
        {
            continue;
        }
        admittedPassages_out.push_back(p_passage);
        contexts_out.push_back(copyPassageContext(p_passage));
    }
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
