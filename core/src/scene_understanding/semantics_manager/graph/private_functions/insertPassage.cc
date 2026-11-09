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

/*!
 * @file            insertPassage.cc
 *
 * @brief           Implements insertPassage(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void insertPassage(scene_graph::SceneGraph           &sceneGraph_inout,
                   std::unique_ptr<semantic::Passage> p_passage_in,
                   RetiredNodes                      &retiredNodes_inout)
{
    if (p_passage_in == nullptr)
    {
        return;
    }

    int passageId{};
    if (p_passage_in->getId(passageId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* A negative id, or an id another passage holds, is rejected and left
     * for the caller to resolve. */
    if (passageId >= 0)
    {
        const std::pair<
            std::map<int, std::unique_ptr<semantic::Passage>>::iterator,
            bool>
            passageInsertion =
                sceneGraph_inout.passages.try_emplace(passageId,
                                                      std::move(p_passage_in));
        if (passageInsertion.second)
        {
            return;
        }
    }
    std::cerr << "[Map] Passage ID collision for " << passageId
              << "; caller must resolve it before destination insertion."
              << std::endl;
    retiredNodes_inout.passages.push_back(std::move(p_passage_in));
}

} /* namespace core */
} /* namespace vs_graphs */
