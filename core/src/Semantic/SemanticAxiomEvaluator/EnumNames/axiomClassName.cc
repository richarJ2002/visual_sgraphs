/**
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
 * @file            axiomClassName.cc
 *
 * @brief           Implements axiomClassName(), declared in EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace ORB_SLAM3
{
namespace semantic
{

std::string axiomClassName(AxiomClass class_in)
{
    switch (class_in)
    {
    case AxiomClass::HARD:
        return "HARD";
    case AxiomClass::DERIVED:
        return "DERIVED";
    }
    return "UNKNOWN_AXIOM_CLASS";
}

} // namespace semantic
} // namespace ORB_SLAM3
