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
 * @file            TransitionEvaluationContext.h
 *
 * @brief           Declares the (currently empty) context parameter of
 *                  evaluateTransition(), reserved for later phases.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_TRANSITION_EVALUATION_CONTEXT_H
#define SEMANTIC_AXIOM_EVALUATOR_TRANSITION_EVALUATION_CONTEXT_H

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Context for evaluateTransition(), reserved for the dynamic
 *              frame/transaction/merge checks Phase 2/7/8 add.
 *
 *              This foundation slice's evaluateTransition() does not yet
 *              implement AX-FRAME-01/AX-TXN-01/AX-MERGE-01 detection logic
 *              (each reports a fixed UNKNOWN placeholder regardless of
 *              \p before_in/\p after_in), so this type intentionally
 *              carries no fields yet. A later phase that adds real
 *              transition detection extends this type with whatever
 *              deterministic, snapshot-external facts that detection
 *              genuinely needs (e.g. which Sim3 transform, if any, was
 *              applied) -- never with a live pointer, a lock, a ROS type,
 *              or a wall-clock value, matching every other type in this
 *              module.
 */
struct TransitionEvaluationContext
{};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_TRANSITION_EVALUATION_CONTEXT_H
