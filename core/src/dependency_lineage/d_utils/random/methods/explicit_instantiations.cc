/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            explicit_instantiations.cc
 *
 * @brief           Instantiates every Random member template for the value
 *                  type the project uses, so that each one is compiled even
 *                  when no caller uses it.
 */

#include "dependency_lineage/d_utils/random/random.h"

namespace vs_graphs::core::d_utils::random
{

/*!
 * @brief           Explicit instantiation of
 *                  Random::generateUniformValue<double>(), which the
 *                  bounded overload calls.
 */
template double Random::generateUniformValue<double>();

/*!
 * @brief           Explicit instantiation of
 *                  Random::generateUniformValue<double>(double, double),
 *                  the form TemplatedVocabulary uses.
 */
template double Random::generateUniformValue<double>(double, double);

/*!
 * @brief           Explicit instantiation of
 *                  Random::generateGaussianValue<double>(double, double);
 *                  no project code calls it, double is the type the
 *                  other templates use.
 */
template double Random::generateGaussianValue<double>(double, double);

} /* namespace vs_graphs::core::d_utils::random */
