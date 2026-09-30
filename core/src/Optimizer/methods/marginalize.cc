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

/*!
 * @file            marginalize.cc
 *
 * @brief           Implements Optimizer::marginalize(), declared in
 *                  Optimizer.h.
 */

#include "Optimizer.h"

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::marginalize(const Eigen::MatrixXd &H_in,
                                       const int             &start_in,
                                       const int             &end_in,
                                       Eigen::MatrixXd       &marginalized_out)
{
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start_in;
    // Size of block to marginalize
    const int b = end_in - start_in + 1;
    // Size of block after block to marginalize
    const int c = H_in.cols() - (end_in + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H_in.rows(), H_in.cols());
    if (a > 0)
    {
        Hn.block(0, 0, a, a)     = H_in.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H_in.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H_in.block(a, 0, b, a);
    }
    if (a > 0 && c > 0)
    {
        Hn.block(0, a, a, c) = H_in.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H_in.block(a + b, 0, c, a);
    }
    if (c > 0)
    {
        Hn.block(a, a, c, c)     = H_in.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H_in.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H_in.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H_in.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b),
                                          Eigen::ComputeThinU |
                                              Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValuesInv =
        svd.singularValues();
    for (int i = 0; i < b; ++i)
    {
        if (singularValuesInv(i) > 1e-6)
            singularValuesInv(i) = 1.0 / singularValuesInv(i);
        else
            singularValuesInv(i) = 0;
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValuesInv.asDiagonal() *
                            svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) =
        Hn.block(0, 0, a + c, a + c) -
        Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(H_in.rows(), H_in.cols());
    if (a > 0)
    {
        result.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        result.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        result.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0)
    {
        result.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        result.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0)
    {
        result.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        result.block(a + b, a, c, b)     = Hn.block(a, a + c, c, b);
        result.block(a, a + b, b, c)     = Hn.block(a + c, a, b, c);
    }

    result.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    marginalized_out = result;
    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
