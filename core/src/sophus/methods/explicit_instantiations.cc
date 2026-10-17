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
 * @brief           Compiles retained Sophus groups and their CRTP forwarders.
 *
 * @note            Move-step spelling is deliberately still upstream Sophus.
 *                  CONVENTIONS L7 and rename map B.10 define this coverage.
 *                  Const-map bases are instantiated member-wise: they have
 *                  no writable storage accessors. No algorithms are added.
 */

#include "sophus/sophus.h"

#include <random>

namespace Sophus
{

/*!
 * @brief           Instantiates single-precision SO2 storage.
 */
template class SO2<float>;
/*!
 * @brief           Instantiates single-precision SO2 owning forwarders.
 */
template class SO2Base<SO2<float>>;
/*!
 * @brief           Instantiates single-precision SO2 mutable-map forwarders.
 */
template class SO2Base<Eigen::Map<SO2<float>>>;
/*!
 * @brief           Instantiates double-precision SO2 storage.
 */
template class SO2<double>;
/*!
 * @brief           Instantiates double-precision SO2 owning forwarders.
 */
template class SO2Base<SO2<double>>;
/*!
 * @brief           Instantiates double-precision SO2 mutable-map forwarders.
 */
template class SO2Base<Eigen::Map<SO2<double>>>;

/*!
 * @brief           Instantiates single-precision SO3 storage.
 */
template class SO3<float>;
/*!
 * @brief           Instantiates single-precision SO3 owning forwarders.
 */
template class SO3Base<SO3<float>>;
/*!
 * @brief           Instantiates single-precision SO3 mutable-map forwarders.
 */
template class SO3Base<Eigen::Map<SO3<float>>>;
/*!
 * @brief           Instantiates double-precision SO3 storage.
 */
template class SO3<double>;
/*!
 * @brief           Instantiates double-precision SO3 owning forwarders.
 */
template class SO3Base<SO3<double>>;
/*!
 * @brief           Instantiates double-precision SO3 mutable-map forwarders.
 */
template class SO3Base<Eigen::Map<SO3<double>>>;

/*!
 * @brief           Instantiates single-precision SE2 storage.
 */
template class SE2<float>;
/*!
 * @brief           Instantiates single-precision SE2 owning forwarders.
 */
template class SE2Base<SE2<float>>;
/*!
 * @brief           Instantiates single-precision SE2 mutable-map forwarders.
 */
template class SE2Base<Eigen::Map<SE2<float>>>;
/*!
 * @brief           Instantiates double-precision SE2 storage.
 */
template class SE2<double>;
/*!
 * @brief           Instantiates double-precision SE2 owning forwarders.
 */
template class SE2Base<SE2<double>>;
/*!
 * @brief           Instantiates double-precision SE2 mutable-map forwarders.
 */
template class SE2Base<Eigen::Map<SE2<double>>>;

/*!
 * @brief           Instantiates single-precision SE3 storage.
 */
template class SE3<float>;
/*!
 * @brief           Instantiates single-precision SE3 owning forwarders.
 */
template class SE3Base<SE3<float>>;
/*!
 * @brief           Instantiates single-precision SE3 mutable-map forwarders.
 */
template class SE3Base<Eigen::Map<SE3<float>>>;
/*!
 * @brief           Instantiates double-precision SE3 storage.
 */
template class SE3<double>;
/*!
 * @brief           Instantiates double-precision SE3 owning forwarders.
 */
template class SE3Base<SE3<double>>;
/*!
 * @brief           Instantiates double-precision SE3 mutable-map forwarders.
 */
template class SE3Base<Eigen::Map<SE3<double>>>;

/*!
 * @brief           Instantiates single-precision RxSO2 storage.
 */
template class RxSO2<float>;
/*!
 * @brief           Instantiates single-precision RxSO2 owning forwarders.
 */
template class RxSO2Base<RxSO2<float>>;
/*!
 * @brief           Instantiates single-precision RxSO2 mutable-map forwarders.
 */
template class RxSO2Base<Eigen::Map<RxSO2<float>>>;
/*!
 * @brief           Instantiates double-precision RxSO2 storage.
 */
template class RxSO2<double>;
/*!
 * @brief           Instantiates double-precision RxSO2 owning forwarders.
 */
template class RxSO2Base<RxSO2<double>>;
/*!
 * @brief           Instantiates double-precision RxSO2 mutable-map forwarders.
 */
template class RxSO2Base<Eigen::Map<RxSO2<double>>>;

/*!
 * @brief           Instantiates single-precision RxSO3 storage.
 */
template class RxSO3<float>;
/*!
 * @brief           Instantiates single-precision RxSO3 owning forwarders.
 */
template class RxSO3Base<RxSO3<float>>;
/*!
 * @brief           Instantiates single-precision RxSO3 mutable-map forwarders.
 */
template class RxSO3Base<Eigen::Map<RxSO3<float>>>;
/*!
 * @brief           Instantiates double-precision RxSO3 storage.
 */
template class RxSO3<double>;
/*!
 * @brief           Instantiates double-precision RxSO3 owning forwarders.
 */
template class RxSO3Base<RxSO3<double>>;
/*!
 * @brief           Instantiates double-precision RxSO3 mutable-map forwarders.
 */
template class RxSO3Base<Eigen::Map<RxSO3<double>>>;

/*!
 * @brief           Instantiates single-precision Sim2 storage.
 */
template class Sim2<float>;
/*!
 * @brief           Instantiates single-precision Sim2 owning forwarders.
 */
template class Sim2Base<Sim2<float>>;
/*!
 * @brief           Instantiates single-precision Sim2 mutable-map forwarders.
 */
template class Sim2Base<Eigen::Map<Sim2<float>>>;
/*!
 * @brief           Instantiates double-precision Sim2 storage.
 */
template class Sim2<double>;
/*!
 * @brief           Instantiates double-precision Sim2 owning forwarders.
 */
template class Sim2Base<Sim2<double>>;
/*!
 * @brief           Instantiates double-precision Sim2 mutable-map forwarders.
 */
template class Sim2Base<Eigen::Map<Sim2<double>>>;

/*!
 * @brief           Instantiates single-precision Sim3 storage.
 */
template class Sim3<float>;
/*!
 * @brief           Instantiates single-precision Sim3 owning forwarders.
 */
template class Sim3Base<Sim3<float>>;
/*!
 * @brief           Instantiates single-precision Sim3 mutable-map forwarders.
 */
template class Sim3Base<Eigen::Map<Sim3<float>>>;
/*!
 * @brief           Instantiates double-precision Sim3 storage.
 */
template class Sim3<double>;
/*!
 * @brief           Instantiates double-precision Sim3 owning forwarders.
 */
template class Sim3Base<Sim3<double>>;
/*!
 * @brief           Instantiates double-precision Sim3 mutable-map forwarders.
 */
template class Sim3Base<Eigen::Map<Sim3<double>>>;

/*!
 * @brief           Compiles only read-only forwarders for const maps (B.10).
 */
template const SO2Base<Eigen::Map<const SO2<float>>>::ComplexT &
    SO2Base<Eigen::Map<const SO2<float>>>::unit_complex() const;
template const SO2Base<Eigen::Map<const SO2<double>>>::ComplexT &
    SO2Base<Eigen::Map<const SO2<double>>>::unit_complex() const;
template const float  *SO2Base<Eigen::Map<const SO2<float>>>::data() const;
template const double *SO2Base<Eigen::Map<const SO2<double>>>::data() const;

template const SO3Base<Eigen::Map<const SO3<float>>>::QuaternionType &
    SO3Base<Eigen::Map<const SO3<float>>>::unit_quaternion() const;
template const SO3Base<Eigen::Map<const SO3<double>>>::QuaternionType &
    SO3Base<Eigen::Map<const SO3<double>>>::unit_quaternion() const;
template const float  *SO3Base<Eigen::Map<const SO3<float>>>::data() const;
template const double *SO3Base<Eigen::Map<const SO3<double>>>::data() const;

template const SE2Base<Eigen::Map<const SE2<float>>>::SO2Type &
    SE2Base<Eigen::Map<const SE2<float>>>::so2() const;
template const SE2Base<Eigen::Map<const SE2<double>>>::SO2Type &
    SE2Base<Eigen::Map<const SE2<double>>>::so2() const;
template const SE2Base<Eigen::Map<const SE2<float>>>::TranslationType &
    SE2Base<Eigen::Map<const SE2<float>>>::translation() const;
template const SE2Base<Eigen::Map<const SE2<double>>>::TranslationType &
    SE2Base<Eigen::Map<const SE2<double>>>::translation() const;
template const SE2Base<Eigen::Map<const SE2<float>>>::SO2Type::ComplexT &
    SE2Base<Eigen::Map<const SE2<float>>>::unit_complex() const;
template const SE2Base<Eigen::Map<const SE2<double>>>::SO2Type::ComplexT &
    SE2Base<Eigen::Map<const SE2<double>>>::unit_complex() const;

template const SE3Base<Eigen::Map<const SE3<float>>>::SO3Type &
    SE3Base<Eigen::Map<const SE3<float>>>::so3() const;
template const SE3Base<Eigen::Map<const SE3<double>>>::SO3Type &
    SE3Base<Eigen::Map<const SE3<double>>>::so3() const;
template const SE3Base<Eigen::Map<const SE3<float>>>::TranslationType &
    SE3Base<Eigen::Map<const SE3<float>>>::translation() const;
template const SE3Base<Eigen::Map<const SE3<double>>>::TranslationType &
    SE3Base<Eigen::Map<const SE3<double>>>::translation() const;
template const SE3Base<Eigen::Map<const SE3<float>>>::QuaternionType &
    SE3Base<Eigen::Map<const SE3<float>>>::unit_quaternion() const;
template const SE3Base<Eigen::Map<const SE3<double>>>::QuaternionType &
    SE3Base<Eigen::Map<const SE3<double>>>::unit_quaternion() const;

template const RxSO2Base<Eigen::Map<const RxSO2<float>>>::ComplexType &
    RxSO2Base<Eigen::Map<const RxSO2<float>>>::complex() const;
template const RxSO2Base<Eigen::Map<const RxSO2<double>>>::ComplexType &
    RxSO2Base<Eigen::Map<const RxSO2<double>>>::complex() const;
template const float  *RxSO2Base<Eigen::Map<const RxSO2<float>>>::data() const;
template const double *RxSO2Base<Eigen::Map<const RxSO2<double>>>::data() const;

template const RxSO3Base<Eigen::Map<const RxSO3<float>>>::QuaternionType &
    RxSO3Base<Eigen::Map<const RxSO3<float>>>::quaternion() const;
template const RxSO3Base<Eigen::Map<const RxSO3<double>>>::QuaternionType &
    RxSO3Base<Eigen::Map<const RxSO3<double>>>::quaternion() const;
template const float  *RxSO3Base<Eigen::Map<const RxSO3<float>>>::data() const;
template const double *RxSO3Base<Eigen::Map<const RxSO3<double>>>::data() const;

template const Sim2Base<Eigen::Map<const Sim2<float>>>::RxSO2Type &
    Sim2Base<Eigen::Map<const Sim2<float>>>::rxso2() const;
template const Sim2Base<Eigen::Map<const Sim2<double>>>::RxSO2Type &
    Sim2Base<Eigen::Map<const Sim2<double>>>::rxso2() const;
template const Sim2Base<Eigen::Map<const Sim2<float>>>::TranslationType &
    Sim2Base<Eigen::Map<const Sim2<float>>>::translation() const;
template const Sim2Base<Eigen::Map<const Sim2<double>>>::TranslationType &
    Sim2Base<Eigen::Map<const Sim2<double>>>::translation() const;
template const Sim2Base<Eigen::Map<const Sim2<float>>>::RxSO2Type::ComplexType &
    Sim2Base<Eigen::Map<const Sim2<float>>>::complex() const;
template const Sim2Base<
    Eigen::Map<const Sim2<double>>>::RxSO2Type::ComplexType &
    Sim2Base<Eigen::Map<const Sim2<double>>>::complex() const;

template const Sim3Base<Eigen::Map<const Sim3<float>>>::RxSO3Type &
    Sim3Base<Eigen::Map<const Sim3<float>>>::rxso3() const;
template const Sim3Base<Eigen::Map<const Sim3<double>>>::RxSO3Type &
    Sim3Base<Eigen::Map<const Sim3<double>>>::rxso3() const;
template const Sim3Base<Eigen::Map<const Sim3<float>>>::TranslationType &
    Sim3Base<Eigen::Map<const Sim3<float>>>::translation() const;
template const Sim3Base<Eigen::Map<const Sim3<double>>>::TranslationType &
    Sim3Base<Eigen::Map<const Sim3<double>>>::translation() const;
template const Sim3Base<Eigen::Map<const Sim3<float>>>::QuaternionType &
    Sim3Base<Eigen::Map<const Sim3<float>>>::quaternion() const;
template const Sim3Base<Eigen::Map<const Sim3<double>>>::QuaternionType &
    Sim3Base<Eigen::Map<const Sim3<double>>>::quaternion() const;

/*!
 * @brief           Compiles both scalar-conversion member specializations.
 */
template SO2<float>    SO2Base<SO2<float>>::cast<float>() const;
template SO2<double>   SO2Base<SO2<float>>::cast<double>() const;
template SO2<float>    SO2Base<SO2<double>>::cast<float>() const;
template SO2<double>   SO2Base<SO2<double>>::cast<double>() const;
template SO3<float>    SO3Base<SO3<float>>::cast<float>() const;
template SO3<double>   SO3Base<SO3<float>>::cast<double>() const;
template SO3<float>    SO3Base<SO3<double>>::cast<float>() const;
template SO3<double>   SO3Base<SO3<double>>::cast<double>() const;
template SE2<float>    SE2Base<SE2<float>>::cast<float>() const;
template SE2<double>   SE2Base<SE2<float>>::cast<double>() const;
template SE2<float>    SE2Base<SE2<double>>::cast<float>() const;
template SE2<double>   SE2Base<SE2<double>>::cast<double>() const;
template SE3<float>    SE3Base<SE3<float>>::cast<float>() const;
template SE3<double>   SE3Base<SE3<float>>::cast<double>() const;
template SE3<float>    SE3Base<SE3<double>>::cast<float>() const;
template SE3<double>   SE3Base<SE3<double>>::cast<double>() const;
/*!
 * Upstream RxSO2 cross-precision casts are ambiguous with Eigen 3.4, also
 * through Sim2. Keep the valid same-precision specializations in this move;
 * the original templates remain unchanged. See resume-source-delivery.md.
 */
template RxSO2<float>  RxSO2Base<RxSO2<float>>::cast<float>() const;
template RxSO2<double> RxSO2Base<RxSO2<double>>::cast<double>() const;
template RxSO3<float>  RxSO3Base<RxSO3<float>>::cast<float>() const;
template RxSO3<double> RxSO3Base<RxSO3<float>>::cast<double>() const;
template RxSO3<float>  RxSO3Base<RxSO3<double>>::cast<float>() const;
template RxSO3<double> RxSO3Base<RxSO3<double>>::cast<double>() const;
template Sim2<float>   Sim2Base<Sim2<float>>::cast<float>() const;
template Sim2<double>  Sim2Base<Sim2<double>>::cast<double>() const;
template Sim3<float>   Sim3Base<Sim3<float>>::cast<float>() const;
template Sim3<double>  Sim3Base<Sim3<float>>::cast<double>() const;
template Sim3<float>   Sim3Base<Sim3<double>>::cast<float>() const;
template Sim3<double>  Sim3Base<Sim3<double>>::cast<double>() const;

/*!
 * @brief           Compiles assignment, composition and both point actions.
 */
template SO2Base<SO2<float>> &
    SO2Base<SO2<float>>::operator= <SO2<float>>(const SO2Base<SO2<float>> &);
template SO2<float> SO2Base<SO2<float>>::operator*
    <SO2<float>>(const SO2Base<SO2<float>> &) const;
template Vector2<float> SO2Base<SO2<float>>::operator*
    <Vector2<float>>(const Eigen::MatrixBase<Vector2<float>> &) const;
template Vector3<float> SO2Base<SO2<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template SO2Base<SO2<float>> SO2Base<SO2<float>>::operator*=
    <SO2<float>>(const SO2Base<SO2<float>> &);
template SO2Base<SO2<double>> &
    SO2Base<SO2<double>>::operator= <SO2<double>>(const SO2Base<SO2<double>> &);
template SO2<double> SO2Base<SO2<double>>::operator*
    <SO2<double>>(const SO2Base<SO2<double>> &) const;
template Vector2<double> SO2Base<SO2<double>>::operator*
    <Vector2<double>>(const Eigen::MatrixBase<Vector2<double>> &) const;
template Vector3<double> SO2Base<SO2<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template SO2Base<SO2<double>> SO2Base<SO2<double>>::operator*=
    <SO2<double>>(const SO2Base<SO2<double>> &);

template SO3Base<SO3<float>> &
    SO3Base<SO3<float>>::operator= <SO3<float>>(const SO3Base<SO3<float>> &);
template SO3<float> SO3Base<SO3<float>>::operator*
    <SO3<float>>(const SO3Base<SO3<float>> &) const;
template Vector3<float> SO3Base<SO3<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template Vector4<float> SO3Base<SO3<float>>::operator*
    <Vector4<float>>(const Eigen::MatrixBase<Vector4<float>> &) const;
template SO3Base<SO3<float>> &SO3Base<SO3<float>>::operator*=
    <SO3<float>>(const SO3Base<SO3<float>> &);
template SO3Base<SO3<double>> &
    SO3Base<SO3<double>>::operator= <SO3<double>>(const SO3Base<SO3<double>> &);
template SO3<double> SO3Base<SO3<double>>::operator*
    <SO3<double>>(const SO3Base<SO3<double>> &) const;
template Vector3<double> SO3Base<SO3<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template Vector4<double> SO3Base<SO3<double>>::operator*
    <Vector4<double>>(const Eigen::MatrixBase<Vector4<double>> &) const;
template SO3Base<SO3<double>> &SO3Base<SO3<double>>::operator*=
    <SO3<double>>(const SO3Base<SO3<double>> &);

template SE2Base<SE2<float>> &
    SE2Base<SE2<float>>::operator= <SE2<float>>(const SE2Base<SE2<float>> &);
template SE2<float> SE2Base<SE2<float>>::operator*
    <SE2<float>>(const SE2Base<SE2<float>> &) const;
template Vector2<float> SE2Base<SE2<float>>::operator*
    <Vector2<float>>(const Eigen::MatrixBase<Vector2<float>> &) const;
template Vector3<float> SE2Base<SE2<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template SE2Base<SE2<float>> &SE2Base<SE2<float>>::operator*=
    <SE2<float>>(const SE2Base<SE2<float>> &);
template SE2Base<SE2<double>> &
    SE2Base<SE2<double>>::operator= <SE2<double>>(const SE2Base<SE2<double>> &);
template SE2<double> SE2Base<SE2<double>>::operator*
    <SE2<double>>(const SE2Base<SE2<double>> &) const;
template Vector2<double> SE2Base<SE2<double>>::operator*
    <Vector2<double>>(const Eigen::MatrixBase<Vector2<double>> &) const;
template Vector3<double> SE2Base<SE2<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template SE2Base<SE2<double>> &SE2Base<SE2<double>>::operator*=
    <SE2<double>>(const SE2Base<SE2<double>> &);

template SE3Base<SE3<float>> &
    SE3Base<SE3<float>>::operator= <SE3<float>>(const SE3Base<SE3<float>> &);
template SE3<float> SE3Base<SE3<float>>::operator*
    <SE3<float>>(const SE3Base<SE3<float>> &) const;
template Vector3<float> SE3Base<SE3<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template Vector4<float> SE3Base<SE3<float>>::operator*
    <Vector4<float>>(const Eigen::MatrixBase<Vector4<float>> &) const;
template SE3Base<SE3<float>> &SE3Base<SE3<float>>::operator*=
    <SE3<float>>(const SE3Base<SE3<float>> &);
template SE3Base<SE3<double>> &
    SE3Base<SE3<double>>::operator= <SE3<double>>(const SE3Base<SE3<double>> &);
template SE3<double> SE3Base<SE3<double>>::operator*
    <SE3<double>>(const SE3Base<SE3<double>> &) const;
template Vector3<double> SE3Base<SE3<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template Vector4<double> SE3Base<SE3<double>>::operator*
    <Vector4<double>>(const Eigen::MatrixBase<Vector4<double>> &) const;
template SE3Base<SE3<double>> &SE3Base<SE3<double>>::operator*=
    <SE3<double>>(const SE3Base<SE3<double>> &);

template RxSO2Base<RxSO2<float>> &RxSO2Base<RxSO2<float>>::operator=
    <RxSO2<float>>(const RxSO2Base<RxSO2<float>> &);
template RxSO2<float> RxSO2Base<RxSO2<float>>::operator*
    <RxSO2<float>>(const RxSO2Base<RxSO2<float>> &) const;
template Vector2<float> RxSO2Base<RxSO2<float>>::operator*
    <Vector2<float>>(const Eigen::MatrixBase<Vector2<float>> &) const;
template Vector3<float> RxSO2Base<RxSO2<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template RxSO2Base<RxSO2<float>> &RxSO2Base<RxSO2<float>>::operator*=
    <RxSO2<float>>(const RxSO2Base<RxSO2<float>> &);
template RxSO2Base<RxSO2<double>> &RxSO2Base<RxSO2<double>>::operator=
    <RxSO2<double>>(const RxSO2Base<RxSO2<double>> &);
template RxSO2<double> RxSO2Base<RxSO2<double>>::operator*
    <RxSO2<double>>(const RxSO2Base<RxSO2<double>> &) const;
template Vector2<double> RxSO2Base<RxSO2<double>>::operator*
    <Vector2<double>>(const Eigen::MatrixBase<Vector2<double>> &) const;
template Vector3<double> RxSO2Base<RxSO2<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template RxSO2Base<RxSO2<double>> &RxSO2Base<RxSO2<double>>::operator*=
    <RxSO2<double>>(const RxSO2Base<RxSO2<double>> &);

template RxSO3Base<RxSO3<float>> &RxSO3Base<RxSO3<float>>::operator=
    <RxSO3<float>>(const RxSO3Base<RxSO3<float>> &);
template RxSO3<float> RxSO3Base<RxSO3<float>>::operator*
    <RxSO3<float>>(const RxSO3Base<RxSO3<float>> &) const;
template Vector3<float> RxSO3Base<RxSO3<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template Vector4<float> RxSO3Base<RxSO3<float>>::operator*
    <Vector4<float>>(const Eigen::MatrixBase<Vector4<float>> &) const;
template RxSO3Base<RxSO3<float>> &RxSO3Base<RxSO3<float>>::operator*=
    <RxSO3<float>>(const RxSO3Base<RxSO3<float>> &);
template RxSO3Base<RxSO3<double>> &RxSO3Base<RxSO3<double>>::operator=
    <RxSO3<double>>(const RxSO3Base<RxSO3<double>> &);
template RxSO3<double> RxSO3Base<RxSO3<double>>::operator*
    <RxSO3<double>>(const RxSO3Base<RxSO3<double>> &) const;
template Vector3<double> RxSO3Base<RxSO3<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template Vector4<double> RxSO3Base<RxSO3<double>>::operator*
    <Vector4<double>>(const Eigen::MatrixBase<Vector4<double>> &) const;
template RxSO3Base<RxSO3<double>> &RxSO3Base<RxSO3<double>>::operator*=
    <RxSO3<double>>(const RxSO3Base<RxSO3<double>> &);

template Sim2Base<Sim2<float>> &Sim2Base<Sim2<float>>::operator=
    <Sim2<float>>(const Sim2Base<Sim2<float>> &);
template Sim2<float> Sim2Base<Sim2<float>>::operator*
    <Sim2<float>>(const Sim2Base<Sim2<float>> &) const;
template Vector2<float> Sim2Base<Sim2<float>>::operator*
    <Vector2<float>>(const Eigen::MatrixBase<Vector2<float>> &) const;
template Vector3<float> Sim2Base<Sim2<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template Sim2Base<Sim2<float>> &Sim2Base<Sim2<float>>::operator*=
    <Sim2<float>>(const Sim2Base<Sim2<float>> &);
template Sim2Base<Sim2<double>> &Sim2Base<Sim2<double>>::operator=
    <Sim2<double>>(const Sim2Base<Sim2<double>> &);
template Sim2<double> Sim2Base<Sim2<double>>::operator*
    <Sim2<double>>(const Sim2Base<Sim2<double>> &) const;
template Vector2<double> Sim2Base<Sim2<double>>::operator*
    <Vector2<double>>(const Eigen::MatrixBase<Vector2<double>> &) const;
template Vector3<double> Sim2Base<Sim2<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template Sim2Base<Sim2<double>> &Sim2Base<Sim2<double>>::operator*=
    <Sim2<double>>(const Sim2Base<Sim2<double>> &);

template Sim3Base<Sim3<float>> &Sim3Base<Sim3<float>>::operator=
    <Sim3<float>>(const Sim3Base<Sim3<float>> &);
template Sim3<float> Sim3Base<Sim3<float>>::operator*
    <Sim3<float>>(const Sim3Base<Sim3<float>> &) const;
template Vector3<float> Sim3Base<Sim3<float>>::operator*
    <Vector3<float>>(const Eigen::MatrixBase<Vector3<float>> &) const;
template Vector4<float> Sim3Base<Sim3<float>>::operator*
    <Vector4<float>>(const Eigen::MatrixBase<Vector4<float>> &) const;
template Sim3Base<Sim3<float>> &Sim3Base<Sim3<float>>::operator*=
    <Sim3<float>>(const Sim3Base<Sim3<float>> &);
template Sim3Base<Sim3<double>> &Sim3Base<Sim3<double>>::operator=
    <Sim3<double>>(const Sim3Base<Sim3<double>> &);
template Sim3<double> Sim3Base<Sim3<double>>::operator*
    <Sim3<double>>(const Sim3Base<Sim3<double>> &) const;
template Vector3<double> Sim3Base<Sim3<double>>::operator*
    <Vector3<double>>(const Eigen::MatrixBase<Vector3<double>> &) const;
template Vector4<double> Sim3Base<Sim3<double>>::operator*
    <Vector4<double>>(const Eigen::MatrixBase<Vector4<double>> &) const;
template Sim3Base<Sim3<double>> &Sim3Base<Sim3<double>>::operator*=
    <Sim3<double>>(const Sim3Base<Sim3<double>> &);

/*!
 * @brief           Compiles floating-point factories and rotation angles.
 */
template SO2<float>  SO2<float>::fitToSO2<float>(const Transformation &);
template SO2<double> SO2<double>::fitToSO2<double>(const Transformation &);
template SO3<float>  SO3<float>::fitToSO3<float>(const Transformation &);
template SO3<double> SO3<double>::fitToSO3<double>(const Transformation &);
template SE2<float>  SE2<float>::fitToSE2<float>(const Matrix3<float> &);
template SE2<double> SE2<double>::fitToSE2<double>(const Matrix3<double> &);
template SE3<float>  SE3<float>::fitToSE3<float>(const Matrix4<float> &);
template SE3<double> SE3<double>::fitToSE3<double>(const Matrix4<double> &);
template float       SO3Base<SO3<float>>::angleX<float>() const;
template float       SO3Base<SO3<float>>::angleY<float>() const;
template float       SO3Base<SO3<float>>::angleZ<float>() const;
template double      SO3Base<SO3<double>>::angleX<double>() const;
template double      SO3Base<SO3<double>>::angleY<double>() const;
template double      SO3Base<SO3<double>>::angleZ<double>() const;

/*!
 * @brief           Compiles sampler templates without executing a generator.
 */
template SO2<float>   SO2<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template SO2<double>  SO2<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template SO3<float>   SO3<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template SO3<double>  SO3<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template SE2<float>   SE2<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template SE2<double>  SE2<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template SE3<float>   SE3<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template SE3<double>  SE3<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template RxSO2<float> RxSO2<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template RxSO2<double>
    RxSO2<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template RxSO3<float> RxSO3<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template RxSO3<double>
                     RxSO3<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template Sim2<float> Sim2<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template Sim2<double> Sim2<double>::sampleUniform<std::mt19937>(std::mt19937 &);
template Sim3<float>  Sim3<float>::sampleUniform<std::mt19937>(std::mt19937 &);
template Sim3<double> Sim3<double>::sampleUniform<std::mt19937>(std::mt19937 &);

/*!
 * @brief           Compiles coordinate translation and copy-like constructors.
 */
template SE2<float>  SE2<float>::trans<float, float>(const float &,
                                                    const float &);
template SE2<double> SE2<double>::trans<double, double>(const double &,
                                                        const double &);
template SE3<float>  SE3<float>::trans<float, float, float>(const float &,
                                                           const float &,
                                                           const float &);
template SE3<double> SE3<double>::trans<double, double, double>(const double &,
                                                                const double &,
                                                                const double &);
template SO2<float>::SO2(const SO2Base<Eigen::Map<SO2<float>>> &);
template SO2<double>::SO2(const SO2Base<Eigen::Map<SO2<double>>> &);
template SO3<float>::SO3(const SO3Base<Eigen::Map<SO3<float>>> &);
template SO3<double>::SO3(const SO3Base<Eigen::Map<SO3<double>>> &);
template SE2<float>::SE2(const SE2Base<Eigen::Map<SE2<float>>> &);
template SE2<double>::SE2(const SE2Base<Eigen::Map<SE2<double>>> &);
template SE3<float>::SE3(const SE3Base<Eigen::Map<SE3<float>>> &);
template SE3<double>::SE3(const SE3Base<Eigen::Map<SE3<double>>> &);
template RxSO2<float>::RxSO2(const RxSO2Base<Eigen::Map<RxSO2<float>>> &);
template RxSO2<double>::RxSO2(const RxSO2Base<Eigen::Map<RxSO2<double>>> &);
template RxSO3<float>::RxSO3(const RxSO3Base<Eigen::Map<RxSO3<float>>> &);
template RxSO3<double>::RxSO3(const RxSO3Base<Eigen::Map<RxSO3<double>>> &);
template Sim2<float>::Sim2(const Sim2Base<Eigen::Map<Sim2<float>>> &);
template Sim2<double>::Sim2(const Sim2Base<Eigen::Map<Sim2<double>>> &);
template Sim3<float>::Sim3(const Sim3Base<Eigen::Map<Sim3<float>>> &);
template Sim3<double>::Sim3(const Sim3Base<Eigen::Map<Sim3<double>>> &);

} /* namespace Sophus */

namespace Eigen
{

/*!
 * @brief           Instantiates mutable single-precision SO2 maps.
 */
template class Map<Sophus::SO2<float>>;
/*!
 * @brief           Instantiates const single-precision SO2 maps.
 */
template class Map<const Sophus::SO2<float>>;
/*!
 * @brief           Instantiates mutable double-precision SO2 maps.
 */
template class Map<Sophus::SO2<double>>;
/*!
 * @brief           Instantiates const double-precision SO2 maps.
 */
template class Map<const Sophus::SO2<double>>;

/*!
 * @brief           Instantiates mutable single-precision SO3 maps.
 */
template class Map<Sophus::SO3<float>>;
/*!
 * @brief           Instantiates const single-precision SO3 maps.
 */
template class Map<const Sophus::SO3<float>>;
/*!
 * @brief           Instantiates mutable double-precision SO3 maps.
 */
template class Map<Sophus::SO3<double>>;
/*!
 * @brief           Instantiates const double-precision SO3 maps.
 */
template class Map<const Sophus::SO3<double>>;

/*!
 * @brief           Instantiates mutable single-precision SE2 maps.
 */
template class Map<Sophus::SE2<float>>;
/*!
 * @brief           Instantiates const single-precision SE2 maps.
 */
template class Map<const Sophus::SE2<float>>;
/*!
 * @brief           Instantiates mutable double-precision SE2 maps.
 */
template class Map<Sophus::SE2<double>>;
/*!
 * @brief           Instantiates const double-precision SE2 maps.
 */
template class Map<const Sophus::SE2<double>>;

/*!
 * @brief           Instantiates mutable single-precision SE3 maps.
 */
template class Map<Sophus::SE3<float>>;
/*!
 * @brief           Instantiates const single-precision SE3 maps.
 */
template class Map<const Sophus::SE3<float>>;
/*!
 * @brief           Instantiates mutable double-precision SE3 maps.
 */
template class Map<Sophus::SE3<double>>;
/*!
 * @brief           Instantiates const double-precision SE3 maps.
 */
template class Map<const Sophus::SE3<double>>;

/*!
 * @brief           Instantiates mutable single-precision RxSO2 maps.
 */
template class Map<Sophus::RxSO2<float>>;
/*!
 * @brief           Instantiates const single-precision RxSO2 maps.
 */
template class Map<const Sophus::RxSO2<float>>;
/*!
 * @brief           Instantiates mutable double-precision RxSO2 maps.
 */
template class Map<Sophus::RxSO2<double>>;
/*!
 * @brief           Instantiates const double-precision RxSO2 maps.
 */
template class Map<const Sophus::RxSO2<double>>;

/*!
 * @brief           Instantiates mutable single-precision RxSO3 maps.
 */
template class Map<Sophus::RxSO3<float>>;
/*!
 * @brief           Instantiates const single-precision RxSO3 maps.
 */
template class Map<const Sophus::RxSO3<float>>;
/*!
 * @brief           Instantiates mutable double-precision RxSO3 maps.
 */
template class Map<Sophus::RxSO3<double>>;
/*!
 * @brief           Instantiates const double-precision RxSO3 maps.
 */
template class Map<const Sophus::RxSO3<double>>;

/*!
 * @brief           Instantiates mutable single-precision Sim2 maps.
 */
template class Map<Sophus::Sim2<float>>;
/*!
 * @brief           Instantiates const single-precision Sim2 maps.
 */
template class Map<const Sophus::Sim2<float>>;
/*!
 * @brief           Instantiates mutable double-precision Sim2 maps.
 */
template class Map<Sophus::Sim2<double>>;
/*!
 * @brief           Instantiates const double-precision Sim2 maps.
 */
template class Map<const Sophus::Sim2<double>>;

/*!
 * @brief           Instantiates mutable single-precision Sim3 maps.
 */
template class Map<Sophus::Sim3<float>>;
/*!
 * @brief           Instantiates const single-precision Sim3 maps.
 */
template class Map<const Sophus::Sim3<float>>;
/*!
 * @brief           Instantiates mutable double-precision Sim3 maps.
 */
template class Map<Sophus::Sim3<double>>;
/*!
 * @brief           Instantiates const double-precision Sim3 maps.
 */
template class Map<const Sophus::Sim3<double>>;

} /* namespace Eigen */
