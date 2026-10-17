/*!
 * @file            StereoMatchOutlierRejectionStatusEnum.h
 *
 * @brief           Declares the status returned by every
 *                  StereoMatchOutlierRejection operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_OBJECTS_STEREOMATCHOUTLIERREJECTION_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_OBJECTS_STEREOMATCHOUTLIERREJECTION_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace stereo_match_outlier_rejection
{

/*!
 * @brief           Result of a StereoMatchOutlierRejection operation. Values
 *                  are fixed and never reordered.
 */
enum class StereoMatchOutlierRejectionStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS = 0U
};

} /* namespace stereo_match_outlier_rejection */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_STEREO_MATCH_OUTLIER_REJECTION_OBJECTS_STEREOMATCHOUTLIERREJECTION_STATUS_ENUM_H \
        */
