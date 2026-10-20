/*!
 * @file            utils/converter/methods/toCvMat.tpp
 *
 * @brief           Implements the Converter::toCvMat() template taking an
 *                  Eigen matrix or vector, declared in
 *                  utils/converter/objects/ConverterClass.h.
 */

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

template <typename Derived>
ConverterStatus Converter::toCvMat(const Eigen::MatrixBase<Derived> &matrix_in,
                                   cv::Mat                          &cvMat_out)
{
    cv::Mat cvMatrix(static_cast<int>(matrix_in.rows()),
                     static_cast<int>(matrix_in.cols()),
                     CV_32F);
    for (int rowIndex = 0; rowIndex < cvMatrix.rows; rowIndex++)
        for (int columnIndex = 0; columnIndex < cvMatrix.cols; columnIndex++)
            cvMatrix.at<float>(rowIndex, columnIndex) =
                static_cast<float>(matrix_in(rowIndex, columnIndex));

    /* cvMatrix owns a fresh buffer, so the result never aliases a matrix the
     * caller passed in as cvMat_out. */
    cvMat_out = cvMatrix;
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} /* namespace converter */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
