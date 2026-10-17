/*!
 * @file            UnsupportedGraphIoClass.h
 *
 * @brief           Declares the data-free adapter for mandatory g2o file I/O.
 */

#ifndef VS_GRAPHS_CORE_G2O_TYPES_UNSUPPORTED_GRAPH_IO_CLASS_H
#define VS_GRAPHS_CORE_G2O_TYPES_UNSUPPORTED_GRAPH_IO_CLASS_H

#include <istream>
#include <ostream>

namespace vs_graphs::core::g2o_types::unsupported_graph_io
{

/*!
 * @brief           Rejects g2o text-file I/O for in-memory graph elements.
 *
 * @tparam          GraphBase
 *                  Original g2o vertex or edge base specialization, providing
 *                  virtual read(std::istream&) and write(std::ostream&) const.
 *
 * @note            g2o requires both hooks even when no text format exists.
 *                  This adapter adds no data or ownership, performs no stream
 *                  operations, and leaves graph state unchanged. Optimization
 *                  hooks and any separate Boost archives remain with the
 *                  element. Types with real g2o file I/O do not use it.
 */
template <typename GraphBase> class UnsupportedGraphIo : public GraphBase
{
  public:
    /*!
     * @brief           Preserves default construction of the original g2o base.
     */
    UnsupportedGraphIo() = default;

    /*!
     * @brief           Rejects unsupported g2o text input without consuming it.
     *
     * @param[in,out]   inputStream_inout
     *                  Borrowed stream; bytes, positions, state, formatting,
     *                  exception mask and locale are unchanged, even if failed.
     *
     * @return          false, indicating that no graph element was read.
     *
     * @post            The element and stream are unchanged.
     */
    bool read(std::istream &inputStream_inout) override;

    /*!
     * @brief           Rejects unsupported g2o text output.
     *
     * @param[in,out]   outputStream_inout
     *                  Borrowed stream; bytes, positions, state, formatting,
     *                  exception mask and locale are unchanged, even if failed.
     *
     * @return          false, indicating that no graph element was written.
     *
     * @post            The element and stream are unchanged.
     */
    bool write(std::ostream &outputStream_inout) const override;
};

} /* namespace vs_graphs::core::g2o_types::unsupported_graph_io */

#include "g2o_types/unsupported_graph_io/methods/read.tpp"
#include "g2o_types/unsupported_graph_io/methods/write.tpp"

#endif /* VS_GRAPHS_CORE_G2O_TYPES_UNSUPPORTED_GRAPH_IO_CLASS_H */
