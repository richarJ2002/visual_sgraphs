/*!
 * @file            write.tpp
 *
 * @brief           Implements rejection of unsupported g2o text output.
 */

namespace vs_graphs::core::g2o_types::unsupported_graph_io
{

template <typename GraphBase>
bool UnsupportedGraphIo<GraphBase>::write(
    [[maybe_unused]] std::ostream &outputStream_inout) const
{
    return false;
}

} /* namespace vs_graphs::core::g2o_types::unsupported_graph_io */
