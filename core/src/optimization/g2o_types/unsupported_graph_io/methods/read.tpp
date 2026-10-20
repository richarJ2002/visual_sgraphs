/*!
 * @file            read.tpp
 *
 * @brief           Implements rejection of unsupported g2o text input.
 */

namespace vs_graphs::core::g2o_types::unsupported_graph_io
{

template <typename GraphBase>
bool UnsupportedGraphIo<GraphBase>::read(
    [[maybe_unused]] std::istream &inputStream_inout)
{
    return false;
}

} /* namespace vs_graphs::core::g2o_types::unsupported_graph_io */
