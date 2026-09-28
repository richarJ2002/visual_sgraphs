

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticCandidatesStatus angleSignature(const RoomContextSnapshot &snapshot_in,
                                        const double               tolerance_in,
                                        const std::size_t          cap_in,
                                        std::vector<double> &angleSignature_out)
{
    std::vector<Eigen::Vector3d> normals;
    normals.reserve(std::min(snapshot_in.wallNormals.size(), cap_in));
    for (const Eigen::Vector3d &normal : snapshot_in.wallNormals)
    {
        if (normals.size() == cap_in)
        {
            break;
        }
        if (normal.allFinite() && normal.norm() > 1e-12)
        {
            normals.push_back(normal.normalized());
        }
    }
    std::vector<double> signature;
    const std::size_t   pairCap =
        std::min(cap_in, normals.size() * (normals.size() - 1U) / 2U);
    signature.reserve(pairCap);
    for (std::size_t first = 0U; first < normals.size(); ++first)
    {
        for (std::size_t second = first + 1U; second < normals.size(); ++second)
        {
            if (signature.size() == cap_in)
            {
                break;
            }
            const double dot =
                std::clamp(std::abs(normals[first].dot(normals[second])),
                           0.0,
                           1.0);
            const double angle = std::acos(dot);
            signature.push_back(angle <= tolerance_in ? 0.0 : angle);
        }
        if (signature.size() == cap_in)
        {
            break;
        }
    }
    std::sort(signature.begin(), signature.end());
    angleSignature_out = signature;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
