/*!
 * @file            OrbVocabulary.h
 *
 * @brief           Shared bag-of-words vocabulary alias used by the keyframe
 *                  database, loop closing and tracking.
 *
 *                  The alias keeps its historical `vs_graphs::core` scope so
 *                  existing use sites are unaffected; it lives with the
 *                  keyframe database because that module owns the vocabulary
 *                  instance.
 */

#ifndef VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_OBJECTS_ORB_VOCABULARY_H
#define VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_OBJECTS_ORB_VOCABULARY_H

#include "dependency_lineage/dbow2/descriptors/descriptors.h"
#include "dependency_lineage/dbow2/vocabulary/vocabulary.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Bag-of-words vocabulary over ORB descriptors.
 */
typedef dbow2::vocabulary::TemplatedVocabulary<
    dbow2::descriptors::ORBDescriptorFunctions::Descriptor,
    dbow2::descriptors::ORBDescriptorFunctions>
    ORBVocabulary;

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_KEYFRAME_DATABASE_OBJECTS_ORB_VOCABULARY_H */
