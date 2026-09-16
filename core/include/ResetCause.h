/*!
 * @file         ResetCause.h
 *
 * @brief        Declares stable internal reset/new-map attribution values.
 */

#ifndef VS_GRAPHS_CORE_RESET_CAUSE_H
#define VS_GRAPHS_CORE_RESET_CAUSE_H

#include <cstdint>
#include <string>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Internal causes observable at reset and new-map
 *               call sites.
 *
 *              The public System::Reset() and System::ResetActiveMap()
 *              API has unknown external callers and retains no cause
 *              while deferring work to the next tracking call. Those
 *              execution points therefore remain explicitly unknown;
 *              known package-owned callers report the exact request
 *              here instead of changing System or Tracking object
 *              layout.
 */
enum class ResetCause : std::uint8_t
{
    /*!
     * @brief        Public reset with no attributed package cause.
     */
    UNATTRIBUTED_PUBLIC_REQUEST                    = 0U,
    /*!
     * @brief        Differing causes merged before one execution.
     */
    MULTIPLE_COALESCED_REQUESTS                    = 1U,
    /*!
     * @brief        Local mapper reported invalid inertial data.
     */
    LOCAL_MAPPER_BAD_IMU                           = 2U,
    /*!
     * @brief        Sensor timestamp moved backwards in time.
     */
    NON_MONOTONIC_SENSOR_TIMESTAMP                 = 3U,
    /*!
     * @brief        Timestamp jump before IMU initialization.
     */
    TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION       = 4U,
    /*!
     * @brief        Timestamp jump before the second IMU bundle
     *               adjustment.
     */
    TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA            = 5U,
    /*!
     * @brief        Timestamp jump after the second IMU bundle
     *               adjustment.
     */
    TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA             = 6U,
    /*!
     * @brief        Visual tracking lost while the map is small.
     */
    VISUAL_TRACKING_LOST_SMALL_MAP                 = 7U,
    /*!
     * @brief        Visual tracking lost before IMU initialization.
     */
    VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION = 8U,
    /*!
     * @brief        Visual tracking lost; a new map is created.
     */
    VISUAL_TRACKING_LOST_NEW_MAP                   = 9U,
    /*!
     * @brief        Initialization lacked enough map points.
     */
    INITIALIZATION_INSUFFICIENT_POINTS             = 10U,
    /*!
     * @brief        Monocular initialization produced an invalid map.
     */
    INITIALIZATION_INVALID_MONOCULAR_MAP           = 11U,
    /*!
     * @brief        Gap in the delivered IMU stream.
     */
    IMU_DELIVERY_GAP                               = 12U,
    /*!
     * @brief        Sensor processing could not keep up.
     */
    SENSOR_PROCESSING_OVERLOAD                     = 13U,
    /*!
     * @brief        Reset requested from the viewer.
     */
    VIEWER_REQUEST                                 = 14U,
    /*!
     * @brief        Dataset changed while the map is small.
     */
    DATASET_CHANGE_SMALL_MAP                       = 15U,
    /*!
     * @brief        Dataset changed; a new map is created.
     */
    DATASET_CHANGE_NEW_MAP                         = 16U
};

/*!
 * @brief        Retains the truthful cause of one deferred
 *               active-map reset execution.
 */
class ResetCauseRetention
{
  public:
    ResetCauseRetention()  = default;
    ~ResetCauseRetention() = default;

    /*!
     * @brief        Retains one deferred reset cause.
     *
     *               The first retained cause wins; a differing later cause
     *               coalesces into MULTIPLE_COALESCED_REQUESTS. Callers
     *               serialize concurrent access.
     *
     * @param[in]    cause_in
     *               Cause to retain.
     */
    void                     retain(ResetCause cause_in) noexcept;
    /*!
     * @brief        Returns the retained cause and clears it.
     *
     * @return       Retained cause, or UNATTRIBUTED_PUBLIC_REQUEST when
     *               no cause was retained.
     */
    [[nodiscard]] ResetCause consume() noexcept;
    /*!
     * @brief        Checks whether a cause is currently retained.
     *
     * @return       True when a cause is retained.
     */
    [[nodiscard]] bool       hasRetainedCause() const noexcept;

  private:
    /*!
     * @brief        Whether a cause is currently retained.
     */
    bool       hasCause{false};
    /*!
     * @brief        Retained cause; meaningful only when hasCause is set.
     */
    ResetCause cause{ResetCause::UNATTRIBUTED_PUBLIC_REQUEST};
};

/*!
 * @brief        Retains a deferred reset cause without changing the
 *               owner's object layout. Thread-safe through an internal
 *               mutex.
 *
 * @param[in]    p_owner_in
 *               Owning object the cause is attributed to; may be null.
 * @param[in]    cause_in
 *               Cause to retain.
 */
void retainResetCause(const void *p_owner_in, ResetCause cause_in);

/*!
 * @brief        Returns and clears the deferred reset cause for one
 *               owner. Thread-safe through an internal mutex.
 *
 * @param[in]    p_owner_in
 *               Owning object the cause was attributed to; may be null.
 *
 * @return       Retained cause, or UNATTRIBUTED_PUBLIC_REQUEST when the
 *               owner has no retained cause.
 */
[[nodiscard]] ResetCause consumeResetCause(const void *p_owner_in);

/*!
 * @brief        Clears any deferred reset cause when its owner is
 *               destroyed. Thread-safe through an internal mutex.
 *
 * @param[in]    p_owner_in
 *               Owning object whose cause is discarded; may be null.
 */
void clearResetCause(const void *p_owner_in) noexcept;

/*!
 * @brief        Action requested or performed at an attributed call
 *               site.
 */
enum class ResetAction : std::uint8_t
{
    /*!
     * @brief        Attribution point at the reset request.
     */
    RESET_ACTIVE_MAP_REQUEST   = 0U,
    /*!
     * @brief        Attribution point at the reset execution.
     */
    RESET_ACTIVE_MAP_EXECUTION = 1U,
    /*!
     * @brief        Attribution point at the new-map execution.
     */
    CREATE_MAP_EXECUTION       = 2U
};

/*!
 * @brief        Maps a reset cause to its stable log name.
 *
 * @param[in]    cause_in
 *               Cause to name.
 *
 * @return       Pointer to a static snake-case name, or "unknown" for
 *               an unmapped value.
 */
[[nodiscard]] const char *resetCauseToString(ResetCause cause_in) noexcept;
/*!
 * @brief        Maps a reset action to its stable log name.
 *
 * @param[in]    action_in
 *               Action to name.
 *
 * @return       Pointer to a static snake-case name, or "unknown" for
 *               an unmapped value.
 */
[[nodiscard]] const char *resetActionToString(ResetAction action_in) noexcept;
/*!
 * @brief        Formats one log line attributing a reset execution.
 *
 * @param[in]    cause_in
 *               Attributed cause.
 * @param[in]    action_in
 *               Attributed action.
 *
 * @return       Text of the form "VSG_RESET_ATTRIBUTION cause=<cause>
 *               action=<action>".
 */
[[nodiscard]] std::string formatResetAttribution(ResetCause  cause_in,
                                                 ResetAction action_in);
/*!
 * @brief        Writes one reset-attribution line to standard output.
 *
 * @param[in]    cause_in
 *               Attributed cause.
 * @param[in]    action_in
 *               Attributed action.
 */
void reportResetAttribution(ResetCause cause_in, ResetAction action_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_RESET_CAUSE_H */
