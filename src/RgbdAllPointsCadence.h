/*!
 * @file RgbdAllPointsCadence.h
 * @brief Package-private simulated-time cadence decision for all map points.
 */

#ifndef VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H
#define VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H

#include <cstdint>

namespace vs_graphs::rgbd
{

/*!
 * @brief        Decides when the all-map-points cloud is due, using message
 *               (simulated) time so the rate follows the sensor clock.
 */
class AllPointsCadence
{
  public:
    /*!
     * @brief        Creates a cadence that publishes at most once per period.
     *
     * @param[in]    periodNanoseconds_in
     *               Minimum time between publications, in nanoseconds; zero or
     *               negative means every packet is due.
     */
    explicit AllPointsCadence(std::int64_t periodNanoseconds_in) noexcept;

    /*!
     * @brief        Tells whether this packet should publish all map points
     *               and, when it should, reserves the publication.
     *
     *               A publication is due on the first call, when the map
     *               revision changed, when message time went backwards, or when
     *               a full period has passed. While a reservation is pending,
     *               no further publication is due; call commitPublication() or
     *               rollbackPublication() to resolve it.
     *
     * @param[in]    messageTimeNanoseconds_in
     *               Message timestamp of the packet, in nanoseconds.
     * @param[in]    mapRevision_in
     *               Revision counter of the map being published.
     *
     * @return       True when the caller should publish now (a reservation is
     *               now pending); false otherwise.
     */
    [[nodiscard]] bool shouldPublish(std::int64_t  messageTimeNanoseconds_in,
                                     std::uint64_t mapRevision_in) noexcept;

    /*!
     * @brief Commits the currently reserved all-points publication.
     *
     * The reservation becomes the cadence state only after the associated
     * publication has completed successfully.
     */
    void commitPublication() noexcept;

    /*!
     * @brief Rolls back the currently reserved all-points publication.
     *
     * A rolled-back reservation does not consume cadence, allowing the next
     * eligible packet to retry publication.
     */
    void rollbackPublication() noexcept;

  private:
    /*!
     * @brief        Minimum time between publications, in nanoseconds.
     */
    const std::int64_t periodNanoseconds;

    /*!
     * @brief        True once any publication has been reserved.
     */
    bool hasPublished{false};

    /*!
     * @brief        Message time, in nanoseconds, of the latest reserved
     *               publication.
     */
    std::int64_t lastPublishedTimeNanoseconds{0};

    /*!
     * @brief        Map revision of the latest reserved publication.
     */
    std::uint64_t lastPublishedMapRevision{0U};

    /*!
     * @brief        True between shouldPublish() returning true and the
     *               matching commit or rollback.
     */
    bool hasPendingReservation{false};

    /*!
     * @brief        Value of hasPublished before the pending reservation,
     *               restored on rollback.
     */
    bool previousHasPublished{false};

    /*!
     * @brief        Value of lastPublishedTimeNanoseconds before the pending
     *               reservation, restored on rollback.
     */
    std::int64_t previousPublishedTimeNanoseconds{0};

    /*!
     * @brief        Value of lastPublishedMapRevision before the pending
     *               reservation, restored on rollback.
     */
    std::uint64_t previousPublishedMapRevision{0U};
};

} /* namespace vs_graphs::rgbd */

#endif /* VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H */
