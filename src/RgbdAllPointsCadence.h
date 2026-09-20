/*!
 * @file RgbdAllPointsCadence.h
 * @brief Package-private simulated-time cadence decision for all map points.
 */

#ifndef VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H
#define VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H

#include <cstdint>

namespace vs_graphs::rgbd
{

class AllPointsCadence
{
  public:
    explicit AllPointsCadence(std::int64_t periodNanoseconds_in) noexcept;

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
    const std::int64_t periodNanoseconds;
    bool               hasPublished{false};
    std::int64_t       lastPublishedTimeNanoseconds{0};
    std::uint64_t      lastPublishedMapRevision{0U};
    bool               hasPendingReservation{false};
    bool               previousHasPublished{false};
    std::int64_t       previousPublishedTimeNanoseconds{0};
    std::uint64_t      previousPublishedMapRevision{0U};
};

} /* namespace vs_graphs::rgbd */

#endif /* VS_GRAPHS_RGBD_ALL_POINTS_CADENCE_H */
