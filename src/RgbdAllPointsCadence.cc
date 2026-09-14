#include "RgbdAllPointsCadence.h"

namespace vs_graphs::rgbd
{

AllPointsCadence::AllPointsCadence(
    const std::int64_t periodNanoseconds_in) noexcept :
    periodNanoseconds(periodNanoseconds_in)
{}

bool AllPointsCadence::shouldPublish(
    const std::int64_t  messageTimeNanoseconds_in,
    const std::uint64_t mapRevision_in) noexcept
{
    if (hasPendingReservation)
    {
        return false;
    }

    if (periodNanoseconds <= 0 || !hasPublished ||
        mapRevision_in != lastPublishedMapRevision ||
        messageTimeNanoseconds_in < lastPublishedTimeNanoseconds ||
        static_cast<long double>(messageTimeNanoseconds_in) -
                static_cast<long double>(lastPublishedTimeNanoseconds) >=
            static_cast<long double>(periodNanoseconds))
    {
        previousHasPublished             = hasPublished;
        previousPublishedTimeNanoseconds = lastPublishedTimeNanoseconds;
        previousPublishedMapRevision     = lastPublishedMapRevision;
        hasPublished                     = true;
        lastPublishedTimeNanoseconds     = messageTimeNanoseconds_in;
        lastPublishedMapRevision         = mapRevision_in;
        hasPendingReservation            = true;
        return true;
    }

    return false;
}

void AllPointsCadence::commitPublication() noexcept
{
    hasPendingReservation = false;
}

void AllPointsCadence::rollbackPublication() noexcept
{
    if (hasPendingReservation)
    {
        hasPublished                 = previousHasPublished;
        lastPublishedTimeNanoseconds = previousPublishedTimeNanoseconds;
        lastPublishedMapRevision     = previousPublishedMapRevision;
        hasPendingReservation        = false;
    }
}

} /* namespace vs_graphs::rgbd */
