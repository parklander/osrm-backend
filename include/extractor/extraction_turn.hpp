#ifndef OSRM_EXTRACTION_TURN_HPP
#define OSRM_EXTRACTION_TURN_HPP

#include <boost/numeric/conversion/cast.hpp>

#include <cstdint>

namespace osrm
{
namespace extractor
{

struct ExtractionTurn
{
    static const constexpr std::uint16_t INVALID_TURN_PENALTY =
        std::numeric_limits<std::uint16_t>::max();

    ExtractionTurn(const bool is_uturn_, const double angle_)
        : is_uturn(is_uturn_), angle(angle_), duration(INVALID_TURN_PENALTY),
          weight(INVALID_TURN_PENALTY)
    {
    }

    void SetDuration(const double seconds)
    {
        duration = boost::numeric_cast<std::uint16_t>(seconds * 10);
    }
    double GetDuration() const
    {
        if (duration == INVALID_TURN_PENALTY)
        {
            return -1;
        }
        else
        {
            return duration / 10.;
        }
    }

    void SetWeight(const double floating_weight)
    {
        weight = boost::numeric_cast<std::uint16_t>(floating_weight * 10);
    }
    double GetWeight() const
    {
        if (weight == INVALID_TURN_PENALTY)
        {
            return -1;
        }
        else
        {
            return weight / 10.;
        }
    }

    const bool is_uturn;
    const double angle;
    // we limit the duration/weight of turn penalties to 2^15-1 which roughly translates to 1.5
    // hours
    std::uint16_t duration;
    std::uint16_t weight;
};
}
}

#endif
