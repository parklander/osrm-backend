#ifndef OSRM_EXTRACTION_TURN_HPP
#define OSRM_EXTRACTION_TURN_HPP

namespace osrm
{
namespace extractor
{

struct ExtractionTurn
{
    ExtractionTurn(const bool is_uturn_, const double angle_)
        : is_uturn(is_uturn_), angle(angle_), duration(-1), weight(-1)
    {
    }

    const bool is_uturn;
    const double angle;
    double duration;
    EdgeWeight weight;
};
}
}

#endif
