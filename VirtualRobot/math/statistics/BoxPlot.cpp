#include "BoxPlot.h"

#include "measures.h"


namespace math { namespace stat
{


BoxPlot::BoxPlot() = default;


BoxPlot::BoxPlot(const std::vector<float>& values, bool isSorted, float whisk) :
    whisk(whisk)
{
    set(values, isSorted);
}

void BoxPlot::set(const std::vector<float>& _values, bool isSorted)
{
    const std::vector<float>& values = isSorted ? _values : sorted(_values);
    
    this->minimum = stat::min(values, true);
    this->maximum = stat::max(values, true);
    
    this->lowerQuartile = stat::lowerQuartile(values, true);
    this->median = stat::median(values, true);
    this->upperQuartile = stat::upperQuartile(values, true);
    
    float iqr = interquartileRange(lowerQuartile, upperQuartile);
    
    this->minWhisker = std::max(minimum, lowerQuartile - whisk * iqr);
    this->maxWhisker = std::min(maximum, upperQuartile + whisk * iqr);
    
    
    // compute outliers and correct whiskers if necessary
    
    {
        auto it = values.begin();
        for (; it != values.end() && *it < minWhisker; ++it)
        {
            outliers.push_back(*it);
        }
        minWhisker = (it != values.end()) ? *it : minimum;
    }
    
    {
        auto rit = values.rbegin();
        for (; rit != values.rend() && *rit > maxWhisker; ++rit)
        {
            outliers.push_back(*rit);
        }
        maxWhisker = (rit != values.rend()) ? *rit : maximum;
    }
    
}



}}
