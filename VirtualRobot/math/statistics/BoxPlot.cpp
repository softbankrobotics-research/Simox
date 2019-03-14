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
    
    const float iqr = interquartileRange(lowerQuartile, upperQuartile);
    
    this->minWhisker = lowerQuartile - whisk * iqr;
    this->maxWhisker = upperQuartile + whisk * iqr;
    
    
    // compute outliers and correct whiskers if necessary
    
    {
        auto it = values.begin();
        for (; it != values.end() && *it < minWhisker; ++it)
        {
            outliers.push_back(*it);
        }
        minWhisker = (it != values.begin()) ? *it : minimum;
    }
    
    {
        auto rit = values.rbegin();
        for (; rit != values.rend() && *rit > maxWhisker; ++rit)
        {
            outliers.push_back(*rit);
        }
        maxWhisker = (rit != values.rbegin()) ? *rit : maximum;
    }
    
}



}}
