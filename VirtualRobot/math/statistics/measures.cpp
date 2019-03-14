#include "measures.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>


namespace math { namespace stat
{

static void checkNotEmpty(const std::vector<float>& values)
{
    if (values.empty())
        throw std::range_error("Given values are empty.");
}

void sort(std::vector<float>& values)
{
    std::sort(values.begin(), values.end());
}

std::vector<float> sorted(const std::vector<float>& values)
{
    std::vector<float> s = values;
    std::sort(s.begin(), s.end());
    return s;
}

float min(const std::vector<float>& values, bool isSorted)
{
    checkNotEmpty(values);
    return isSorted ? values.front() : *std::min_element(values.begin(), values.end());
}

float max(const std::vector<float>& values, bool isSorted)
{
    checkNotEmpty(values);
    return isSorted ? values.back() : *std::max_element(values.begin(), values.end());
}

float mean(const std::vector<float>& values)
{
    checkNotEmpty(values);
    
    float sum = 0;
    for (float v : values)
    {
        sum += v;
    }
    return sum / values.size();
}

float stddev(const std::vector<float>& values)
{
    return stddev(values, mean(values));
}

float stddev(const std::vector<float>& values, float mean)
{
    checkNotEmpty(values);
    float sum = 0;
    for (float v : values)
    {
        float diff = v - mean;
        sum += diff * diff;
    }
    float variance = sum / (values.size() - 1);
    return std::sqrt(variance);
}

float quantile(const std::vector<float>& _values, float p, bool isSorted)
{
    checkNotEmpty(_values);
    const std::vector<float>& values = isSorted ? _values : sorted(_values);
    
    float location = p * values.size();
    
    std::size_t floor = static_cast<std::size_t>(std::floor(location));
    std::size_t ceil = static_cast<std::size_t>(std::ceil(location));
    
    if (floor == ceil)
    {
        return values.at(floor);
    }
    else
    {
        float t = location - floor;
        return (1 - t) * values.at(floor) + t * values.at(ceil);
    }
}

float lowerQuartile(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .25, isSorted);
}

float median(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .5, isSorted);
}

float upperQuartile(const std::vector<float>& values, bool isSorted)
{
    return quantile(values, .75, isSorted);
}

float interquartileRange(const std::vector<float>& _values, bool isSorted)
{
    checkNotEmpty(_values);
    
    const std::vector<float>& values = isSorted ? _values : sorted(_values);
    return interquartileRange(lowerQuartile(values, true), upperQuartile(values, true));
}

float interquartileRange(float lowerQuartile, float upperQuartile)
{
    return upperQuartile - lowerQuartile;
}


}}
