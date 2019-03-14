#pragma once

#include <vector>


namespace math { namespace stat
{

    void sort(std::vector<float>& values);
    std::vector<float> sorted(const std::vector<float>& values);

    
    float min(const std::vector<float>& values, bool isSorted=false);
    float max(const std::vector<float>& values, bool isSorted=false);
    float mean(const std::vector<float>& values);
    
    float stddev(const std::vector<float>& values);
    float stddev(const std::vector<float>& values, float mean);
    
    float quantile(const std::vector<float>& values, float p, bool isSorted=false);
    
    float lowerQuartile(const std::vector<float>& values, bool isSorted=false);
    float median(const std::vector<float>& values, bool isSorted=false);
    float upperQuartile(const std::vector<float>& values, bool isSorted=false);
    
    float interquartileRange(const std::vector<float>& values, bool isSorted=false);
    float interquartileRange(float lowerQuartile, float upperQuartile);
    


}}
