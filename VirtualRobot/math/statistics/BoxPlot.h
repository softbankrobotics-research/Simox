#pragma once

#include <vector>


namespace math { namespace stat
{

    /**
     * @brief Computes and stores statistical measures found in a box plot.
     */
    class BoxPlot
    {
    public:
        
        
        BoxPlot();
        BoxPlot(const std::vector<float>& values, bool isSorted = false, float whisk = 1.5);
        
        void set(const std::vector<float>& values, bool isSorted = false);
        
        float whisk = 1.5;
        
        float minimum;
        float minWhisker;
        float lowerQuartile;
        float median;
        float mean;
        float upperQuartile;
        float maxWhisker;
        float maximum;
        
        std::vector<float> outliers;
        
    };

}}

