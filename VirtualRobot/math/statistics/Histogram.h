#pragma once

#include <vector>
#include <ostream>


namespace VirtualRobot
{

    /**
     * @brief Histogram for one-dimensional float data.
     */
    class Histogram
    {
    public:
        
        /// No initialization constructor.
        Histogram();
        
        /// Construct with the given data. 
        /// Minimum and maximum are derived automatically.
        Histogram(const std::vector<float>& data, std::size_t numBins = 128);
        
        /// Construct with the given data and given limits.
        Histogram(const std::vector<float>& data, float min, float max, std::size_t numBins = 128);
        
        
        /// Set the number of bins and set them to 0.
        void resetBins(std::size_t numBins);
        
        /// Insert the given value into the histogram.
        void insert(float value);
        /// Inserts the given values into the histogram.
        void insert(const std::vector<float>& value);
        
        /// Set the limits.
        void setMinMax(float min, float max);
        /// Set the limits from the given data.
        void setMinMax(const std::vector<float>& data);
        
        
        /// Get the bins.
        const std::vector<std::size_t>& getBins() const;
        
        /// Get the number of bins.
        std::size_t getNumberOfBins() const;
        
        ///  Transfer the given value to its corresponding bin index.
        std::size_t valueToIndex(float value) const;
        /// Get the value at the center of the bin with the given index.
        float indexToValue(std::size_t index) const;
        
        /// Get the minimum, i.e. the lower limit.
        float getMin() const;
        /// Get the maximum, i.e. the upper limit.
        float getMax() const;
        
        /// Return the index of the bin containing the fewest data points.
        std::size_t getMinBinIndex() const;
        /// Returns the corresponding value of the bin containing the fewest data points.
        float getMinBinValue() const;
        
        /// Return the index of the bin containing the most data points.
        std::size_t getMaxBinIndex() const;
        /// Returns the corresponding value of the bin containing the most data points.
        float getMaxBinValue() const;
        
        
        /**
         * @brief Applies a median filter to the histogram bins.
         *
         * The size specifies the number of neighours (in each direction)
         * considered.
         * That is, if k = size, 2k+1 values are considered for each point 
         * (k neighbours in each direction).
         *
         * @param size the size of the neighbourhood (in each direction)
         */
        void applyMedianFilter(std::size_t size = 2);
        
        
        /// Streams a CVS-like description of the histogram containing the bin data.
        friend std::ostream& operator<<(std::ostream& os, const Histogram& histo);
        
        
    private:
        
        /// The minimum mapped value.
        float min;
        /// The maximum mapped value.
        float max;
        
        /// The bins.
        std::vector<std::size_t> bins;
        
    };

}
