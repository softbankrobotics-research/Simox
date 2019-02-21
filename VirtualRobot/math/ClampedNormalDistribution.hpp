#pragma once

#include <random>


namespace VirtualRobot
{

    /**
     * @class A normal distribution with lower and upper bounds (min and max).
     *
     * The normal distribution's mean is not limited to the clamping bounds.
     * 
     * When a new number is generated, random numbers are generated according
     * to the normal distribution until a number between min and max is
     * generated. If the number of attempts exceeds a threshold, the last
     * generated number drawn uniformly between min and max.
     */
    template <typename RealType>
    class ClampedNormalDistribution
    {
    public:
    
        /// Construct an unclamped standard normal distribution.
        ClampedNormalDistribution()
        {}
    
        /// Construct a clamped normal distribution centered between min and max
        /// with t * stddev being equal to the distance from center to bound.
        ClampedNormalDistribution(RealType min, RealType max, RealType t = 3) :
            _distrib(.5 * (min + max), .5 / t * (max - min)), _min(min), _max(max)
        {}
    
        /// Construct a clamped normal distribution with given parameters.
        ClampedNormalDistribution(RealType mean, RealType stddev, RealType min, RealType max) :
            _distrib(mean, stddev), _min(min), _max(max)
        {}
    
        /// Construct an unclamped normal distribution with given mean and standard deviation.
        static ClampedNormalDistribution Unclamped(RealType mean = 0, RealType stddev = 1)
        {
            return ClampedNormalDistribution(mean, stddev, std::numeric_limits<RealType>::min(),
                                             std::numeric_limits<RealType>::max());
        }
    
        /// Get the mean of the normal distribution.
        RealType mean() const { return _distrib.mean(); }
        /// Set the mean of the normal distribution.
        void setMean(RealType value) { this->_distrib = { value, _distrib.stddev() }; }
    
        /// Get the standard deviation of the normal distribution.
        RealType stddev() const {  return _distrib.stddev(); }
        /// Set the standard deviation of the normal distribution.
        void setStddev(RealType value) { this->_distrib = { _distrib.mean(), value }; }
    
        /// Get the lower clipping bound.
        RealType min() const { return _min; }
        /// Set the lower clipping bound.
        void setMin(RealType value) { this->_min = value; }
    
        /// Get the upper clipping bound.
        RealType max() const { return _max; }
        /// Set the lower clipping bound.
        void setMax(RealType value) { this->_max = value; }
    
        
        /// Get the maximal number of attempts.
        int maxAttempts() const { return _maxAttempts; }
        /// Set the maximal number of attempts.
        void setMaxAttempts(int value) { this->_maxAttempts = value; }
    
        
        /// Generate a new random number between min and max.
        template <class Generator>
        RealType operator()(Generator& gen)
        {
            assert(_min < _max);
            int attempts = 0;
            RealType result;
            do
            {
                result = _distrib(gen);
            }
            while ((result < _min || _max < result) && attempts++ < _maxAttempts);
    
            if (attempts >= _maxAttempts)
            {
                result = uniform(gen);
            }
            return result;
        }
        
        /// Generate a new random number drawn uniformly between min and max.
        template <class Generator>
        RealType uniform(Generator& gen) const
        {
            std::uniform_real_distribution<RealType> uniformDistrib(_min, _max);
            return uniformDistrib(gen);
        }
    
    
        /// Equality operator (member-wise equality).
        bool operator==(const ClampedNormalDistribution& rhs) const
        {
            return this == &rhs ||
                   (this->mean() == rhs.mean()
                    && this->stddev() == rhs.stddev()
                    && this->_min == rhs._min
                    && this->_max == rhs._max
                    && this->_maxAttempts == rhs._maxAttempts);
        }
    
        
    private:
    
        /// The normal distribution.
        std::normal_distribution<RealType> _distrib {0, 1};
    
        /// The lower bound.
        RealType _min = std::numeric_limits<RealType>::min();
        /// The upper bound.
        RealType _max = std::numeric_limits<RealType>::max();
    
        /// The maximal number of times it is attempted to generate a value 
        /// inside the bounds before uniformly drawing the result.
        int _maxAttempts = 100;
    
    };

    /// Stream a human-readable description of rhs to os.
    template <typename RealT>
    std::ostream& operator<< (std::ostream& os, const ClampedNormalDistribution<RealT>& rhs)
    {
        // for some reason, the first string must be wrapped in string constructor
        os << std::string("<ClampedNormalDistribution mean=") << rhs.mean() << " stddev=" 
           << rhs.stddev() << " [min, max]=[" << rhs.min() << ", " << rhs.max() << "] >";
        return os;
    }

    
    /// A clamped normal distribution of floats.
    using ClampedNormalDistributionf = ClampedNormalDistribution<float>;
    /// A clamped normal distribution of doubles.
    using ClampedNormalDistributiond = ClampedNormalDistribution<double>;
    
}
