#include "ConfigurationConstraint.h"
#include "CSpace.h"

namespace MotionPlanning
{

    ConfigurationConstraint::ConfigurationConstraint(unsigned int dimension)
    {
        this->dimension = dimension;
    }

    ConfigurationConstraint::~ConfigurationConstraint()
    {
    }

}
