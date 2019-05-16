#include "Attribute.h"


namespace mjcf
{

Eigen::Vector5f mjcf_solimpDefault()
{
    Eigen::Vector5f v;
    v << 0.9f, 0.95f, 0.001f, 0.5, 2;
    return v;
}

}
