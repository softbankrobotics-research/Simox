#include "equality.h"

#include "../Document.h"


using namespace mjcf;


const std::string EqualityDefaults::tag = "equality";
const std::string EqualityConnect::tag = "connect";
const std::string EqualityWeld::tag = "weld";
const std::string EqualityJoint::tag = "joint";
const std::string EqualityTendon::tag = "tendon";
const std::string EqualityDistance::tag = "distance";
const std::string EqualitySection::tag  = "equality";


Eigen::Vector7f EqualityWeld::relposeDefault()
{
    Eigen::Vector7f v;
    v << 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f;
    return v;
}


Eigen::Vector5f EqualityJoint::polycoefDefault()
{
    Eigen::Vector5f v;
    v << 0, 1, 0, 0, 0;
    return v;
}

Eigen::Vector5f EqualityTendon::polycoefDefault()
{
    Eigen::Vector5f v;
    v << 0, 1, 0, 0, 0;
    return v;
}

EqualityWeld EqualitySection::addWeld(const std::string& name, const std::string& body1, const std::string& body2)
{
    EqualityWeld weld = addChild<EqualityWeld>();

    weld.name = name;
    weld.body1 = body1;
    weld.body2 = body2;

    return weld;
}
