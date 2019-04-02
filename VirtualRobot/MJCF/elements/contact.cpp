#include "contact.h"

#include "../Document.h"

using namespace mjcf;


const std::string ContactPair::tag    = "pair";
const std::string ContactExclude::tag = "exclude";
const std::string ContactSection::tag = "contact";


Eigen::Vector5f ContactPair::condimDefault() 
{
    Eigen::Vector5f v;
    v << 1, 1, 0.005f, 0.0001f, 0.0001f;
    return v;
}

ContactPair ContactSection::addPair(const Geom& geom1, const Geom& geom2)
{
    return addPair(geom1.name.get(), geom2.name.get());
}

ContactPair ContactSection::addPair(const std::string& geom1Name, const std::string& geom2Name)
{
    ContactPair pair = addChild<ContactPair>();

    pair.geom1 = geom1Name;
    pair.geom2 = geom2Name;

    return pair;
}


ContactExclude ContactSection::addExclude(const Body& body1, const Body& body2)
{
    return addExclude(body1.name.get(), body2.name.get());
}

ContactExclude ContactSection::addExclude(const std::string& body1Name, const std::string& body2Name)
{
    ContactExclude exclude = addChild<ContactExclude>();

    exclude.body1 = body1Name;
    exclude.body2 = body2Name;

    return exclude;
}
