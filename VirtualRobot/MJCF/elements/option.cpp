#include "option.h"

#include "../Document.h"


using namespace mjcf;


const std::string OptionFlag::tag    = "flag";
const std::string OptionSection::tag = "option";


OptionFlag OptionSection::flag()
{
    return getOrCreateChild<OptionFlag>();
}
