#include "mjcf_utils.h"

#include <algorithm>
#include <map>


namespace mjcf
{

    std::size_t commonPrefixLength(const std::string& a, const std::string& b)
    {
        const std::string* smaller = &a;
        const std::string* bigger = &b;
        if (b.size() < a.size())
        {
            std::swap(smaller, bigger);
        }

        auto mismatch = std::mismatch(smaller->begin(), smaller->end(),
                                      bigger->begin()).first;
        return std::size_t(std::distance(smaller->begin(), mismatch));
    }
    
    template <>
    std::string toAttr<bool>(const bool& b)
    {
        static const std::string strings[] = { "false", "true" };
        return strings[int(b)];
    }
    
    void fromAttr(const std::string& valueStr, bool& value)
    {
        static const std::map<std::string, bool> map = {
            { "true", true },
            { "false", false },
            { "1", true },
            { "0", false },
        };
        value = map.at(valueStr);
    }

    Eigen::Vector2f strToVec2(const char* string)
    {
        Eigen::Vector2f v;
        sscanf(string, "%f %f", &v(0), &v(1));
        return v;
    }

    Eigen::Vector3f strToVec3(const char* string)
    {
        Eigen::Vector3f v;
        sscanf(string, "%f %f %f", &v(0), &v(1), &v(2));
        return v;
    }

    Eigen::Quaternionf strToQuat(const char* string)
    {
        Eigen::Quaternionf q;
        sscanf(string, "%f %f %f %f", &q.w(), &q.x(), &q.y(), &q.z());
        return q;
    }


    
    bool equals(const char* lhs, const char* rhs)
    {
        return strcmp(lhs, rhs) == 0;
    }
    
}

bool std::operator==(const char* lhs, const std::string& rhs)
{
    return mjcf::equals(lhs, rhs.c_str());
}

bool std::operator==(const std::string& lhs, const char* rhs)
{
    return mjcf::equals(lhs.c_str(), rhs);
}
