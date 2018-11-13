#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot
{
namespace mjcf 
{

    class MergedBodySet
    {
    public:
        
        void addBody(const std::string& bodyName);
        bool containsBody(const std::string& bodyName) const;
        
        const std::string& getMergedBodyName() const;
        
        
    private:
        
        void updateMergedBodyName();
        
        std::string mergedBodyName;
        std::set<std::string> originalBodyNames;
        
    };


    class MjcfMasslessBodySanitizer
    {
    public:
        
        MjcfMasslessBodySanitizer(DocumentPtr& document, RobotPtr& robot);
        
        void sanitize();
        
        const std::vector<MergedBodySet>& getMergedBodySets() const;

        
    private:
        
        void sanitizeRecursion(mjcf::Element* body);
        void sanitizeLeafBody(mjcf::Element* body);
        
        MergedBodySet& getMergedBodySetWith(const std::string& bodyName);
        
        
        DocumentPtr& document;
        RobotPtr& robot;
        
        std::vector<MergedBodySet> mergedBodySets;
        
    };
    
} 
}
