#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot
{
namespace mjcf 
{

    class MergedBodySet
    {
    public:
        
        MergedBodySet();
        MergedBodySet(const std::string& bodyName);
        
        void addBody(const std::string& bodyName);
        bool containsBody(const std::string& bodyName) const;
        
        const std::string& getMergedBodyName() const;
        
        
    private:
        
        void updateMergedBodyName();
        
        std::string mergedBodyName;
        std::vector<std::string> originalBodyNames;
        
    };
    

    class MasslessBodySanitizer
    {
    public:
        
        MasslessBodySanitizer(DocumentPtr& document, RobotPtr& robot);
        
        void sanitize();
        
        const std::vector<MergedBodySet>& getMergedBodySets() const;

        const std::string& getMergedBodyName(const std::string& originalBodyName);
        
        MergedBodySet& getMergedBodySetWith(const std::string& bodyName);
        
        
    private:
        
        void sanitizeRecursion(mjcf::XMLElement* body);
        void sanitizeLeafBody(mjcf::XMLElement* body);
        
        void mergeBodies(XMLElement* body, XMLElement* childBody, Eigen::Matrix4f& accChildPose);
        
        void updateChildPos(XMLElement* elem, const Eigen::Matrix4f& accChildPose);
        void updateChildQuat(XMLElement* elem, const Eigen::Matrix3f& accChildOri);
        void updateChildAxis(XMLElement* elem, const Eigen::Matrix3f& accChildOri, 
                             const char* attrName = "axis");
        
        
        
        const std::string t = "| ";
        
        
        DocumentPtr& document;
        RobotPtr& robot;
        
        std::vector<MergedBodySet> mergedBodySets;
        
    };
    
} 
}
