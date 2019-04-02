#pragma once

#include <VirtualRobot.h>
#include <VirtualRobot/MJCF/Document.h>


namespace VirtualRobot
{
namespace mujoco
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
        
        MasslessBodySanitizer(mjcf::Document& document, RobotPtr& robot);
        
        void sanitize();
        
        const std::vector<MergedBodySet>& getMergedBodySets() const;

        const std::string& getMergedBodyName(const std::string& originalBodyName);
        
        MergedBodySet& getMergedBodySetWith(const std::string& bodyName);
        
        
    private:
        
        void sanitizeRecursion(mjcf::Body body);
        void sanitizeLeafBody(mjcf::Body body);
        
        void mergeBodies(mjcf::Body body, mjcf::Body childBody, Eigen::Matrix4f& accChildPose);
        
        const std::string t = "| ";
        
        
        mjcf::Document& document;
        RobotPtr& robot;
        
        std::vector<MergedBodySet> mergedBodySets;
        
    };
    
} 
}
