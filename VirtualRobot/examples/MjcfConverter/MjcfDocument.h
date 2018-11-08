#pragma once

#include <memory>
#include <tinyxml2.h>

#include <Eigen/Eigen>

#include <VirtualRobot/Robot.h>


namespace VirtualRobot
{
namespace mjcf
{

    using Element = tinyxml2::XMLElement;

    class Document : public tinyxml2::XMLDocument
    {
        
    public:
        
        Document();
        
        void setModelName(const std::string& name);
        
        
        Element* compiler()  { return topLevelElement("compiler");  }
        Element* option()    { return topLevelElement("option");    }
        Element* size()      { return topLevelElement("size");      }
        Element* visual()    { return topLevelElement("visual");    }
        Element* statistic() { return topLevelElement("statistic"); }
        Element* default_()  { return topLevelElement("default");   }
        Element* asset()     { return topLevelElement("asset");     }
        Element* worldbody() { return topLevelElement("worldbody"); }
        Element* contact()   { return topLevelElement("contact");   }
        Element* equality()  { return topLevelElement("equality");  }
        Element* tendon()    { return topLevelElement("tendon");    }
        Element* actuator()  { return topLevelElement("actuator");  }
        Element* sensor()    { return topLevelElement("sensor");    }
        Element* keyframe()  { return topLevelElement("keyframe");  }
        
        
        Element* addNewElement(Element* parent, const std::string& elemName);
        
        Element* addBodyElement(Element* parent, RobotNodePtr node);
        Element* addGeomElement(Element* body, const std::string& meshName);

        
        Element* addMeshElement(const std::string& name, const std::string& file);
        
        
        
    private:

        /// Gets the top-level element (child of root element) with the given
        /// name. If it does not exist, it is created.
        Element* topLevelElement(const std::string& name);
        
        Element* addInertialElement(Element* body, RobotNodePtr node);
        Element* addJointElement(Element* body, RobotNodePtr node);
        
        
        std::string toAttr(bool b);
        template <int dim>
        std::string toAttr(const Eigen::Matrix<float, dim, 1>& v);
        std::string toAttr(const Eigen::Quaternionf& v);
        
        
        float floatCompPrecision = 1e-6f;
        
        Eigen::IOFormat iofVector {5, 0, "", " ", "", "", "", ""};
        
        

        
        /// The "mujoco" root element.
        Element* root;
        
    };
    
    using DocumentPtr = std::unique_ptr<Document>;
 
    
    template <int dim>
    std::string Document::toAttr(const Eigen::Matrix<float, dim, 1>& v)
    {
        std::stringstream ss;
        ss << v.format(iofVector);
        return ss.str();
    }
    
}
}
