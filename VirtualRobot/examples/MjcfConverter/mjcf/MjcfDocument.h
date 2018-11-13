#pragma once

#include <memory>

#include <Eigen/Eigen>

#include <VirtualRobot/Tools/tinyxml2.h>

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
        
        // Top level elements
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
        
        
        /// Set the name of the Mujoco model.
        void setModelName(const std::string& name);
        
        /// Add a skybox texture asset (only one allowed).
        Element* addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2);
        
        /// Add a new element with a name (tag) to a parent.
        /// @param first if true, will be inserted as first, otherweise at end (default)
        Element* addNewElement(Element* parent, const std::string& elemName, bool first = false);
        
        
        /// Add a body element to a parent from a RobotNode.
        /// Adds inertial and joint element, if necessary.
        Element* addBodyElement(Element* parent, RobotNodePtr node);
        
        /// Add an inertial element to a body from a RobotNode.
        Element* addInertialElement(Element* body, RobotNodePtr node);
        /// Add a dummy inertial element with small mass and identity inertia matrix.
        Element* addDummyInertial(Element* body);
        /// Add a joint element to a body from a RobotNode.
        Element* addJointElement(Element* body, RobotNodePtr node);
        
        /// Add a geom to a body, referencing a mesh.
        Element* addGeomElement(Element* body, const std::string& meshName);
        /// Add a mesh asset with a name and a file.
        Element* addMeshElement(const std::string& name, const std::string& file);
        
        Element* addMotorElement(const std::string& jointName);
        
        /// Set the pos and quat attribute of a body.
        void setBodyPose(Element* body, const Eigen::Matrix4f& pose);
        /// Set the axis attribute of a joint.
        void setJointAxis(Element* joint, const Eigen::Vector3f& axis);
        
        /// Add a conact/exclude element between the given bodies.
        Element* addContactExclude(const Element& body1, const Element& body2);
        /// Add a conact/exclude element between the given bodies.
        Element* addContactExclude(const std::string& body1Name, const std::string& body2Name);
        
        
    private:
        
        /// Gets the top-level element (child of root element) with a name. 
        /// If it does not exist, it is created.
        Element* topLevelElement(const std::string& name);

        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        /// Precision when comparing floats (e.g. with zero).
        float floatCompPrecision = 1e-6f;
        /// Mass used for dummy inertial elements.
        float dummyMass = 0.0001f;
        
        /// The "mujoco" root element.
        Element* root;
        
        
        friend class ContactExcludeVisitor;
        
    };
    
    using DocumentPtr = std::unique_ptr<Document>;
 

    
    

    
}
}
