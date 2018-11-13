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
        
        
        void setLengthScaling(float value);
        void setFloatCompPrecision(float value);
        void setDummyMass(float value);
        
        
        /// Set the name of the Mujoco model.
        void setModelName(const std::string& name);
        
        /**
         * @brief Set the class attribute of all new applicable elements 
         * (after calling this method). Pass an empty string to disable class attributes.
         * @param excludeBody If true (default), the class will not be set on
         *  new body elements and its children (inertial, joint, ...). 
         *  They should be the childclass attribute of the robot root body.
         */
        void setNewElementClass(const std::string& className, bool excludeBody = true);
        
        
        // Top level elements
        Element* compiler()  { return section("compiler");  }
        Element* option()    { return section("option");    }
        Element* size()      { return section("size");      }
        Element* visual()    { return section("visual");    }
        Element* statistic() { return section("statistic"); }
        Element* default_()  { return section("default");   }
        Element* asset()     { return section("asset");     }
        Element* worldbody() { return section("worldbody"); }
        Element* contact()   { return section("contact");   }
        Element* equality()  { return section("equality");  }
        Element* tendon()    { return section("tendon");    }
        Element* actuator()  { return section("actuator");  }
        Element* sensor()    { return section("sensor");    }
        Element* keyframe()  { return section("keyframe");  }

        
        /// Adds a body element to the worldbody with the robot's name as name and childclass.
        Element* addRobotRootBodyElement(const std::string& robotName);
        /// Get the (least recently added) robot root body.
        Element* robotRootBody() const;
        
        /// Add a new defaults class with a class name.
        Element* addDefaultsClass(const std::string& className);

        /// Add a skybox texture asset (only one allowed).
        Element* addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2);        
        
        
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
        Element* addMeshElement(const std::string& name, const std::string& file,
                                const std::string& className = "");

        /// Add a conact/exclude element between the given bodies.
        Element* addContactExclude(const Element& body1, const Element& body2);
        /// Add a conact/exclude element between the given bodies.
        Element* addContactExclude(const std::string& body1Name, const std::string& body2Name);
        
        Element* addMotorElement(const std::string& jointName, const std::string& className = "");
        

        
        /// Set the pos and quat attribute of a body.
        void setBodyPose(Element* body, const Eigen::Matrix4f& pose);
        /// Set the axis attribute of a joint.
        void setJointAxis(Element* joint, const Eigen::Vector3f& axis);

        
        
    private:
        
        using ElementType = std::pair<std::string, std::string>;
        
        static const std::set<ElementType> ELEM_NAMES_WITH_ATTR_CLASS;
        
        static bool allowsClassAttr(const ElementType& type);
        
        
    private:
        
        /// Add a new element with a name (tag) to a parent.
        /// @param className If not empty, set the class attribute of the new element.
        /// @param first If true, will be inserted as first, otherweise at end (default)
        Element* addNewElement(Element* parent, const std::string& elemName, 
                               const std::string& className = "", bool first = false);
        
        /// Gets the top-level element (child of root element) with a name. 
        /// If it does not exist, it is created.
        Element* section(const std::string& name);
        
        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        /// Precision when comparing floats (e.g. with zero).
        float floatCompPrecision = 1e-6f;
        /// Mass used for dummy inertial elements.
        float dummyMass = 0.0001f;
        
        /// The "mujoco" root element.
        Element* root;
        
        /// The wrapper element of the robot.
        Element* robotRootBody_;
        
        std::string newElementClass = "";
        bool newElementClassExcludeBody = true;
        
        
        friend class ContactExcludeVisitor;
        
    };

    using DocumentPtr = std::unique_ptr<Document>;
 

    
    

    
}
}
