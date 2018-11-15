#pragma once

#include <memory>

#include <Eigen/Eigen>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Util/xml/tinyxml2.h>

#include "utils.h"


namespace VirtualRobot
{
namespace mjcf
{

    using XMLElement = tinyxml2::XMLElement;

    /**
     * @brief A MJCF (Mujoco XML) document.
     */
    class Document : public tinyxml2::XMLDocument
    {
        
    public:
        
        /// Constructor.
        Document();
        
        
        /// Set the precision for float comparison (used e.g. when comparing 
        /// to zero or identity).
        void setFloatCompPrecision(float value);
        /// Set the mass set in dummy inertial elements.
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
        
        
        // Section elements (children of top-level 'mujoco' element).
        XMLElement* compiler()  { return section("compiler");  }
        XMLElement* option()    { return section("option");    }
        XMLElement* size()      { return section("size");      }
        XMLElement* visual()    { return section("visual");    }
        XMLElement* statistic() { return section("statistic"); }
        XMLElement* default_()  { return section("default");   }
        XMLElement* asset()     { return section("asset");     }
        XMLElement* worldbody() { return section("worldbody"); }
        XMLElement* contact()   { return section("contact");   }
        XMLElement* equality()  { return section("equality");  }
        XMLElement* tendon()    { return section("tendon");    }
        XMLElement* actuator()  { return section("actuator");  }
        XMLElement* sensor()    { return section("sensor");    }
        XMLElement* keyframe()  { return section("keyframe");  }

        
        /// Adds a body element to the worldbody with the robot's name as name and childclass.
        XMLElement* addRobotRootBodyElement(const std::string& robotName);
        /// Get the (least recently added) robot root body.
        XMLElement* robotRootBody() const;
        
        /// Add a new defaults class with a class name.
        XMLElement* addDefaultsClass(const std::string& className);

        template <typename AttrT>
        XMLElement* addDefaultAttr(XMLElement* defaultsClass, const std::string& elementName, 
                                      const std::string& attrName, const AttrT& attrValue);
        
        
        /// Add a skybox texture asset (only one allowed).
        XMLElement* addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2);        
        
        
        /// Add a body element to a parent from a RobotNode.
        /// Adds inertial and joint element, if necessary.
        XMLElement* addBodyElement(XMLElement* parent, RobotNodePtr node);
        
        /// Add an inertial element to a body from a RobotNode.
        XMLElement* addInertialElement(XMLElement* body, RobotNodePtr node);
        /// Add a dummy inertial element with small mass and identity inertia matrix.
        XMLElement* addDummyInertial(XMLElement* body);
        /// Add a joint element to a body from a RobotNode.
        XMLElement* addJointElement(XMLElement* body, RobotNodePtr node);
        
        /// Add a geom to a body, referencing a mesh.
        XMLElement* addGeomElement(XMLElement* body, const std::string& meshName);
        /// Add a mesh asset with a name and a file.
        XMLElement* addMeshElement(const std::string& name, const std::string& file,
                                const std::string& className = "");

        /// Add a conact/exclude element between the given bodies.
        XMLElement* addContactExclude(const XMLElement& body1, const XMLElement& body2);
        /// Add a conact/exclude element between the given bodies.
        XMLElement* addContactExclude(const std::string& body1Name, const std::string& body2Name);
        
        XMLElement* addActuatorMotorElement(const std::string& jointName);
        XMLElement* addActuatorPositionElement(const std::string& jointName, float kp = 0);
        XMLElement* addActuatorPositionElement(
                const std::string& jointName, bool ctrlLimited, const Eigen::Vector2f& ctrlRange, float kp = 0);
        XMLElement* addActuatorVelocityElement(const std::string& jointName, float kv = 0);
        

        
        /// Set the pos and quat attribute of an element.
        void setElementPose(XMLElement* elem, const Eigen::Matrix4f& pose);
        void setElementPos(XMLElement* elem, Eigen::Vector3f pos);
        void setElementQuat(XMLElement* elem, const Eigen::Quaternionf& quat);
        
        /// Set the axis attribute of a joint.
        void setJointAxis(XMLElement* joint, const Eigen::Vector3f& axis);

        
        
    private:
        
        /// (ParentTag, ElementTag)
        using ElementType = std::pair<std::string, std::string>;
        
        /// Element types allowing a class attribute.
        static const std::set<ElementType> ELEM_NAMES_WITH_ATTR_CLASS;
        
        /// Return true if the given ElementType allows for a class attribute.
        static bool allowsClassAttr(const ElementType& type);
        
        
    private:
        
        
        
        /// Add a new element with a name (tag) to a parent.
        /// @param className If not empty, set the class attribute of the new element.
        /// @param first If true, will be inserted as first, otherweise at end (default)
        XMLElement* addNewElement(XMLElement* parent, const std::string& elemName, 
                                  const std::string& className = "", bool first = false);
        
        /// Return the first child element of parent with the given element name.
        /// If it does not exist, create it.
        XMLElement* getOrCreateElement(XMLElement* parent, const std::string& elemName);
        
        /// Gets the section element (child of root element) with a name.
        /// If it does not exist, it is created.
        XMLElement* section(const std::string& name);
        
        XMLElement* addActuatorShortcut(
                const std::string& type, const std::string& name, const std::string& jointName, 
                const std::string& paramName = "", float paramValue = 0);
        
        
        /// Precision when comparing floats (e.g. with zero).
        float floatCompPrecision = 1e-6f;
        /// Mass used for dummy inertial elements.
        float dummyMass = 0.0001f;
        
        
        /// The "mujoco" root element.
        XMLElement* root;
        
        /// The wrapper element of the robot.
        XMLElement* robotRootBody_;
        
        
        /// The class added to new elements, if applicable.
        std::string newElementClass = "";
        /// Indicate whether body elements and their children shall be 
        /// exluded from setting the class attribute.
        bool newElementClassExcludeBody = true;
        
        
    };
    
    template<typename AttrT>
    XMLElement* Document::addDefaultAttr(
            XMLElement* defaultsClass, const std::string& elementName, 
            const std::string& attrName, const AttrT& attrValue)
    {
        assertElementIs(defaultsClass, "default");
        
        XMLElement* element = getOrCreateElement(defaultsClass, elementName);
        element->SetAttribute(attrName.c_str(), attrValue);
        
        return element;
    }

    using DocumentPtr = std::unique_ptr<Document>;
    
}
}
