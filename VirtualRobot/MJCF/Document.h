#pragma once

#include <iostream>
#include <memory>
#include <set>

#include <Eigen/Eigen>

#include <VirtualRobot/Util/xml/tinyxml2.h>

#include "elements.h"


namespace mjcf
{

    /**
     * @brief A MJCF (Mujoco XML) document.
     */
    class Document
    {
        
    public:

        /// Constructor.
        Document();
        
        /// Set the precision for float comparison.
        float getFloatCompPrecision() const;
        /// Set the precision for float comparison (used e.g. when comparing
        /// to zero or identity).
        void setFloatCompPrecision(float value);
        /// Get the mass of dummy inertial elements.
        float getDummyMass() const;
        /// Set the mass of dummy inertial elements.
        void setDummyMass(float value);
        
        /// Load from an MJCF file.
        void loadFile(const std::string& fileName);
        /// Save to an MJCF file.
        void saveFile(const std::string& fileName);
        
        /// Make a deep copy of source to this.
        void deepCopyFrom(const Document& source);
        
        void print(std::ostream& os = std::cout) const;

        std::string getModelName() const;
        /// Set the name of the Mujoco model.
        void setModelName(const std::string& name);

        
        // Section elements (children of top-level 'mujoco' element).
        CompilerSection  compiler()  { return section<CompilerSection>();   }
        OptionSection    option()    { return section<OptionSection>();     }
        SizeSection      size()      { return section<SizeSection>();       }
        VisualSection    visual()    { return section<VisualSection>();     }
        StatisticSection statistic() { return section<StatisticSection>();  }
        DefaultSection   default_()  { return section<DefaultSection>();    }
        AssetSection     asset()     { return section<AssetSection>();      }
        Worldbody        worldbody() { return section<Worldbody>();         }
        ContactSection   contact()   { return section<ContactSection>();    }
        EqualitySection  equality()  { return section<EqualitySection>();   }
        TendonSection    tendon()    { return section<TendonSection>();     }
        ActuatorSection  actuator()  { return section<ActuatorSection>();   }
        SensorSection    sensor()    { return section<SensorSection>();     } 
        KeyframeSection  keyframe()  { return section<KeyframeSection>();   }

        Include addInclude(const std::string& filename);
        
        /**
         * @brief Set the class attribute of all new applicable elements
         * (after calling this method). Pass an empty string to disable class attributes.
         * @param excludeBody If true (default), the class will not be set on
         *  new body elements and its children (inertial, joint, ...).
         *  They should be the childclass attribute of the robot root body.
         */
        void setNewElementClass(const std::string& className, bool excludeBody = true);
        
        
        template <class Derived>
        friend class Element;

        
    private:

        // METHODS

        /**
         * @brief Add a new element to a parent.
         * @param className If not empty, set the class attribute of the new element.
         * @param first If true, will be inserted as first, otherweise at end (default)
         */
        template <class ElementD, class ParentD>
        ElementD createElement(Element<ParentD> parent, const std::string& className = "", bool front = false);
        
        /// Return the first child element of parent with the given element name.
        /// If it does not exist, create it.
        template <class ElementD, class ParentD>
        ElementD getOrCreateElement(Element<ParentD>& parent);

        /// Gets the section element (child of root element). If it does not exist, it is created.
        template <class SectionD>
        SectionD section();

        /// Prints the document to os.
        friend std::ostream& operator<<(std::ostream& os, const Document& rhs);
        
        
    private:
        
        // ATTRIBUTES
        
        /// The document.
        tinyxml2::XMLDocument doc;
        
        /// The "mujoco" root element.
        std::unique_ptr<MujocoRoot> root;

        
        /// Precision when comparing floats (e.g. with zero).
        float floatCompPrecision = 1e-6f;
        /// Mass used for dummy inertial elements.
        float dummyMass = 0.0001f;
        
        /// The class added to new elements, if applicable.
        std::string newElementClass = "";
        /// Indicate whether body elements and their children shall be
        /// exluded from setting the class attribute.
        bool newElementClassExcludeBody = true;

        
    };
    
    using DocumentPtr = std::unique_ptr<Document>;
    

    
#include "elements/has_member.hpp"
    
    define_has_member(class_);
    
    template <class ElementD, class ParentD>
    ElementD Document::createElement(Element<ParentD> parent, const std::string& className, bool front)
    {
        ElementD element(this, doc.NewElement(ElementD::Derived::tag.c_str()));

        auto applyNewElementClass = [this]()
        {
            bool isSet = !newElementClass.empty();
            bool hasClass = has_member(ElementD, class_);
            bool excludeBecauseBody = std::is_same<ParentD, Body>() && newElementClassExcludeBody;
                    // body itself does not have a class attribute
            
            bool inDefaultClass = std::is_same<ParentD, DefaultClass>();
            
            return isSet                     // must not be empty
                    && hasClass              // must have class attribute
                    && !excludeBecauseBody   // not excluded because of body exclusion
                    && !inDefaultClass;      // not part of default class
        };
                
        
        if (!className.empty())
        {
            element.setAttribute("class", className);
        }
        else if (applyNewElementClass())
        {
            element.setAttribute("class", newElementClass);
        }

        if (front)
        {
            parent.insertFirstChild(element);
        }
        else
        {
            parent.insertEndChild(element);
        }
        return element;
    }

// cleanup macros by has_member.hpp
#undef define_has_member
#undef has_member
    
    
    template <class ElementD, class ParentD>
    ElementD Document::getOrCreateElement(Element<ParentD>& parent)
    {
        ElementD element = parent.template firstChild<ElementD>();
        
        if (!element)
        {
            element = createElement<ElementD>(parent);
        }
        
        return element;
    }
    
    template<class SectionT>
    SectionT Document::section()
    {
        return getOrCreateElement<SectionT>(*root);
    }
    
    // Implementation of Element::createNewElement, which depends on the 
    // definition of Document.
    template <class D>
    template <class ParentD, class ElementD>
    ElementD Element<D>::createElement(Element<ParentD> parent, const std::string& className)
    {
        return _document->createElement<ElementD, ParentD>(parent, className);
    }
    
    template <class D>
    void Element<D>::insertComment(const std::string& text, bool front)
    {
        tinyxml2::XMLComment* comment = _document->doc.NewComment(text.c_str());
        if (front)
        {
            _element->InsertFirstChild(comment);
        }
        else
        {
            _element->InsertEndChild(comment);
        }
    }
    
}
