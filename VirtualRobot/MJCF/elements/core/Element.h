#pragma once

#include <VirtualRobot/Util/xml/tinyxml2.h>

#include "const_aware_ptr.hpp"
#include "exceptions.h"
#include "mjcf_utils.h"
#include "Visitor.h"


namespace mjcf
{

    // Forward declaration.
    class Document;
    
    template <class _Derived>
    class Element
    {
    public:
        
        using Derived = _Derived;
        
        // Make all Element<*> friends of each other.
        template <class OtherDerived> friend class Element;
        
        
    public:

        // CONSTRUCTORS
        
        /// Empty constructor.
        Element() {}
        
        /**
         * @brief Construct and element with given document and underlying (tinyxml2) element.
         * @throws error::InvalidElementTag if `element`'s tag does not match `Derived::tag`.
         */
        Element(Document* document, tinyxml2::XMLElement* element);
        
    protected:
        
        /**
         * Only allow inheriting classes to construct with a const pointers,
         * since const-ness is effectively cast away on construction, and offering
         * it publicly would create a false sense of security at this point.
         */
        Element(const Document* document, const tinyxml2::XMLElement* element);
        
        
    public:
        
        // TYPE CHECKING
        
        /// Indicate whether this is an element of the given type.
        template <class OtherT>
        static bool isSame() { return std::is_same<Derived, typename OtherT::Derived>(); }

        
        /// Indicate whether the underlying element's tag is OtherT's tag.
        /// This should be true at virtually all times.
        template <class OtherT>
        bool isElement() const { return OtherT::tag.empty() || std::string(_element->Value()) == OtherT::tag; }
        
        
        // ATTRIBUTES
        
        /// Indicate whether this element has an attribute with the given name.
        bool isAttributeSet(const std::string& attrName) const;

        /// Get attribute value.
        template <typename AttrT = std::string>
        AttrT getAttribute(const std::string& name) const;
        /// Get the attribute value, or the default value if it does not exist.
        template <typename AttrT = std::string>
        AttrT getAttribute(const std::string& name, const AttrT& defaultValue) const;
        /// Set the attribute.
        template <typename AttrT>
        void setAttribute(const std::string& name, const AttrT& value);
        
        std::vector<std::string> getSetAttributeNames() const;
        
        
        // CHILDREN
        
        /// Indicate whether this element has any children.
        bool hasChildren() const;
        
        /// Indicate whether there is a child of the given type.
        template <class OtherDerived>
        bool hasChild() const;
        /// Indicate whether there is a child for which the given predicate is true.
        template <class OtherDerived>
        bool hasChild(std::function<bool(OtherDerived)> predicate) const;
        /// Indicate whether there is a child with the given attribute value.
        template <class OtherDerived>
        bool hasChild(const std::string& attrName, const std::string& attrValue) const;
        
        /// Get the first child of the given type.
        template <class OtherDerived>
        OtherDerived firstChild() const;
        /// Get the first child of the given type for which the given predicate is true.
        template <class OtherDerived>
        OtherDerived firstChild(std::function<bool(OtherDerived)> predicate) const;
        /// Get the first child of the given type with the given attribute value.
        template <class OtherDerived>
        OtherDerived firstChild(const std::string& attrName, const std::string& attrValue) const;
        
        
        // SIBLINGS
        
        /// Get the next sibling element of the given type.
        template <class OtherDerived>
        OtherDerived nextSiblingElement() const;
        /// Get the next sibling element of the given type for which the predicate is true.
        template <class OtherDerived>
        OtherDerived nextSiblingElement(std::function<bool(OtherDerived)> predicate) const;
        /// Get the next sibling element of the given type with the given attribute value.
        template <class OtherDerived>
        OtherDerived nextSiblingElement(const std::string& attrName, const std::string& attrValue) const;

        
        // PARENT
        
        /// Get the parent element.
        template <class ParentDerived>
        ParentDerived parent();
        template <class ParentDerived>
        const ParentDerived parent() const;
        
        
        // INSERTION & DELETION

        /// Add a new child of this with of given type.
        template <class OtherDerived>
        OtherDerived addChild(const std::string& className = "");
        
        /// Get the first child of type OtherDerived. If there is none, add one.
        template <class OtherDerived>
        OtherDerived getOrCreateChild();
        
        
        template <class OtherDerived>
        void insertChild(Element<OtherDerived>& element, bool front = false);
        
        /// Insert an element at the front.
        template <class OtherDerived>
        void insertFirstChild(Element<OtherDerived>& element);
        /// Insert an element at the end.
        template <class OtherDerived>
        void insertEndChild(Element<OtherDerived>& element);

        /// Delete given child from this.
        template <class OtherDerived>
        void deleteChild(Element<OtherDerived>& element);
        
        
        /// Insert a comment as child of *this.
        void insertComment(const std::string& text, bool front = false);
        
        
        // VISITING
        
        /// Accept a hierarchical visit of the elements in *this.
        /// @see tinyxml2::XMLNode::Accept()
        bool accept(Visitor& visitor);
        
        
        // CLONING AND TRANSFORMATION
        
        /// Create a deep clone of this element.
        Derived deepClone() const;
        
        /**
         * @brief Transform this element to another type.
         * 
         * Change the underlying element's tag and return the element as the 
         * new type. Using `*this` afterwards can result in unexpected 
         * behaviour (and should in general be avoided), so use this method
         * with caution!
         */
        template <class OtherDerived>
        OtherDerived transform();
        
        
        /**
         * @brief Reinterpret this element as the given element type.
         * This is only safe from and to `AnyElement`.
         */
        // (Ideally, this functionality would only be accessible from AnyElement.
        //  However, giving AnyElement access to other types' document() and element() seems not
        //  to work without making document() and element() public (friend declaration does not
        //  seem to work). This seems to only effect the const methods.
        template <class OtherDerived>
        OtherDerived reinterpret() const { return Element<OtherDerived>(_document, _element); }
        
        
        // OPERATORS
        
        /// Indicate whether this contains a valid element.
        operator bool() const { return bool(_element); }
        
        template <typename Derived>
        friend std::ostream& operator<<(std::ostream& os, const Element<Derived>& element);
        
        
    protected:
        
        Document* document() { return _document; }
        const Document* document() const  { return _document; }
        
        tinyxml2::XMLElement* element() { return _element; }
        const tinyxml2::XMLElement* element() const { return _element; }

        
    private:

        /// Use document to create a new element of type ElementD with given parent.
        template <class ParentD, class ElementD>
        ElementD createElement(Element<ParentD> parent, const std::string& className = "");
        
        void assertElemValueEqualsTag();
        
        template <class OtherDerived>
        std::function<bool(OtherDerived)> predicateWithAttrib(
                const std::string& attrName, const std::string& attrValue) const;
        
        Document* _document = nullptr;
        detail::const_aware_ptr<tinyxml2::XMLElement> _element = nullptr;
        
    };
    
    
    template <class D>
    Element<D>::Element(Document* document, tinyxml2::XMLElement* element) : 
        _document(document), _element(element)
    {
        assertElemValueEqualsTag();
    }
    
    template <class D>
    Element<D>::Element(const Document* document, const tinyxml2::XMLElement* element) :
        Element(const_cast<Document*>(document), const_cast<tinyxml2::XMLElement*>(element))
    {}
    
    template <class D>
    bool Element<D>::isAttributeSet(const std::string& attrName) const
    {
        return _element->Attribute(attrName.c_str()) != nullptr;
    }
    
    template <class D>
    template <typename AttrT>    
    AttrT Element<D>::getAttribute(const std::string& name) const
    {
        const char* attr = _element->Attribute(name.c_str());
        if (!attr)
        {
            throw AttribNotSetError(name);
        }
        AttrT value;
        fromAttr(attr, value);
        return value;
    }
    
    template <class D>
    template <typename AttrT>
    AttrT Element<D>::getAttribute(const std::string& name, const AttrT& defaultValue) const
    {
        return isAttributeSet(name) ? getAttribute<AttrT>(name) : defaultValue;
    }
    
    template <class D>
    template<typename AttrT>
    void Element<D>::setAttribute(const std::string& name, const AttrT& value)
    {
        std::string attrStr = toAttr(value);
        _element->SetAttribute(name.c_str(), attrStr.c_str());
    }

    template <class D>
    std::vector<std::string> Element<D>::getSetAttributeNames() const
    {
        std::vector<std::string> names;
        for (auto* attr = _element->FirstAttribute(); attr; attr = attr->Next())
        {
            names.emplace_back(attr->Name());
        }
        return names;
    }
    
    template <class D>
    bool Element<D>::hasChildren() const
    {
        return !_element->NoChildren();
    }
    
    template <class D>
    template <class OtherD>
    bool Element<D>::hasChild() const
    {
        return firstChild<OtherD>();
    }
    
    template <class D>
    template <class OtherD>
    bool Element<D>::hasChild(std::function<bool(OtherD)> predicate) const
    {
        return firstChild<OtherD>(predicate);
    }
    
    template <class D>
    template <class OtherD>
    bool Element<D>::hasChild(const std::string& attrName, const std::string& attrValue) const
    {
        return firstChild<OtherD>(attrName, attrValue);
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::firstChild() const
    {
        return Element<OtherD>(_document, /*may be null*/ _element->FirstChildElement(OtherD::tag.c_str()));
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::firstChild(std::function<bool(OtherD)> predicate) const
    {
        for (OtherD child = firstChild<OtherD>(); child;
             child = child.template nextSiblingElement<OtherD>())
        {
            if (predicate(child))
            {
                return child;
            }
        }
        return {};
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::firstChild(const std::string& attrName, const std::string& attrValue) const
    {
        return firstChild<OtherD>(predicateWithAttrib<OtherD>(attrName, attrValue));
    }


    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::nextSiblingElement() const
    {
        return Element<OtherD>(_document, _element->NextSiblingElement(OtherD::tag.c_str()));
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::nextSiblingElement(std::function<bool (OtherD)> predicate) const
    {
        for (OtherD sibling = nextSiblingElement<OtherD>(); sibling;
             sibling = sibling.template nextSiblingElement<OtherD>())
        {
            if (predicate(sibling))
            {
                return sibling;
            }
        }
        return {};
    }
    
    template <class D>
    template<class OtherD>
    OtherD Element<D>::nextSiblingElement(const std::string& attrName, const std::string& attrValue) const
    {
        return nextSiblingElement<OtherD>(predicateWithAttrib<OtherD>(attrName, attrValue));
    }
    
    
    template <class D>
    template <class ParentDerived>
    ParentDerived Element<D>::parent()
    {
        return { _document, _element->Parent()->ToElement() };
    }
    
    template <class D>
    template <class ParentDerived>
    const ParentDerived Element<D>::parent() const
    {
        return Element<ParentDerived>(_document, _element->Parent()->ToElement());
    }
    
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::addChild(const std::string& className)
    {
        return createElement<D, OtherD>(*this, className);
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::getOrCreateChild()
    {
        OtherD child = firstChild<OtherD>();
        if (!child)
        {
            child = addChild<OtherD>();
        }
        return child;
    }
    
    
    template <class D>
    template <class OtherD>
    void Element<D>::insertChild(Element<OtherD>& element, bool front)
    {
        front ? insertFirstChild(element) : insertEndChild(element());
    }
    
    template <class D>
    template <class OtherD>
    void Element<D>::insertFirstChild(Element<OtherD>& element)
    {
        _element->InsertFirstChild(element._element);
    }
    
    template <class D>
    template <class OtherD>
    void Element<D>::insertEndChild(Element<OtherD>& element)
    {
        _element->InsertEndChild(element._element);
    }
    
    template <class D>
    template <class OtherD>
    void Element<D>::deleteChild(Element<OtherD>& element)
    {
        _element->DeleteChild(element._element);
    }
    
    template <class D>
    bool Element<D>::accept(Visitor& visitor)
    {
        return _element->Accept(visitor.adapter());
    }
    
    template <class D>
    auto Element<D>::deepClone() const -> Derived
    {
        return { _document, _element->DeepClone(nullptr)->ToElement() };
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::transform()
    {
        _element->SetValue(OtherD::tag.c_str());
        return { _document, _element };
    }
    
    template <class D>
    void Element<D>::assertElemValueEqualsTag()
    {
        if (_element && !isElement<D>())
        {
            throw error::InvalidElementTag(Derived::tag, _element->Value());
        }
    }
    
    template <class D>
    template <class OtherDerived>
    std::function<bool(OtherDerived)> Element<D>::predicateWithAttrib(
            const std::string& attrName, const std::string& attrValue) const
    {
        return [&attrName, &attrValue](OtherDerived e)
        { 
            return e.isAttributeSet(attrName) && e.getAttribute(attrName) == attrValue;
        };
    }
    
    template <class D>
    std::ostream& operator<<(std::ostream& os, const Element<D>& element)
    {
        os << "MJCF Element '" << D::tag << "' (-> " << element._element << ")";
        return os;
    }
    

#define mjcf_ElementDerivedConstructorsBase(Base, Derived)  \
    Derived() : Base<Derived>() {}                   \
    Derived(Document* document, ::tinyxml2::XMLElement* elem) : Base<Derived>(document, elem) {} \
    Derived(const Base<Derived>& base) : Base<Derived>(base) {}      \
    Derived(const Derived& other) : Base<Derived>(other) {} \
    Derived(Derived&& other) : Base<Derived>(other) {}      \
    Derived& operator=(const Base<Derived>& base)           \
    { Base<Derived>::operator=(base);  return *this; }      \
    Derived& operator=(const Derived& other)                \
    { Base<Derived>::operator=(other); return *this; }      \
    Derived& operator=(Derived&& other)                     \
    { Base<Derived>::operator=(other); return *this; }
    
    
#define mjcf_ElementDerivedConstructors(Derived)            \
    mjcf_ElementDerivedConstructorsBase(Element, Derived)
    

}
