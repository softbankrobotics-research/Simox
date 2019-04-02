#pragma once

#include <VirtualRobot.h>
#include <VirtualRobot/Util/xml/tinyxml2.h>

#include "exceptions.h"
#include "mjcf_utils.h"


namespace mjcf
{

    // Forward declaration.
    class Document;
    
    template <class _Derived>
    class Element
    {
    public:
        
        using Derived = _Derived;
        
        template <class OtherDerived>
        friend class Element;
        
        
    public:

        // CONSTRUCTORS
        
        /// Constructor.
        Element(Document* document, tinyxml2::XMLElement* elem);
        
        
        /// Indicate whether this is an element of the given type.
        template <class OtherT>
        bool isElement() const { return std::is_same<Derived, typename OtherT::Derived>(); }

        
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

        
        // INSERTION & DELETION

        /// Add a new child of this with of given type.
        template <class OtherDerived>
        OtherDerived addChild(const std::string& className = "");
        
        /// Get the first child of type OtherDerived. If there is none, add one.
        template <class OtherDerived>
        OtherDerived getOrCreateChild();
        
        
        /// Insert an element at the front.
        template <class OtherDerived>
        void insertFirstChild(const Element<OtherDerived>& element);
        /// Insert an element at the end.
        template <class OtherDerived>
        void insertEndChild(const Element<OtherDerived>& element);

        /// Delete given child from this.
        template <class OtherDerived>
        void deleteChild(const Element<OtherDerived>& element);
        
        
        // CLONING AND TRANSFORMATION
        
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
        
        
        // OPERATORS
        
        /// Indicate whether this contains a valid element.
        operator bool() const { return elem != nullptr; }
        
        template <typename Derived>
        friend std::ostream& operator<<(std::ostream& os, const Element<Derived>& element);
        
        
    protected:
        
        Document& document() { return *_document; }
        const Document& document() const  { return *_document; }
        
        tinyxml2::XMLElement* element() { return elem; }
        const tinyxml2::XMLElement* element() const { return elem; }
        
        
    private:

        /// Use document to create a new element of type ElementD with given parent.
        template <class ParentD, class ElementD>
        ElementD createElement(Element<ParentD> parent, const std::string& className = "");
        
        void assertElemValueEqualsTag();
        
        template <class OtherDerived>
        std::function<bool(OtherDerived)> predicateWithAttrib(
                const std::string& attrName, const std::string& attrValue) const;
        
        Document* _document;
        tinyxml2::XMLElement* elem;
        
    };
    
    template <class D>
    Element<D>::Element(Document* document, tinyxml2::XMLElement* elem) : 
        _document(document), elem(elem)
    {
        assertElemValueEqualsTag();
    }
    
    template <class D>
    bool Element<D>::isAttributeSet(const std::string& attrName) const
    {
        return elem->Attribute(attrName.c_str()) != nullptr;
    }
    
    template <class D>
    template <typename AttrT>    
    AttrT Element<D>::getAttribute(const std::string& name) const
    {
        const char* attr = elem->Attribute(name.c_str());
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
        elem->SetAttribute(name.c_str(), attrStr.c_str());
    }

    template <class D>
    std::vector<std::string> Element<D>::getSetAttributeNames() const
    {
        std::vector<std::string> names;
        for (auto* attr = elem->FirstAttribute(); attr; attr = attr->Next())
        {
            names.emplace_back(attr->Name());
        }
        return names;
    }
    
    template <class D>
    bool Element<D>::hasChildren() const
    {
        return !elem->NoChildren();
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
        return OtherD(_document, /*may be null*/ elem->FirstChildElement(OtherD::tag.c_str()));
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
        return OtherD(_document, nullptr);
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
        return OtherD(_document, elem->NextSiblingElement(OtherD::tag.c_str()));
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
        return OtherD(_document, nullptr);
    }
    
    template <class D>
    template<class OtherD>
    OtherD Element<D>::nextSiblingElement(const std::string& attrName, const std::string& attrValue) const
    {
        return nextSiblingElement<OtherD>(predicateWithAttrib<OtherD>(attrName, attrValue));
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
    void Element<D>::insertFirstChild(const Element<OtherD>& element)
    {
        elem->InsertFirstChild(element.elem);
    }
    
    template <class D>
    template <class OtherD>
    void Element<D>::insertEndChild(const Element<OtherD>& element)
    {
        elem->InsertEndChild(element.elem);
    }
    
    template <class D>
    template <class OtherD>
    void Element<D>::deleteChild(const Element<OtherD>& element)
    {
        elem->DeleteChild(element.elem);
    }
    
    template <class D>
    auto Element<D>::deepClone() const -> Derived
    {
        return { _document, elem->DeepClone(nullptr)->ToElement() };
    }
    
    template <class D>
    template <class OtherD>
    OtherD Element<D>::transform()
    {
        elem->SetValue(OtherD::tag.c_str());
        return { _document, elem };
    }
    
    template <class D>
    void Element<D>::assertElemValueEqualsTag()
    {
        if (elem && !Derived::Tag.empty())
        {
            VR_ASSERT_MESSAGE(elem->Value() == Derived::tag, 
                              "Element tag '" + std::string(elem->Value()) + "' must be '" + Derived::tag + "'");
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
        os << "MJCF Element '" << D::tag << "' (-> " << element.elem << ")";
        return os;
    }
    

#define mjcf_ElementDerivedConstructorsBase(Base, Derived)  \
    Derived() : Base<Derived>(nullptr, nullptr) {}                   \
    Derived(Document* document, tinyxml2::XMLElement* elem) : Base<Derived>(document, elem) {} \
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
