#pragma once

#include "core/Attribute.h"


namespace mjcf
{


struct DefaultClass : public Element<DefaultClass>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(DefaultClass)
    
    mjcf_ClassAttribute(DefaultClass);
    
    template <class ElementD>
    ElementD getElement();
    
    template <class ElementD, typename AttrT>
    ElementD setElementAttr(const std::string& attrName, const AttrT& attrValue);
};


struct DefaultSection : public Element<DefaultSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(DefaultSection)
    
    /// Indicate whether there is a default class with the given name.
    bool hasClass(const std::string& className);
    
    /// Get the default class for given class name. If it does not exist, add it.
    DefaultClass getClass(const std::string& className);
    
    /// Add a new default class with given class name.
    DefaultClass addClass(const std::string& className);
};



template<class ElementD>
ElementD DefaultClass::getElement()
{
    return getOrCreateChild<ElementD>();
}

template<class ElementD, typename AttrT>
ElementD DefaultClass::setElementAttr(const std::string& attrName, const AttrT& attrValue)
{
    ElementD element = getElement<ElementD>();
    element.setAttribute(attrName, attrValue);
    return element;
}


}
