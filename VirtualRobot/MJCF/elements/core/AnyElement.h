#pragma once

#include "Element.h"


namespace mjcf
{

    /**
     * @brief An element that can hold any type.
     */
    class AnyElement : public Element<AnyElement>
    {
    public:
        
        static const std::string tag;
        mjcf_ElementDerivedConstructors(AnyElement)
        
        /// Construct from any other element type.
        template <class OtherDerived>
        AnyElement(const Element<OtherDerived>& other) : 
            AnyElement(other.template reinterpret<AnyElement>())
        {}
        
        
        /// Indicate whether *this is an element of type OtherDerived.
        template <typename OtherDerived>
        bool is() const { return isElement<OtherDerived>(); }
        
        /// Get *this as element of type OtherDerived.
        /// Should only be called if `is()` is true.
        template <typename OtherDerived>
        OtherDerived as() { return { document(), element() }; }
        
        template <typename OtherDerived>
        const OtherDerived as() const { return reinterpret<OtherDerived>(); }
        
        
        /// Return the actual tag.
        std::string actualTag() const { return element()->Value(); }
        
    };

}
