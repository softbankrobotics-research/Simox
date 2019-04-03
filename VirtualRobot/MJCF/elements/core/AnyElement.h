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
        
        
        /// Indicate whether *this is an element of type OtherDerived.
        template <typename OtherDerived>
        bool is() const { return isElement<OtherDerived>(); }
        
        /// Get *this as element of type OtherDerived.
        /// Should only be called if `is()` is true.
        template <typename OtherDerived>
        OtherDerived as() { return OtherDerived( document(), element() ); }
        
        template <typename OtherDerived>
        const OtherDerived as() const { return OtherDerived( document(), element() ); }
        
    };

}
