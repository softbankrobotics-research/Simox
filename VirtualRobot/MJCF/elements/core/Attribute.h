#pragma once

#include <VirtualRobot.h>
#include <VirtualRobot/math/Helpers.h>

#include "Element.h"


namespace mjcf
{

    /**
     * @class Base class for an attribute of an Element<Derived>.
     * Stores a pointer to the owning element.
     */
    template <class Derived>
    class AttributeBase
    {
    public:
        
        AttributeBase() : _owner(nullptr) 
        {}
        
        /// Constructor.
        /// @param owner the owning element (Element instance of which this is a member).
        AttributeBase(Element<Derived>& owner) : _owner(&owner) 
        {}

        
        /* _owner must always point to the element this was constructed with.
         * We therefore delete copy and move constructors (otherwise, the owner
         * will be unavailable. In addition, _owner is constant, so it is not
         * changed by copy or move operators. */
        
        /// Deleted copy constructor.
        AttributeBase(const AttributeBase<Derived>& other) = delete;
        /// Deleted move constructor.
        AttributeBase(AttributeBase<Derived>&& other) = delete;
        
        
    protected:
        /// Get the owning element.
        Element<Derived>& owner() { return *_owner; }
        const Element<Derived>& owner() const { return *_owner; }
        
    private:
        /// The owning element (Element instance of which this is a member).
        /// It must not change after construction, and thus is const-qualified.
        Element<Derived>* const _owner;
        
    };

    
    
    /**
     * @class An attribute of an Element<Derived> of type AttrT.
     */
    template <class Derived, typename AttrT>
    class Attribute : public AttributeBase<Derived>
    {
    public:
        
        /// Constructor.
        Attribute(Element<Derived>& element, const std::string& name)
            : AttributeBase<Derived>(element), name(name)
        {}
        
        /// Get the attribute value (or the default value if the attribute does not exist).
        virtual AttrT get() const = 0;
        /// Set the attribute value.
        void set(const AttrT& value)
        {
            VR_ASSERT(this->owner());
            this->owner().template setAttribute<AttrT>(name, value);
        }
        
        /// Conversion operator to AttrT for reading.
        operator AttrT()
        {
            return this->get();
        }
        
        /// Copy assignment operator for writing an attribute value.
        Attribute& operator=(const AttrT& value)
        {
            this->set(value);
            return *this;
        }
        
        /// Move assignment operator for writing an attribute value.
        Attribute& operator=(AttrT&& value)
        {
            this->set(value);
            return *this;
        }
        
        /// Indicate whether the attribute is required (implies no default).
        virtual bool isRequired() const = 0;
        /// Indicate whether the attribute is optional or has a default.
        bool isOptional() const { return !isRequired(); }
        /// Indicate whether the attribute has a default (implies optional).
        virtual bool hasDefault() const = 0;
        
        /// Indicate whether the attribute is set.
        bool isSet() const { return this->owner().isAttributeSet(this->name); }
        
        
    protected:
        
        std::string name;   ///< The attribute name.
        
    };

    template <class Derived, typename AttrT>
    std::ostream& operator<<(std::ostream& os, const Attribute<Derived, AttrT>& rhs)
    {
        os << rhs.get();
        return os;
    }
    
    
    /**
     * @class An attribute of an Element<Derived> of type AttrT
     * that is required or optional and has no default value.
     */
    template <class Derived, typename AttrT>
    class AttributeNoDef : public Attribute<Derived, AttrT>
    {
        using Base = Attribute<Derived, AttrT>;
    public:
        
        /// Construct an required or optional attribute without a default.
        AttributeNoDef(Element<Derived>& element, const std::string& name, bool required)
            : Base(element, name), required(required)
        {}
        
        AttrT get() const override
        {
            VR_ASSERT(this->owner());
            return this->owner().template getAttribute<AttrT>(this->name);
        }
        
        bool isRequired() const override { return  required; }
        bool hasDefault() const override { return  false; }
        
        using Base::operator=;
        
        
    private:
        
        bool required;      ///< Whether the attribute is optional.
    };
    
    
    
    /**
     * @class An attribute of an Element<Derived> of type AttrT
     * that is optional and has a default value.
     */
    template <class Derived, typename AttrT>
    class AttributeDef : public Attribute<Derived, AttrT>
    {
        using Base = Attribute<Derived, AttrT>;
        
    public:
        
        /// Construct an optional attribute with a default.
        AttributeDef(Element<Derived>& element, const std::string& name, const AttrT& defaultValue)
            : Base(element, name), defaultValue(defaultValue)
        {}
        
        AttrT get() const override
        {
            VR_ASSERT(this->owner());
            return this->owner().template getAttribute<AttrT>(this->name, this->defaultValue);
        }
        
        bool isRequired() const override { return  false; }
        bool hasDefault() const override { return  true; }
        
        using Base::operator=;
        
        
    private:
        
        AttrT defaultValue; ///< The default value.
        
    };
    

// Macro for a requried or optional Attribute in an Element<Derived> of type AttrT
// with potentially different memberName and attrName
#define mjcf_AttributeFullNoDef(Derived, AttrT, memberName, attrName, required) \
    AttributeNoDef<Derived, AttrT> memberName {*this, attrName, required}
    
// Macro for an optional Attribute in an Element<Derived> of type AttrT
// with potentially different memberName and attrName.
#define mjcf_AttributeFullDef(Derived, AttrT, memberName, attrName, defaultValue) \
    AttributeDef<Derived, AttrT> memberName {*this, attrName, defaultValue}

    
// A requried Attribute in an Element<Derived> of type AttrT.
#define mjcf_AttributeReq(Derived, AttrT, name) \
    mjcf_AttributeFullNoDef(Derived, AttrT, name, #name, true)

#define mjcf_AttributeOpt(Derived, AttrT, name) \
    mjcf_AttributeFullNoDef(Derived, AttrT, name, #name, false)
    
// An optional Attribute in an Element<Derived> of type AttrT.
#define mjcf_AttributeDef(Derived, AttrT, name, defaultValue) \
    mjcf_AttributeFullDef(Derived, AttrT, name, #name, defaultValue)

    
// Attributes with a name that is a keyword (e.g. "class").
#define mjcf_AttributeKwReq(Derived, AttrT, name) \
    mjcf_AttributeFullNoDef(Derived, AttrT, name ## _, #name, true)

#define mjcf_AttributeKwOpt(Derived, AttrT, name) \
    mjcf_AttributeFullNoDef(Derived, AttrT, name ## _, #name, false)
    
#define mjcf_AttributeKwDef(Derived, AttrT, name, defaultValue) \
    mjcf_AttributeFullDef(Derived, AttrT, name ## _, #name, defaultValue)


// bool shorthands
#define mjcf_BoolAttributeReq(Derived, name) \
    mjcf_AttributeReq(Derived, bool, name)
#define mjcf_BoolAttributeOpt(Derived, name) \
    mjcf_AttributeOpt(Derived, bool, name)
#define mjcf_BoolAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, bool, name, defaultValue)
    
// int shorthands
#define mjcf_IntAttributeReq(Derived, name) \
    mjcf_AttributeReq(Derived, int, name)
#define mjcf_IntAttributeOpt(Derived, name) \
    mjcf_AttributeOpt(Derived, int, name)
#define mjcf_IntAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, int, name, defaultValue)
    
    
// float shorthands
#define mjcf_FloatAttributeReq(Derived, name) \
    mjcf_AttributeReq(Derived, float, name)
#define mjcf_FloatAttributeOpt(Derived, name) \
    mjcf_AttributeOpt(Derived, float, name)
#define mjcf_FloatAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, float, name, defaultValue)
    
// string shorthands
#define mjcf_StringAttributeReq(Derived, memberName) \
    mjcf_AttributeReq(Derived, std::string, memberName)
#define mjcf_StringAttributeOpt(Derived, memberName) \
    mjcf_AttributeOpt(Derived, std::string, memberName)
#define mjcf_StringAttributeDef(Derived, memberName, defaultValue) \
    mjcf_AttributeDef(Derived, std::string, memberName, defaultValue)
    
#define mjcf_StringAttributeDefEmpty(Derived, memberName, defaultValue) \
    mjcf_StringAttributeDef(Derived, std::string, memberName, "")


// vector shorthands
    
#define mjcf_Vector2fAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, Eigen::Vector2f, name, defaultValue)
    
#define mjcf_Vector3fAttributeReq(Derived, name) \
    mjcf_AttributeReq(Derived, Eigen::Vector3f, name)
#define mjcf_Vector3fAttributeOpt(Derived, name) \
    mjcf_AttributeOpt(Derived, Eigen::Vector3f, name)
#define mjcf_Vector3fAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, Eigen::Vector3f, name, defaultValue)
    
    
// name
#define mjcf_NameAttribute(Derived) \
    mjcf_StringAttributeOpt(Derived, name)
    
// class
#define mjcf_ClassAttribute(Derived) \
    mjcf_AttributeKwReq(Derived, std::string, class)

// group
#define mjcf_GroupAttribute(Derived) \
    mjcf_IntAttributeDef(Derived, group, 0)


// RGB
#define mjcf_RgbAttributeDef(Derived, name, defaultValue) \
    mjcf_Vector3fAttributeDef(Derived, name, defaultValue)
    
#define mjcf_RgbaAttributeDef(Derived, name, defaultValue) \
    mjcf_AttributeDef(Derived, Eigen::Vector4f, name, defaultValue)

    
// pos
#define mjcf_PosAttribute(Derived) \
    mjcf_Vector3fAttributeDef(Derived, pos, Eigen::Vector3f::Zero())

// quat
#define mjcf_QuatAttribute(Derived) \
    mjcf_AttributeDef(Derived, Eigen::Quaternionf, quat, Eigen::Quaternionf::Identity())

// pose = pos + quat
#define mjcf_PoseAttributes(Derived) \
    Eigen::Matrix4f getPose() const                                     \
    { return math::Helpers::Pose(pos.get(), quat.get()); }              \
    void setPose(const Eigen::Matrix4f& p)                              \
    {   pos = math::Helpers::Position(p);                               \
        quat = Eigen::Quaternionf(math::Helpers::Orientation(p));   }   \
    mjcf_PosAttribute(Derived);                                         \
    mjcf_QuatAttribute(Derived)


Eigen::Vector5f mjcf_solimpDefault();

// solref
#define mjcf_SolrefAttribute(Derived) \
    mjcf_AttributeDef(Derived, Eigen::Vector2f, solref, Eigen::Vector2f(0.02f, 1.f))
// solimp
#define mjcf_SolimpAttribute(Derived) \
    mjcf_AttributeDef(Derived, Eigen::Vector5f, solimp, mjcf_solimpDefault())
    
    
} 
