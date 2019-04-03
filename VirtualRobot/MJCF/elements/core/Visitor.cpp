#include "Visitor.h"

#include "AnyElement.h"

namespace mjcf
{

Visitor::Adapter::Adapter(Visitor& owner) : 
    owner(owner)
{}

bool Visitor::Adapter::VisitEnter(const tinyxml2::XMLElement& element, const tinyxml2::XMLAttribute*)
{
    return owner.visitEnter(owner.cast(element));
}

bool Visitor::Adapter::VisitExit(const tinyxml2::XMLElement& element)
{
    return owner.visitExit(owner.cast(element));
}

Visitor::Visitor(Document& document) : document(document)
{}

Visitor::Adapter* Visitor::adapter()
{
    return &_adapter;
}

const Visitor::Adapter* Visitor::adapter() const
{
    return &_adapter;
}

const AnyElement Visitor::cast(const tinyxml2::XMLElement& element)
{
    // The constructor of Element requires a non-const element pointer.
    // As AnyElement is passed as const& to the user-provided visit method,
    // this const cast should be sufficiently safe.
    return { &document, const_cast<tinyxml2::XMLElement*>(&element) };
}

}
