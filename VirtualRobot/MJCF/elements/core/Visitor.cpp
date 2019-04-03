#include "Visitor.h"

#include "AnyElement.h"

namespace mjcf
{

namespace detail
{

VisitorAdapter::VisitorAdapter(Visitor& owner) : 
    owner(owner)
{}

bool VisitorAdapter::visitEnter(const tinyxml2::XMLElement& element)
{
    return owner.visitEnter(owner.cast(element));
}

bool VisitorAdapter::visitExit(const tinyxml2::XMLElement& element)
{
    return owner.visitExit(owner.cast(element));
}

}

Visitor::Visitor(Document& document) : document(document)
{}

detail::VisitorAdapter* Visitor::adapter()
{
    return &_adapter;
}

const detail::VisitorAdapter* Visitor::adapter() const
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
