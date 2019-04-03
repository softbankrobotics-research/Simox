#pragma once

#include <VirtualRobot/Util/xml/tinyxml2.h>


namespace mjcf
{
    class AnyElement;
    class Document;
    class Visitor;

namespace detail
{

    class VisitorAdapter : public tinyxml2::XMLVisitor
    {
    public:
        VisitorAdapter(Visitor& owner);
        virtual bool visitEnter(const tinyxml2::XMLElement& element);
        virtual bool visitExit(const tinyxml2::XMLElement& element);
        
    private:
        Visitor& owner;
    };

}

    class Visitor
    {
    public:
        
        Visitor(Document& document);
        virtual ~Visitor() = default;
        
        virtual bool visitEnter(const AnyElement&) { return true; }
        virtual bool visitExit(const AnyElement&) { return true; }
        
        
        detail::VisitorAdapter* adapter();
        const detail::VisitorAdapter* adapter() const;
        
        
    private:
        
        friend class detail::VisitorAdapter;
        const AnyElement cast(const tinyxml2::XMLElement& element);
        
        
        Document& document;
        
        detail::VisitorAdapter _adapter { *this };
        
    };

}
