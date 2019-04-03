#pragma once

#include <VirtualRobot/Util/xml/tinyxml2.h>


namespace mjcf
{
    class AnyElement;
    class Document;
    class Visitor;

    
    class Visitor
    {
    public:
        class Adapter : public tinyxml2::XMLVisitor
        {
        public:
            Adapter(Visitor& owner);
            
            virtual bool visitEnter(const tinyxml2::XMLElement& element);
            virtual bool visitExit(const tinyxml2::XMLElement& element);
            
        private:
            Visitor& owner;
        };
        friend class Adapter;
        
        
    public:
        
        Visitor(Document& document);
        virtual ~Visitor() = default;
        
        virtual bool visitEnter(const AnyElement&) { return true; }
        virtual bool visitExit(const AnyElement&) { return true; }
        
        
        Adapter* adapter();
        const Adapter* adapter() const;

        const AnyElement cast(const tinyxml2::XMLElement& element);
        
        Document& document;
        
        Adapter _adapter { *this };
        
    };

}
