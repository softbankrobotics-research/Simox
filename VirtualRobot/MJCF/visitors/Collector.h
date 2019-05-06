#pragma once

#include "../elements.h"


namespace mjcf
{
    
    /**
     * @brief A visitor that collects all elements of the specified type.
     */
    template <class ElementT>
    class Collector : public Visitor
    {
    public:
        
        static std::vector<ElementT> collect(Document& document, AnyElement root);
        
        Collector(Document& document) : Visitor(document) {}
        
        // Visitor interface
        virtual bool visitEnter(const AnyElement& element) override
        {
            if (element.is<ElementT>())
            {
                collected.push_back(element.as<ElementT>());
            }
            return true;
        }
        
        std::vector<ElementT>& getCollected() { return collected; }
        const std::vector<ElementT>& getCollected() const  { return collected; }
        
        
    private:
        
        std::vector<ElementT> collected;
        
    };
    
    template <class ElementT>
    std::vector<ElementT> Collector<ElementT>::collect(Document& document, AnyElement root)
    {
        mjcf::Collector<ElementT> collector(document);
        root.accept(collector);
        return collector.getCollected();
    }
}

