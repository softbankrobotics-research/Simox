#include "Document.h"

#include "elements/exceptions.h"


using namespace mjcf;


Document::Document()
{
    // create root element
    tinyxml2::XMLElement* xml = doc.NewElement(MujocoRoot::tag.c_str());
    doc.InsertEndChild(xml);
    root.reset(new MujocoRoot(this, xml));
}

void Document::loadFile(const std::string& fileName)
{
    tinyxml2::XMLError error = doc.LoadFile(fileName.c_str());
    if (error != tinyxml2::XML_SUCCESS)
    {
        throw MjcfIOError(doc.ErrorStr());
    }
}

void Document::saveFile(const std::string& fileName)
{
    tinyxml2::XMLError error = doc.SaveFile(fileName.c_str());
    if (error != tinyxml2::XML_SUCCESS)
    {
        throw MjcfIOError(doc.ErrorStr());
    }
}

void Document::deepCopyFrom(const Document& source)
{
    source.doc.DeepCopy(&this->doc);
}



std::string Document::getModelName() const
{
    if (root->isAttributeSet("model"))
    {
        return root->getAttribute("model");
    }
    else
    {
        return "";
    }
}
void Document::setModelName(const std::string& name)
{
    root->setAttribute("model", name);
}

Include Document::addInclude(const std::string& filename)
{
    Include include = root->addChild<Include>();
    include.file = filename;
    return include;
}

void Document::setNewElementClass(const std::string& className, bool excludeBody)
{
    this->newElementClass = className;
    this->newElementClassExcludeBody = excludeBody;
    
    if (!className.empty() && !default_().hasChild<DefaultClass>("class", className))
    {
        default_().addClass(className);
    }
}


float Document::getFloatCompPrecision() const
{
    return floatCompPrecision;
}

void Document::setFloatCompPrecision(float value)
{
    floatCompPrecision = value;
}

float Document::getDummyMass() const
{
    return dummyMass;
}

void Document::setDummyMass(float value)
{
    dummyMass = value;
}

