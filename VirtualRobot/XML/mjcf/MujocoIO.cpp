#include "MujocoIO.h"
#include "utils.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>


using namespace VirtualRobot;
using namespace mjcf;
namespace tx = tinyxml2; 
namespace fs = boost::filesystem;



MujocoIO::MujocoIO(RobotPtr robot) : robot(robot) 
{
    THROW_VR_EXCEPTION_IF(!robot, "Given RobotPtr robot is null.");
}

void MujocoIO::saveMJCF(const std::string& filename, const std::string& basePath, 
                        const std::string& meshRelDir)
{
    THROW_VR_EXCEPTION_IF(filename.empty(), "Given filename is empty.");
    
    setPaths(filename, basePath, meshRelDir);
    
    document.reset(new mjcf::Document());
    document->setModelName(robot->getName());
    
    makeCompiler();
    
    makeDefaultsGroup();
    document->setNewElementClass(robot->getName(), true);
    
    addSkybox();
    
    if (withMocapBody)
    {
        std::cout << "Adding mocap body ..." << std::endl;
        addMocapBody();
    }
    
    std::cout << "Creating bodies structure ..." << std::endl;
    makeNodeBodies();
    
    std::cout << "Adding meshes and geoms ..." << std::endl;
    addNodeBodyMeshes();
    
    bool print = false;
    if (print)
    {
        std::cout << "===========================" << std::endl
                  << "Current model: "             << std::endl
                  << "--------------"              << std::endl;
        document->Print();
        std::cout << "===========================" << std::endl;
    }
    
    std::cout << "Merging massless bodies ..." << std::endl;
    masslessBodySanitizer.sanitize();
    
    std::cout << "Adding contact excludes ..." << std::endl;
    addContactExcludes();

    std::cout << "Adding actuators ..." << std::endl;
    addActuators();
    
    std::cout << "Scaling lengths by " << lengthScaling << " ..." << std::endl;
    scaleLengths(document->robotRootBody());
    
    std::cout << "Done." << std::endl;
    
    if (print)
    {
        std::cout << std::endl;
        std::cout << "===========================" << std::endl
                  << "Output file: "             << std::endl
                  << "------------"              << std::endl;
        document->Print();
        std::cout << "===========================" << std::endl;
    }
    
    assert(!outputFileName.empty());
    std::cout << "Writing to " << (outputDirectory / outputFileName) << std::endl;
    document->SaveFile((outputDirectory / outputFileName).c_str());
}

void MujocoIO::setPaths(const std::string& filename, const std::string& basePath, const std::string& meshRelDir)
{
    outputDirectory = basePath;
    outputFileName = filename;
    outputMeshRelDirectory = meshRelDir;
    
    ensureDirectoriesExist();
}

void MujocoIO::ensureDirectoriesExist()
{
    auto ensureDirExists = [](const fs::path& dir, const std::string& errMsgName)
    {
        if (!fs::is_directory(dir))
        {
            std::cout << "Creating directory: " << dir << std::endl;
            bool success = fs::create_directories(dir);
            THROW_VR_EXCEPTION_IF(!success, "Could not create " << errMsgName << ": " << dir);
        }
    };

    ensureDirExists(outputDirectory, "output directory");
    ensureDirExists(outputMeshDirectory(), "output mesh directory");
}

void MujocoIO::makeCompiler()
{
    document->compiler()->SetAttribute("angle", "radian");
    document->compiler()->SetAttribute("balanceinertia", "true");
}

void MujocoIO::makeDefaultsGroup()
{
    XMLElement* defaultsClass = document->addDefaultsClass(robot->getName());
    
    std::stringstream comment;
    comment << "Add default values for " << robot->getName() << " here.";
    defaultsClass->InsertFirstChild(document->NewComment(comment.str().c_str()));
    
    document->addDefaultAttr(defaultsClass, "joint", "frictionloss", 1);
    document->addDefaultAttr(defaultsClass, "joint", "damping", 0);
    document->addDefaultAttr(defaultsClass, "geom", "condim", 4);
    document->addDefaultAttr(defaultsClass, "position", "kp", 1);
    document->addDefaultAttr(defaultsClass, "velocity", "kv", 1);
}


void MujocoIO::addSkybox()
{
    document->addSkyboxTexture(Eigen::Vector3f(.8f, .9f, .95f), 
                               Eigen::Vector3f(.4f, .6f, .8f));
}


void MujocoIO::addMocapBody()
{
    std::string className = "mocap";
    float geomSize = 0.01f;
    
    std::string bodyName;
    {
        std::stringstream ss;
        ss << robot->getName() << "_Mocap";
        bodyName = ss.str();
    }
    
    // add defaults class
    XMLElement* defClass = document->addDefaultsClass(className);
    
    document->addDefaultAttr(defClass, "geom", "rgba", 
                             toAttr(Eigen::Vector4f(.9f, .5f, .5f, .5f)).c_str());
    
    document->addDefaultAttr(defClass, "equality", "solimp", 
                             toAttr(Eigen::Vector3f{.95f, .99f, .001f}).c_str());
    document->addDefaultAttr(defClass, "equality", "solref", 
                             toAttr(Eigen::Vector2f{ .02f, 1.f}).c_str());
    
    // add body
    XMLElement* mocap = document->addMocapBody(bodyName, geomSize);
    mocap->SetAttribute("childclass", className.c_str());
    
    // add equality weld constraint
    document->addEqualityWeld(bodyName, robot->getName(), bodyName, className);
}


void MujocoIO::makeNodeBodies()
{
    nodeBodies.clear();
    
    RobotNodePtr rootNode = robot->getRootNode();
    assert(rootNode);
    
    // add root
    XMLElement* robotRootBody = document->addRobotRootBodyElement(robot->getName());
    
    if (withMocapBody)
    {
        document->addDummyInertial(robotRootBody);
        XMLElement* joint = document->addFreeJointElement(robotRootBody);
        joint->SetAttribute("name", robot->getName().c_str());
    }
    
    XMLElement* root = document->addBodyElement(robotRootBody, rootNode);
    nodeBodies[rootNode->getName()] = root;
    assert(root);
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        addNodeBody(node);
    }
}

void MujocoIO::addNodeBodyMeshes()
{
    bool meshlabserverAviable = system("which meshlabserver > /dev/null 2>&1") == 0;
    bool notAvailableReported = false;
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        VisualizationNodePtr visualization = node->getVisualization(SceneObject::VisualizationType::Full);
        
        if (!visualization)
        {
            continue;
        }
        
        std::cout << t << "Node " << node->getName() << ":\t";
        
        fs::path srcMeshPath = visualization->getFilename();
        
        fs::path dstMeshFileName = srcMeshPath.filename();
        dstMeshFileName.replace_extension("stl");
        fs::path dstMeshPath = outputMeshDirectory() / dstMeshFileName;
        
        if (!fs::exists(dstMeshPath))
        {
            if (srcMeshPath.extension().string() != "stl")
            {
                std::cout << "Converting to .stl: " << srcMeshPath;
                
                if (!meshlabserverAviable)
                {
                    if (!notAvailableReported)
                    {
                        std::cout << std::endl 
                                  << "Command 'meshlabserver' not available, "
                                     "cannot convert meshes." << std::endl;
                        notAvailableReported = true;
                    }
                    continue;
                }
                
                // meshlabserver available
                std::stringstream convertCommand;
                convertCommand << "meshlabserver"
                               << " -i " << srcMeshPath 
                               << " -o " << dstMeshPath;
                
                // run command
                std::cout << "----------------------------------------------------------" << std::endl;
                int r = system(convertCommand.str().c_str());
                std::cout << "----------------------------------------------------------" << std::endl;
                if (r != 0)
                {
                    std::cout << "Command returned with error: " << r << "\n"
                              << "Command was: " << convertCommand.str() << std::endl;
                }
            }
            else
            {
                std::cout << "Copying: " << srcMeshPath << "\n"
                          << "     to: " << dstMeshPath;
                fs::copy_file(srcMeshPath, dstMeshPath);
            }
        }
        else
        {
            std::cout << "skipping (" << outputMeshRelDirectory / dstMeshFileName 
                      << " already exists)";
        }
        std::cout << std::endl;
        
        
        
        // add asset
        std::string meshName = node->getName();
        document->addMeshElement(meshName, fs::absolute(dstMeshPath).string());
        
        // add geom to body
        XMLElement* body = nodeBodies[node->getName()];
        document->addGeomElement(body, meshName);
    }
}



XMLElement* MujocoIO::addNodeBody(RobotNodePtr node)
{
    XMLElement* element = nodeBodies[node->getName()];
    if (element)
    {
        // break recursion
        return element;
    }
    
    XMLElement* parent = nodeBodies[node->getParent()->getName()];
    if (!parent)
    {
        parent = addNodeBody(robot->getRobotNode(node->getParent()->getName()));
    }
    
    element = document->addBodyElement(parent, node);
    nodeBodies[node->getName()] = element;

    return element;
}

struct ParentChildContactExcludeVisitor : public tinyxml2::XMLVisitor
{
    
    ParentChildContactExcludeVisitor(Document& document) : document(document) {}
    ~ParentChildContactExcludeVisitor() override = default;

    bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
    Document& document;  ///< The document.
    bool firstSkipped = false;  ///< Used to skip the root element.
};

bool ParentChildContactExcludeVisitor::VisitEnter(const tinyxml2::XMLElement& body, const tinyxml2::XMLAttribute*)
{
    if (!isElement(body, "body"))
    {
        return true;
    }
    
    if (!firstSkipped)
    {
        firstSkipped = true;
        return true;
    }
    
    const XMLElement* parent = body.Parent()->ToElement();
    assert(parent);
    
    document.addContactExclude(*parent, body);
    
    return true;
}


void MujocoIO::addContactExcludes()
{
    RobotNodePtr rootNode = robot->getRootNode();
    
    std::vector<std::pair<std::string, std::string>> excludePairs;
            
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        for (std::string& ignore : node->getPhysics().ignoreCollisions)
        {
            // I found an <IgnoreCollision> element referring to a non-existing node.
            // => check node existence here
            if (robot->hasRobotNode(ignore))
            {
                excludePairs.push_back({node->getName(), ignore});
            }
        }
    }
    
    // resolve body names and add exludes
    for (auto& excludePair : excludePairs)
    {
        std::string body1 = masslessBodySanitizer.getMergedBodyName(excludePair.first);
        std::string body2 = masslessBodySanitizer.getMergedBodyName(excludePair.second);
        document->addContactExclude(body1, body2);
    }
    
    ParentChildContactExcludeVisitor visitor(*document);
    document->robotRootBody()->Accept(&visitor);
}

void MujocoIO::addActuators()
{
    std::vector<const mjcf::XMLElement*> jointElements = getAllElements("joint");
    
    for (auto joint : jointElements)
    {
        std::string name = joint->Attribute("name");
        switch (actuatorType)
        {
            case ActuatorType::MOTOR:
                document->addActuatorMotorElement(name);
                break;
                
            case ActuatorType::POSITION:
            {
                XMLElement* act = document->addActuatorPositionElement(name);
                
                const char* limited = joint->Attribute("limited");
                if (limited)
                {
                    act->SetAttribute("ctrllimited", limited);
                    
                    const char* range = joint->Attribute("range");
                    if (range)
                    {
                        act->SetAttribute("ctrlrange", range);
                    }
                }
            }
                break;
                
            case ActuatorType::VELOCITY:
                document->addActuatorVelocityElement(name);
                break;
        }
    }
}

void MujocoIO::scaleLengths(XMLElement* elem)
{
    assert(elem);
    
    if (isElement(elem, "joint"))
    {
        if (isAttr(elem, "type", "slide") && hasAttr(elem, "range"))
        {
            std::cout << t << "Scaling range of slide joint " 
                      << elem->Attribute("name") << std::endl;
            
            Eigen::Vector2f range = strToVec2(elem->Attribute("range"));
            range *= lengthScaling;
            setAttr(elem, "range", range);
        }
    }
    else if (isElement(elem, "position")
             //&& isElement(elem.Parent()->ToElement(), "actuator")
             && hasAttr(elem, "ctrlrange"))
    {
        std::cout << t << "Scaling ctrlrange of position actuator " 
                  << elem->Attribute("name") << std::endl;
        
        Eigen::Vector2f ctrlrange = strToVec2(elem->Attribute("ctrlrange"));
        ctrlrange *= lengthScaling;
        setAttr(elem, "ctrlrange", ctrlrange);
    }
    else if (hasAttr(elem, "pos"))
    {
        std::cout << t << "Scaling pos of " << elem->Value() << " ";
        if (hasAttr(elem, "name"))
        {
            std::cout << "'" << elem->Attribute("name") << "'";
        }
        else
        {
            std::cout << "element";
        }
        std::cout << std::endl;
        
        Eigen::Vector3f pos = strToVec(elem->Attribute("pos"));
        pos *= lengthScaling;
        setAttr(elem, "pos", pos);
    }
    
    
    for (XMLElement* child = elem->FirstChildElement();
         child;
         child = child->NextSiblingElement())
    {
        scaleLengths(child);
    }
}


struct ListElementsVisitor : public tinyxml2::XMLVisitor
{
    
    ListElementsVisitor(const std::string& elementName) : elementName(elementName) {}
    ~ListElementsVisitor() override = default;

    // XMLVisitor interface
    bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
    const std::vector<const tinyxml2::XMLElement*>& getFoundElements() const;
    
    std::string elementName;
    std::vector<const tinyxml2::XMLElement*> foundElements;
};

bool ListElementsVisitor::VisitEnter(const tinyxml2::XMLElement& elem, const tinyxml2::XMLAttribute*)
{
    if (isElement(elem, elementName))
    {
        foundElements.push_back(&elem);
    }
    return true;
}

std::vector<const mjcf::XMLElement*> MujocoIO::getAllElements(const std::string& elemName)
{
    ListElementsVisitor visitor(elemName);
    document->worldbody()->Accept(&visitor);
    return visitor.foundElements;
}

void MujocoIO::setLengthScaling(float value)
{
    lengthScaling = value;
}

void MujocoIO::setActuatorType(ActuatorType value)
{
    actuatorType = value;
}

void MujocoIO::setWithMocapBody(bool enabled)
{
    withMocapBody = enabled;
}

