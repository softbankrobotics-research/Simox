#include "MujocoIO.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobotException.h>

#include <VirtualRobot/MJCF/visitors/Collector.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/RobotIO.h>


namespace fs = boost::filesystem;


namespace VirtualRobot::mujoco
{

ActuatorType toActuatorType(const std::string& string)
{
    static const std::map<std::string, ActuatorType> map
    {
        {"motor",    ActuatorType::MOTOR},
        {"position", ActuatorType::POSITION},
        {"velocity", ActuatorType::VELOCITY}
    };
    std::string lower = string;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return map.at(lower);
}


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
    
    makeDefaultsClass();
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
    
    if (verbose)
    {
        std::cout << "===========================" << std::endl
                  << "Current model: "             << std::endl
                  << "--------------"              << std::endl;
        std::cout << *document;
        std::cout << "===========================" << std::endl;
    }
    
    std::cout << "Merging massless bodies ..." << std::endl;
    masslessBodySanitizer.sanitize(robotRoot);
    
    std::cout << "Adding contact excludes ..." << std::endl;
    addContactExcludes();

    std::cout << "Adding actuators ..." << std::endl;
    addActuators();
    
    std::cout << "Scaling lengths by " << lengthScaling << " ..." << std::endl;
    scaleLengths(robotRoot);
    
    std::cout << "Done." << std::endl;
    
    if (verbose)
    {
        std::cout << std::endl;
        std::cout << "===========================" << std::endl
                  << "Output file: "             << std::endl
                  << "------------"              << std::endl;
        std::cout << *document;
        std::cout << "===========================" << std::endl;
    }
    
    assert(!outputFileName.empty());
    std::cout << "Writing to " << (outputDirectory / outputFileName) << std::endl;
    document->saveFile((outputDirectory / outputFileName).string());
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
    document->compiler().angle = "radian";
    document->compiler().balanceinertia = true;
}

void MujocoIO::makeDefaultsClass()
{
    mjcf::DefaultClass defaultsClass = document->default_().addClass(robot->getName());
    
    std::stringstream comment;
    comment << "Add default values for " << robot->getName() << " here.";
    defaultsClass.insertComment(comment.str(), true);
    
    mjcf::Joint joint = defaultsClass.getElement<mjcf::Joint>();
    joint.frictionloss = 1;
    joint.damping = 0;
    
    mjcf::Geom geom = defaultsClass.getElement<mjcf::Geom>();
    geom.condim = 4;
    
    mjcf::ActuatorPosition actPos = defaultsClass.getElement<mjcf::ActuatorPosition>();
    actPos.kp = 1;
    
    mjcf::ActuatorVelocity actVel = defaultsClass.getElement<mjcf::ActuatorVelocity >();
    actVel.kv = 1;
}


void MujocoIO::addSkybox()
{
    document->asset().addSkyboxTexture(Eigen::Vector3f(.8f, .9f, .95f), 
                                       Eigen::Vector3f(.4f, .6f, .8f));
}


void MujocoIO::addMocapBody()
{
    const std::string className = "mocap";
    const float geomSize = 0.01f;
    
    std::string bodyName;
    {
        std::stringstream ss;
        ss << robot->getName() << "_Mocap";
        bodyName = ss.str();
    }
    
    // add defaults class
    mjcf::DefaultClass defaultClass = document->default_().getClass(className);
    
    mjcf::Geom geom = defaultClass.getElement<mjcf::Geom>();
    geom.rgba = Eigen::Vector4f(.9f, .5f, .5f, .5f);
    
    {
        mjcf::EqualityDefaults equality = defaultClass.getElement<mjcf::EqualityDefaults>();
        Eigen::Vector5f solimp = equality.solimp;
        solimp(1) = 0.99f;
        equality.solimp = solimp;
        //equality.solref = Eigen::Vector2f(.02f, 1.f);
    }
    
    // add body
    mjcf::Body mocap = document->worldbody().addMocapBody(bodyName, geomSize);
    mocap.childclass = className;
    
    // add equality weld constraint
    mjcf::EqualityWeld weld = document->equality().addWeld(bodyName, robot->getName(), bodyName);
    weld.class_ = className;
}


void MujocoIO::makeNodeBodies()
{
    nodeBodies.clear();
    
    RobotNodePtr rootNode = robot->getRootNode();
    assert(rootNode);
    
    // add root
    robotRoot = document->worldbody().addBody(robot->getName(), robot->getName());
    
    if (withMocapBody)
    {
        robotRoot.addDummyInertial();
        mjcf::FreeJoint joint = robotRoot.addFreeJoint();
        joint.name = robot->getName();
    }
    
    mjcf::Body root = addNodeBody(robotRoot, rootNode);
    nodeBodies[rootNode->getName()] = root;
    assert(root);
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        addNodeBody(node);
    }
}

mjcf::Body MujocoIO::addNodeBody(mjcf::Body parent, RobotNodePtr node)
{
    mjcf::Body body = parent.addBody(node->getName());

    if (node->hasParent())
    {
        body.setPose(node->getTransformationFrom(node->getParent()));
    }
    
    if (node->isRotationalJoint() || node->isTranslationalJoint())
    {
        addNodeJoint(body, node);
    }
    
    addNodeInertial(body, node);
    
    return body;
}

mjcf::Joint MujocoIO::addNodeJoint(mjcf::Body body, RobotNodePtr node)
{
    assert(node->isRotationalJoint() xor node->isTranslationalJoint());
    
    mjcf::Joint joint = body.addJoint();
    joint.name = node->getName();
    
    // get the axis
    Eigen::Vector3f axis;
    if (node->isRotationalJoint())
    {
        RobotNodeRevolutePtr revolute = boost::dynamic_pointer_cast<RobotNodeRevolute>(node);
        assert(revolute);
        axis = revolute->getJointRotationAxisInJointCoordSystem();
    }
    else if (node->isTranslationalJoint())
    {
        RobotNodePrismaticPtr prismatic = boost::dynamic_pointer_cast<RobotNodePrismatic>(node);
        assert(prismatic);
        axis = prismatic->getJointTranslationDirectionJointCoordSystem();
    }
    
    joint.type = node->isRotationalJoint() ? "hinge" : "slide";
    joint.axis = axis;
    joint.limited = !node->isLimitless();
    
    if (!node->isLimitless())
    {
        joint.range = { node->getJointLimitLow(), node->getJointLimitHigh() };
    }
    
    return joint;
}

mjcf::Inertial MujocoIO::addNodeInertial(mjcf::Body body, RobotNodePtr node)
{
    Eigen::Matrix3f matrix = node->getInertiaMatrix();
    if (matrix.isIdentity(document->getFloatCompPrecision()) 
        && node->getMass() < document->getFloatCompPrecision())
    {
        // dont set an inertial element and let it be derived automatically
        return { };
    }
    
    mjcf::Inertial inertial = body.addInertial();
    inertial.pos = node->getCoMLocal();
    inertial.mass = node->getMass();
    inertial.inertiaFromMatrix(matrix);
    
    return inertial;
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
                std::cout << "Converting to .stl: " << srcMeshPath << std::endl;
                
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
                std::cout << "Running command: " << convertCommand.str() << std::endl;
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
        const std::string meshName = node->getName();
        document->asset().addMesh(meshName, fs::absolute(dstMeshPath).string());
        
        // add geom to body
        mjcf::Body& body = nodeBodies.at(node->getName());
        body.addGeomMesh(meshName);
    }
}



mjcf::Body MujocoIO::addNodeBody(RobotNodePtr node)
{
    // see whether body for the node already exists
    auto find = nodeBodies.find(node->getName());
    if (find != nodeBodies.end())
    {
        // exists => break recursion
        return find->second;
    }
    
    // check whether body exists for parent node
    mjcf::Body parent;
    find = nodeBodies.find(node->getParent()->getName());
    if (find == nodeBodies.end())
    {
        // parent does not exist => create it first
        parent = addNodeBody(robot->getRobotNode(node->getParent()->getName()));
    }
    else
    {
        // parent exists
        parent = find->second;
    }
    
    // add body as child of parent
    mjcf::Body body = addNodeBody(parent, node);
    nodeBodies[node->getName()] = body;

    return body;
}

struct ParentChildContactExcludeVisitor : public mjcf::Visitor
{
    ParentChildContactExcludeVisitor(mjcf::Document& document) : mjcf::Visitor (document)
    {}
    virtual ~ParentChildContactExcludeVisitor() override = default;

    //bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    bool visitEnter(const mjcf::AnyElement& element) override;
    
    std::vector<std::pair<std::string, std::string>> excludePairs;
    bool firstSkipped = false;  ///< Used to skip the root element.
};

bool ParentChildContactExcludeVisitor::visitEnter(const mjcf::AnyElement& element)
{
    if (!element.is<mjcf::Body>())
    {
        return true;
    }
    
    const mjcf::Body body = element.as<mjcf::Body>();
    
    if (!firstSkipped)
    {
        firstSkipped = true;
        return true;
    }
    
    const mjcf::Body parent = body.parent<mjcf::Body>();
    assert(parent);
    excludePairs.emplace_back(parent.name.get(), body.name.get());
    
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
    for (const auto& excludePair : excludePairs)
    {
        const std::string body1 = masslessBodySanitizer.getMergedBodyName(excludePair.first);
        const std::string body2 = masslessBodySanitizer.getMergedBodyName(excludePair.second);
        document->contact().addExclude(body1, body2);
    }
    
    // Add excludes between parent and child elemenets. 
    // This should actually not be necessary?
    ParentChildContactExcludeVisitor visitor(*document);
    robotRoot.accept(visitor);
    for (const auto& excludePair : visitor.excludePairs)
    {
        document->contact().addExclude(excludePair.first, excludePair.second);
    }
}

void MujocoIO::addActuators()
{
    std::vector<mjcf::Joint> jointElements = mjcf::Collector<mjcf::Joint>::collect(
                *document, document->worldbody());
    
    for (auto joint : jointElements)
    {
        const std::string name = joint.name;
        switch (actuatorType)
        {
            case ActuatorType::MOTOR:
            {
                mjcf::ActuatorMotor act = document->actuator().addMotor(name);
                act.name = joint.name;
                break;
            }
                
            case ActuatorType::POSITION:
            {
                mjcf::ActuatorPosition act = document->actuator().addPosition(name);
                act.name = joint.name;
                
                if (joint.limited)
                {
                    act.ctrllimited = joint.limited;
                    act.ctrlrange = joint.range;
                }
            }
                break;
                
            case ActuatorType::VELOCITY:
                mjcf::ActuatorVelocity act = document->actuator().addVelocity(name);
                act.name = joint.name;
                break;
        }
    }
}

void MujocoIO::scaleLengths(mjcf::AnyElement element)
{
    assert(elem);
    
    if (element.is<mjcf::Joint>())
    {
        mjcf::Joint joint = element.as<mjcf::Joint>();
        
        if (joint.type == "slide" || joint.range.isSet())
        {
            std::cout << t << "Scaling range of slide joint " << joint.name << std::endl;
            joint.range = joint.range.get() * lengthScaling;
        }
    }
    else if (element.is<mjcf::ActuatorPosition>())
    {
        mjcf::ActuatorPosition act = element.as<mjcf::ActuatorPosition>();
        if (act.ctrlrange.isSet())
        {
            std::cout << t << "Scaling ctrlrange of position actuator " << act.name << std::endl;
            act.ctrlrange = act.ctrlrange.get() * lengthScaling;
        }
    }
    else if (element.isAttributeSet("pos"))
    {
        std::cout << t << "Scaling pos of " << element.actualTag() << " ";
        if (element.isAttributeSet("name"))
        {
            std::cout << "'" << element.getAttribute("name") << "'";
        }
        else
        {
            std::cout << "element";
        }
        std::cout << std::endl;
        
        Eigen::Vector3f pos = element.getAttribute<Eigen::Vector3f>("pos");
        pos *= lengthScaling;
        element.setAttribute("pos", pos);
    }
    
    for (mjcf::AnyElement child = element.firstChild<mjcf::AnyElement>();
         child; child = child.nextSiblingElement<mjcf::AnyElement>())
    {
        scaleLengths(child);
    }
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

void MujocoIO::setVerbose(bool value)
{
    this->verbose = value;
}



}
