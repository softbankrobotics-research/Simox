#include "RrtWorkspaceVisualization.h"
#include "MotionPlanning/CSpace/CSpace.h"
#include "MotionPlanning/CSpace/CSpaceNode.h"
#include "MotionPlanning/CSpace/CSpacePath.h"
#include "MotionPlanning/CSpace/CSpaceTree.h"
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace MotionPlanning
{

    RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string& TCPName)
    {
        this->robot = robot;
        this->cspace = cspace;
        MOTIONPLANNING_ASSERT(robot);
        MOTIONPLANNING_ASSERT(cspace);
        robotNodeSet = cspace->getJointSet();
        MOTIONPLANNING_ASSERT(robotNodeSet);
        setTCPName(TCPName);
        init();
    }


    RrtWorkspaceVisualization::RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::JointSetPtr robotNodeSet, const std::string& TCPName)
    {
        this->robot = robot;
        cspace = CSpacePtr();
        MOTIONPLANNING_ASSERT(robot);
        MOTIONPLANNING_ASSERT(robotNodeSet);
        this->robotNodeSet = robotNodeSet;
        setTCPName(TCPName);
        init();
    }

    RrtWorkspaceVisualization::~RrtWorkspaceVisualization()
    {
    }

    void RrtWorkspaceVisualization::init()
    {
        visualization = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->createVisualisationSet({});

        setPathStyle();
        setTreeStyle();

        RenderColors red;
        red.nodeR = 1.0f;
        red.nodeG = 0;
        red.nodeB = 0;
        red.lineR = 0.5f;
        red.lineG = 0.5f;
        red.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eRed] = red;

        RenderColors blue;
        blue.nodeR = 0;
        blue.nodeG = 0;
        blue.nodeB = 1.0f;
        blue.lineR = 0.5f;
        blue.lineG = 0.5f;
        blue.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eBlue] = blue;

        RenderColors green;
        green.nodeR = 0;
        green.nodeG = 1.0f;
        green.nodeB = 0;
        green.lineR = 0.5f;
        green.lineG = 0.5f;
        green.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eGreen] = green;

        RenderColors custom;
        custom.nodeR = 1.0f;
        custom.nodeG = 1.0f;
        custom.nodeB = 0;
        custom.lineR = 0.5f;
        custom.lineG = 0.5f;
        custom.lineB = 0.5f;
        colors[RrtWorkspaceVisualization::eCustom] = custom;

    }


    void RrtWorkspaceVisualization::setPathStyle(float lineSize, float nodeSize, float /*renderComplexity*/)
    {
        pathLineSize = lineSize;
        pathNodeSize = nodeSize;
    }

    bool RrtWorkspaceVisualization::addTree(const CSpaceTreePtr& tree, RrtWorkspaceVisualization::ColorSet colorSet)
    {
        const auto visualizationFactory = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory();
        if (!tree)
        {
            return false;
        }

        if (tree->getDimension() != robotNodeSet->getSize())
        {
            VR_ERROR << " Dimensions do not match: " << tree->getDimension() << "!=" << robotNodeSet->getSize() << endl;
            return false;
        }

        if (!TCPNode)
        {
            return false;
        }

        VirtualRobot::Visualization::Color colorLine(colors[colorSet].lineR, colors[colorSet].lineG, colors[colorSet].lineB);
        VirtualRobot::Visualization::Color colorNode(colors[colorSet].nodeR, colors[colorSet].nodeG, colors[colorSet].nodeB);
        std::map<int, VirtualRobot::Visualization::Color> statusMaterials;
        bool considerStatus = !treeNodeStatusColor.empty();

        for (auto& elem : treeNodeStatusColor)
        {
            VirtualRobot::Visualization::Color c(colors[elem.second].nodeR, colors[elem.second].nodeG, colors[elem.second].nodeB);
            statusMaterials[elem.first] = c;
        }

        std::vector<CSpaceNodePtr> nodes = tree->getNodes();

        // pre-compute tcp positions
        bool updateVisMode = robot->getUpdateVisualization();
        robot->setUpdateVisualization(false);
        std::map<CSpaceNodePtr, Eigen::Vector3f> tcpCoords;
        for (auto& actualNode : nodes)
        {
            // get tcp coords:
            robotNodeSet->setJointValues(actualNode->configuration);
            tcpCoords[actualNode] = TCPNode->getGlobalPose().block<3, 1>(0, 3);
        }
        robot->setUpdateVisualization(updateVisMode);

        std::vector<Eigen::Vector3f> points;
        std::vector<Eigen::Vector3f> from, to;
        for (auto& actualNode : nodes)
        {
            Eigen::Vector3f p = tcpCoords[actualNode];
            points.push_back(p);

            // not for the start node! startNode->parentID < 0
            int parentid = actualNode->parentID;
            if (parentid >= 0) // lines for all configurations
            {
                // create line to parent
                auto parentNode = tree->getNode(parentid);

                if (parentNode)
                {
                    // get tcp coords
                    Eigen::Vector3f p2 = tcpCoords[parentNode];
                    from.push_back(p);
                    to.push_back(p2);
                }
            }
        }

        VirtualRobot::VisualizationSetPtr visuPoints = visualizationFactory->createPointCloud(points, treeNodeSize);
        for (size_t i=0; i<nodes.size(); ++i)
        {
            auto actualNode = nodes[i];
            auto pointVisu = visuPoints->at(i);
            if (considerStatus && statusMaterials.find(actualNode->status) != statusMaterials.end())
            {
                pointVisu->setColor(statusMaterials[actualNode->status]);
            }
            else
            {
                pointVisu->setColor(colorNode);
            }
        }

        VirtualRobot::VisualizationSetPtr visuLines = visualizationFactory->createLineSet(from, to, treeLineSize);
        visuLines->setColor(colorLine);

        visualization->addVisualization(visuPoints);
        visualization->addVisualization(visuLines);

        return true;
    }

    void RrtWorkspaceVisualization::setTreeStyle(float lineSize, float nodeSize, float /*renderComplexity*/)
    {
        treeLineSize = lineSize;
        treeNodeSize = nodeSize;
    }

    bool RrtWorkspaceVisualization::addConfiguration(const Eigen::VectorXf &c, RrtWorkspaceVisualization::ColorSet colorSet, float nodeSizeFactor)
    {
        if (c.rows() != robotNodeSet->getSize())
        {
            VR_ERROR << " Dimensions do not match: " << c.rows() << "!=" << robotNodeSet->getSize() << endl;
            return false;
        }

        if (!TCPNode)
        {
            return false;
        }

        float nodeSolutionSize = pathNodeSize * nodeSizeFactor; //15.0;//1.0f
        VirtualRobot::Visualization::Color colorNode(colors[colorSet].nodeR, colors[colorSet].nodeG, colors[colorSet].nodeB);

        // get tcp coords:
        robotNodeSet->setJointValues(c);
        Eigen::Matrix4f m = TCPNode->getGlobalPose();
        m.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

        auto visu = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->createPoint(nodeSolutionSize);
        visu->setGlobalPose(m);
        visu->setColor(colorNode);

        visualization->addVisualization(visu);
        return true;
    }

    void RrtWorkspaceVisualization::setCustomColor(float nodeR, float nodeG, float nodeB, float lineR, float lineG, float lineB)
    {
        RenderColors custom;
        custom.nodeR = nodeR;
        custom.nodeG = nodeG;
        custom.nodeB = nodeB;
        custom.lineR = lineR;
        custom.lineG = lineG;
        custom.lineB = lineB;
        colors[RrtWorkspaceVisualization::eCustom] = custom;
    }


    void RrtWorkspaceVisualization::reset()
    {
        for (auto visu : visualization->getVisualizations())
        {
            visualization->removeVisualization(visu);
        }
    }

    void RrtWorkspaceVisualization::setTCPName(const std::string TCPName)
    {
        this->TCPName = TCPName;
        TCPNode = robot->getModelNode(TCPName);

        if (!TCPNode)
        {
            VR_ERROR << "No node with name " << TCPName << " available in robot.." << endl;
        }
    }

    bool RrtWorkspaceVisualization::addCSpacePath(const CSpacePathPtr& path, RrtWorkspaceVisualization::ColorSet colorSet)
    {
        if (!path || !robotNodeSet || !TCPNode || !robot)
        {
            return false;
        }

        if (path->getDimension() != robotNodeSet->getSize())
        {
            VR_ERROR << " Dimensions do not match: " << path->getDimension() << "!=" << robotNodeSet->getSize() << endl;
            return false;
        }

        float nodeSolutionSize = pathNodeSize;//15.0;//1.0f
        float lineSolutionSize = pathLineSize;//4.0;
        VirtualRobot::Visualization::Color nodeColor(colors[colorSet].nodeR, colors[colorSet].nodeG, colors[colorSet].nodeB);
        VirtualRobot::Visualization::Color lineColor(colors[colorSet].lineR, colors[colorSet].lineG, colors[colorSet].lineB);

        float x, y, z;
        float x2 = 0.0f, y2 = 0.0f, z2 = 0.0f;

        std::vector<Eigen::Vector3f> points, from, to;
        for (unsigned int i = 0; i < path->getNrOfPoints(); i++)
        {
            if (cspace->hasExclusiveRobotAccess())
            {
                CSpace::lock();
            }

            // get tcp coords:
            robotNodeSet->setJointValues(path->getPoint(i));
            Eigen::Vector3f p = TCPNode->getGlobalPose().block<3, 1>(0, 3);

            if (cspace->hasExclusiveRobotAccess())
            {
                CSpace::unlock();
            }

            points.push_back(p);
            if (i > 0) // lines for all configurations
            {
                to.push_back(p);
            }
            if (i < path->getNrOfPoints() - 1)
            {
                from.push_back(p);
            }
        }

        auto visuPoints = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->createPointCloud(points, nodeSolutionSize);
        visuPoints->setColor(nodeColor);
        visualization->addVisualization(visuPoints);

        auto visuLines = VirtualRobot::VisualizationFactory::getGlobalVisualizationFactory()->createLineSet(from, to, lineSolutionSize);
        visuLines->setColor(lineColor);
        visualization->addVisualization(visuLines);

        return true;
    }

    void RrtWorkspaceVisualization::colorizeTreeNodes(int status, ColorSet colorSet)
    {
        treeNodeStatusColor[status] = colorSet;
    }

    /*
    bool RrtWorkspaceVisualization::addConfig( const Eigen::VectorXf &c )
    {
        return false;
    }*/

} // namespace MotionPlanning
