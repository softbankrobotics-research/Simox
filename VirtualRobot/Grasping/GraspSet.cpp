
#include "GraspSet.h"
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <vector>
#include <VirtualRobot/VirtualRobotException.h>

namespace VirtualRobot
{


    GraspSet::GraspSet(const std::string& name, const std::string& robotType, const std::string& eef, const std::vector< GraspPtr >& grasps)
        : grasps(grasps), name(name), robotType(robotType), eef(eef)
    {

    }

    GraspSet::~GraspSet()
    = default;

    void GraspSet::addGrasp(GraspPtr grasp)
    {
        VR_ASSERT_MESSAGE(grasp, "NULL grasp");
        VR_ASSERT_MESSAGE(!hasGrasp(grasp), "Grasp already added!");
        VR_ASSERT_MESSAGE(isCompatibleGrasp(grasp), "Grasp is not compatible with this grasp set");


        grasps.push_back(grasp);
    }

    bool GraspSet::hasGrasp(GraspPtr grasp)
    {
        VR_ASSERT_MESSAGE(grasp, "NULL grasp");

        for (const auto & i : grasps)
            if (i == grasp)
            {
                return true;
            }

        return false;
    }

    bool GraspSet::hasGrasp(const std::string& name)
    {
        for (auto & grasp : grasps)
        {
            if (grasp->getName() == name)
            {
                return true;
            }
        }

        return false;
    }


    void GraspSet::clear()
    {
        grasps.clear();
    }

    void GraspSet::includeGraspSet(GraspSetPtr grasps)
    {
        std::vector<GraspPtr> includedGrasp=grasps->getGrasps();

        for(const auto & i : includedGrasp)
        {
            if(!hasGrasp(i))
            {
                addGrasp(i);
            }
        }
    }

    void GraspSet::print()
    {
        cout << "**** Grasp set ****" << endl;
        cout << "Name: " << name << endl;
        cout << "Robot Type: " << robotType << endl;
        cout << "End Effector: " << eef << endl;
        cout << "Grasps:" << endl;

        for (size_t i = 0; i < grasps.size(); i++)
        {
            cout << "** grasp " << i << ":" << endl;
            grasps[i]->print(false);
        }

        cout << endl;
    }

    bool GraspSet::isCompatibleGrasp(GraspPtr grasp)
    {
        if (grasp->getRobotType() != robotType)
        {
            return false;
        }

        if (grasp->getEefName() != eef)
        {
            return false;
        }

        return true;
    }

    unsigned int GraspSet::getSize()
    {
        return (unsigned int)grasps.size();
    }

    VirtualRobot::GraspPtr GraspSet::getGrasp(unsigned int n)
    {
        if (n >= (unsigned int)grasps.size())
        {
            return GraspPtr();
        }

        return grasps[n];
    }

    VirtualRobot::GraspPtr GraspSet::getGrasp(const std::string& name)
    {
        for (auto & grasp : grasps)
        {
            if (grasp->getName() == name)
            {
                return grasp;
            }
        }

        return GraspPtr();
    }


    std::string GraspSet::getName()
    {
        return name;
    }

    std::string GraspSet::getRobotType()
    {
        return robotType;
    }

    std::string GraspSet::getEndEffector()
    {
        return eef;
    }

    std::string GraspSet::getXMLString(int tabs)
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        ss << t << "<GraspSet name='" << name << "' RobotType='" << robotType << "' EndEffector='" << eef << "'>\n";

        for (auto & grasp : grasps)
        {
            ss << grasp->toXML(tabs + 1);
        }

        ss << t << "</GraspSet>\n";

        return ss.str();
    }


    VirtualRobot::GraspSetPtr GraspSet::clone()
    {
        GraspSetPtr res(new GraspSet(name, robotType, eef));

        // clone grasps
        for (auto & grasp : grasps)
        {
            res->addGrasp(grasp->clone());
        }

        return res;
    }

    bool GraspSet::removeGrasp(GraspPtr grasp)
    {
        for (std::vector< GraspPtr >::iterator i = grasps.begin(); i != grasps.end(); i++)
        {
            if (*i == grasp)
            {
                grasps.erase(i);
                return true;
            }
        }

        return false;
    }

    bool GraspSet::removeGrasp(unsigned int i)
    {
        GraspPtr g = getGrasp(i);
        if (!g)
            return false;
        return removeGrasp(g);
    }

    void GraspSet::removeAllGrasps()
    {
        grasps.clear();
    }

    std::vector< GraspPtr > GraspSet::getGrasps()
    {
        std::vector< GraspPtr > res;

        for (const auto & grasp : grasps)
        {
            res.push_back(grasp);
        }

        return res;
    }

    void GraspSet::setPreshape(const std::string& preshape)
    {
        for (auto & grasp : grasps)
        {
            grasp->setPreshape(preshape);
        }
    }



} //  namespace


