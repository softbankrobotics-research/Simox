
#include "GraspSet.h"
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/VisualizationSet.h>
#include <vector>
#include <VirtualRobot/VirtualRobotException.h>

namespace VirtualRobot
{


    GraspSet::GraspSet(const std::string& name, const std::string& robotType, const std::string& eef, const std::vector< GraspPtr >& grasps)
        : grasps(grasps), name(name), robotType(robotType), eef(eef)
    {

    }

    GraspSet::~GraspSet()
    {
    }

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

        for (size_t i = 0; i < grasps.size(); i++)
            if (grasps[i] == grasp)
            {
                return true;
            }

        return false;
    }

    bool GraspSet::hasGrasp(const std::string& name)
    {
        for (size_t i = 0; i < grasps.size(); i++)
        {
            if (grasps[i]->getName() == name)
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

        for(size_t i=0;i<includedGrasp.size();i++)
        {
            if(!hasGrasp(includedGrasp.at(i)))
            {
                addGrasp(includedGrasp.at(i));
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

    unsigned int GraspSet::getSize() const
    {
        return static_cast<unsigned int>(grasps.size());
    }

    VirtualRobot::GraspPtr GraspSet::getGrasp(unsigned int n) const
    {
        if (n >= getSize())
        {
            return GraspPtr();
        }

        return grasps[n];
    }

    VirtualRobot::GraspPtr GraspSet::getGrasp(const std::string& name) const
    {
        for (size_t i = 0; i < grasps.size(); i++)
        {
            if (grasps[i]->getName() == name)
            {
                return grasps[i];
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

    std::string GraspSet::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        ss << t << "<GraspSet name='" << name << "' RobotType='" << robotType << "' EndEffector='" << eef << "'>\n";

        for (size_t i = 0; i < grasps.size(); i++)
        {
            ss << grasps[i]->toXML(tabs + 1);
        }

        ss << t << "</GraspSet>\n";

        return ss.str();
    }


    VirtualRobot::GraspSetPtr GraspSet::clone()
    {
        GraspSetPtr res(new GraspSet(name, robotType, eef));

        // clone grasps
        for (std::vector< GraspPtr >::iterator i = grasps.begin(); i != grasps.end(); i++)
        {
            res->addGrasp((*i)->clone());
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

        for (size_t i = 0; i < grasps.size(); i++)
        {
            res.push_back(grasps[i]);
        }

        return res;
    }

    void GraspSet::setPreshape(const std::string& preshape)
    {
        for (size_t i = 0; i < grasps.size(); i++)
        {
            grasps[i]->setPreshape(preshape);
        }
    }

    VisualizationSetPtr GraspSet::getVisualization(ModelLink::VisualizationType visuType, const EndEffectorPtr &eef, const Eigen::Matrix4f& pose) const
    {
        VR_ASSERT(eef);
        std::vector<VisualizationPtr> visual;
        visual.reserve(getSize());

        for (unsigned int i = 0; i < getSize(); i++)
        {
            // grasp and transform
            GraspPtr g = getGrasp(i);
            auto graspVisu = g->getVisualization(visuType, eef, pose);
            visual.push_back(graspVisu);
        }

        return VisualizationFactory::getInstance()->createVisualisationSet(visual);
    }

} //  namespace


