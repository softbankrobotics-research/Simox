/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef __Saba_ElasticBandProcessor_h__
#define __Saba_ElasticBandProcessor_h__

#include "../Saba.h"
#include "PathProcessor.h"
#include "VirtualRobot/Nodes/RobotNode.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/IK/GenericIKSolver.h"

namespace Saba
{
    /*!
     *
     * \brief The ElasticBandProcessor uses Cartesian distance vectors to move c-space points in order to produce smooth trajectories.
     *
     */
    class SABA_IMPORT_EXPORT ElasticBandProcessor : public PathProcessor
    {
    public:

        ElasticBandProcessor(CSpacePathPtr path,                            // implicitly defines rns to operate on (path->cspace->rns)
                             CSpaceSampledPtr cspace,
                             VirtualRobot::RobotNodePtr node,               // the distance for this node is considered
                             VirtualRobot::SceneObjectSetPtr obstacles,     // these obstacles are considered for path smoothing
                             bool verbose = false);
        ~ElasticBandProcessor() override;

        //! A wrapper to the standard interface.
        CSpacePathPtr optimize(int optimizeSteps) override;

        //! could also be used to disable specific dimensions
        void setWeights (Eigen::VectorXf w);


		/*!
		 * Writes ext and int forces of point i to given variables.
		 */
		void getForces(unsigned int i, Eigen::Vector3f &internalForce, Eigen::Vector3f &externalForce);

		Eigen::Vector3f getWSpacePoint(const Eigen::VectorXf& fc);

    protected:

        bool getCSpaceForce(const Eigen::Vector3f &f, Eigen::VectorXf &fc, float factor, float maxForce);
        
		bool getWSpaceForce(const Eigen::VectorXf& fc, Eigen::Vector3f &f);
    
        bool getObstacleForce(Eigen::Vector3f& f);
        bool initSolution();

        bool elasticBandLoop();
        bool checkRemoveNodes();
        bool checkNewNodes();

        Eigen::VectorXf weights;

        CSpaceSampledPtr cspace;
        VirtualRobot::RobotNodePtr node; // this is the node to move around (e.g. platform, tcp, ...)
        VirtualRobot::SceneObjectSetPtr obstacles; // this cdm is used to calculate the distance between robot and environment
        VirtualRobot::RobotNodeSetPtr rns;
        VirtualRobot::CollisionCheckerPtr colChecker;
        VirtualRobot::GenericIKSolverPtr ik;

        float factorCSpaceNeighborForce;
        float factorCSpaceObstacleForce;
        float maxCSpaceNeighborForce;
        float maxCSpaceObstacleForce;
        float minObstacleDistance;
        bool getNeighborCForce(const Eigen::VectorXf &before, const Eigen::VectorXf &act, const Eigen::VectorXf &next, Eigen::VectorXf &fc);
    };

}// namespace

#endif // __Saba_ElasticBandProcessor_h__
