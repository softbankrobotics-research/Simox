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
* @package    VirtualRobot
* @date       2011-02-24
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*
*
*/

#ifndef _VirtualRobot_CDManager_h_
#define _VirtualRobot_CDManager_h_

#include "../Model/Model.h"
#include "CollisionModel.h"
#include "../Model/ModelNodeSet.h"
#include "CollisionChecker.h"

#include <vector>
#include <set>
#include <string>


namespace VirtualRobot
{
    /*!
    *
    * A framework that can handle different sets of collision models.
    * With a collision detection manager (cdm) multiple sets collision models can be specified for convenient collision
    * detection or distance calculations.
    * Two methods can be used to add collisionModelSets:
    * addCollisionModel(): All added sets are checked against each other.
    * addCollisionModelPair(): Only the specific pairs are checked.
    *
    * The methods can be safely mixed.
    *
    * @see CollsionModelSet
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CDManager
    {
    public:
        //! if pColChecker is not set, the global collision checker is used
        CDManager(const CollisionCheckerPtr &colChecker = CollisionCheckerPtr());
        CDManager(const CDManager&) = default;

        virtual ~CDManager();

        /*!
            Sets of Links can be added.
            All added LinkSet sets are checked against each other.
            Internally for all SceneObjectSets that have been added earlier, the method addCollisionModelPair() is called.
        */
        void addCollisionModel(const LinkSetPtr &m);
        void addCollisionModel(const ModelPtr &m);
        void addCollisionModel(const ModelSetPtr &m);

        /*!
            Here, a specific pair of SceneObjectSets can be added.
            m1 and m2 will only be checked against each other
            and will not be checked against the other SceneObjectSets, that may be part of this CDManager.
            @param m1 The first link set
            @param m2 The second link set
            @param addToModelsList If true, the link sets will be considered by all other objects that are added later on.
        */
        void addCollisionModelPair(const LinkSetPtr &m1, const LinkSetPtr &m2, bool addToModelsList = true);
        void addCollisionModelPair(const ModelLinkPtr &m1, const LinkSetPtr &m2);
        void addCollisionModelPair(const ModelLinkPtr &m1, const ModelLinkPtr &m2);

        /*!
            Here single collision models can be added. Internally they are wrapped by a SceneObjectSet.
        */
        void addCollisionModel(const ModelLinkPtr &m);
        void addCollisionModel(const std::vector<ModelLinkPtr>& m);


        bool hasSceneObjectSet(const LinkSetPtr &m);
        bool hasSceneObject(const ModelLinkPtr &m);

        //! Returns true if one of the specified collision checks report a collision.
        bool isInCollision();

        /*!
            Checks if the model m collides with one of the added colModels. (all SceneObjectSets that have been added are considered)
            It is allowed to use an already added SceneObjectSets.
            Returns true if there is a collision.
        */
        bool isInCollision(const LinkSetPtr &m);

        //! Returns minimum distance of all sets that have been added for consideration.
        float getDistance();

        /*!
            Calculates the shortest distance of m to all added SceneObjectSets.
            m may be an already added SceneObjectSets.
        */
        float getDistance(const LinkSetPtr &m);

        //! Stores min dist position and collision IDs
        float getDistance(Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);

        /*!
            Calculates the shortest distance of SceneObjectSet m to all added colModels.
            Stores nearest positions and corresponding IDs, where P1 and trID1 is used to store the data of m and
            P2 and trID2 is used to store the data of this CDManager.
        */
        float getDistance(const LinkSetPtr &m, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);


        //! All SceneObjectSets that have been added.
        std::vector<LinkSetPtr> getSceneObjectSets();

        CollisionCheckerPtr getCollisionChecker();

    protected:
        /*!
            Performs also a check for sets with only one object added in order to cover potentionally added single SceneObjects.
        */
        bool _hasSceneObjectSet(const LinkSetPtr &m);

        bool isInCollision(const LinkSetPtr &m, const std::vector<LinkSetPtr>& sets);
        float getDistance(const LinkSetPtr &m, const std::vector<LinkSetPtr>& sets, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);
        float getDistance(const LinkSetPtr &m, const std::vector<LinkSetPtr>& sets);
        std::vector< LinkSetPtr > colModels;
        CollisionCheckerPtr colChecker;

        std::vector< ModelPtr > models;
        std::map<LinkSetPtr, std::vector<LinkSetPtr> > colModelPairs;

    };

}

#endif // _VirtualRobot_CDManager_h_

