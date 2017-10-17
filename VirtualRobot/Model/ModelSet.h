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
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelSet_h_
#define _VirtualRobot_ModelSet_h_

#include "../Model/Model.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelSet
    {
    public:
        /*!
         * Initialize this set with a vector of Models.
         *
         * @param name The name of this ModelSet.
         * @param models The models to add to this ModelSet.
         */
        ModelSet(const std::string& name,
                 const std::vector<ModelPtr>& models);

        /*!
         * Destructor.
         */
        virtual ~ModelSet();

        /*!
         * Get the name of this ModelSet.
         *
         * @return The name.
         */
        std::string getName() const;


        ModelPtr& operator[](int i)
        {
            return getModel(i);
        }

        /*!
         * Get the model at position i.
         *
         * @param i The position of the model to get.
         * @return The model.
         */
        ModelPtr& getModel(int i);

        /*!
         * Iterator starting at the first object of this set.
         *
         * @return The iterator.
         */
        std::vector<ModelPtr>::iterator begin();

        /*!
         * Iterator starting at the last object of this set.
         *
         * @return The iterator.
         */
        std::vector<ModelPtr>::iterator end();

        /*!
         * Check, if this set contains the given model.
         *
         * @param node The node to check for.
         * @return True, if the node is contained; false otherwise.
         */
        bool hasModel(const ModelPtr& node) const;

        /*!
         * Check, if this set contains the given model.
         *
         * @param modelName The name of the model to check for.
         * @return True, if the model is contained; false otherwise.
         */
        bool hasModel(const std::string &modelName) const;

        /*!
         * Get all nodes of this set.
         *
         * @return The models contained in this set.
         */
        const std::vector<ModelPtr> getModels() const;

        /*!
         * Print out some information.
         */
        void print() const;

        /*!
         * Get the size of this set.
         *
         * @return The number of associated model nodes.
         */
        virtual unsigned int getSize() const;

        std::vector< std::string > getModelNames() const;

        std::vector<CollisionModelPtr> getCollisionModels() const;
        std::vector<VisualizationNodePtr> getVisualizations() const;

        /*!
         * Create a XML string to represent this ModelNodeSet.
         *
         * @param tabs The number of tabs to start each line with.
         * @return The generated XML string.
         */
        virtual std::string toXML(int tabs) const;

        std::vector<ModelJointPtr> getModelJoints() const;

        std::vector<ModelLinkPtr> getModelLinks() const;

        CollisionCheckerPtr getCollisionChecker() const;

		/*!
        * Clone this modelset. Does not create a deep copy, just clones the pointers to all involved models.
		*/
        virtual ModelSetPtr clone(const std::string &name);

        virtual ObstaclePtr createStaticObstacle(const std::string &name) const;

    protected:
        std::string name;
        std::vector<ModelPtr> models;
    };
}

#endif
