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
* @author     Adrian Knobloch
* @copyright  2016 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_JointSet_h_
#define _VirtualRobot_JointSet_h_

#include "../Model/Model.h"
#include "ModelNodeSet.h"

namespace VirtualRobot
{
    class JointSet : public ModelNodeSet
    {
    protected:
        /*!
         * Initialize this set with a vector of ModelNodes.
         *
         * @param name The name of this JointSet.
         * @param model The associated model.
         * @param modelNodes The model nodes to add to this JointSet.
         * @param kinematicRoot    This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                         kinematicRoot does not have to be a node of this set.
         *                         If not given, the first entry of modelNodes will be set as the kinematic root.
         * @param tcp   The tcp.
         *              If not given, the last entry of modelNodes will be set as the tcp.
         */
        JointSet(const std::string& name,
                     const ModelWeakPtr& model,
                     const std::vector<ModelNodePtr>& modelNodes,
                     const ModelNodePtr kinematicRoot = ModelNodePtr(),
                     const CoordinatePtr tcp = CoordinatePtr());

    public:
        /*!
         * Destructor.
         */
        virtual ~JointSet();

        /*!
         * Create a new JointSet.
         *
         * @param model The associated model.
         * @param name The name of the new JointSet.
         * @param modelNodeNames The names of the model nodes to add.
         * @param kinematicRootName The name of the kinematic root.
         *                          This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                          The kinematic root does not have to be a node of this set.
         *                          If no name provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcpName The name of the tcp.
         *                The tcp does not have to be a node of this set.
         *                If no name provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new JointSet is registered to the model.
         * @return The newly created JointSet.
         */
        static JointSetPtr createJointSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<std::string>& modelNodeNames,
                                                  const std::string& kinematicRootName = "",
                                                  const std::string& tcpName = "",
                                                  bool registerToModel = false);
        /*!
         * Create a new JointSet.
         *
         * @param model The associated model.
         * @param name The name of the new JointSet.
         * @param modelNodes The nodes to add to this set.
         * @param kinematicRoot This specifies the first node of the model's kinematic tree to be used for updating all members of this set.
         *                      The kinematic root does not have to be a node of this set.
         *                      If no kinematic root provided, the first node of the given model nodes will be set as the kinematic root.
         * @param tcp The tcp.
         *            The tcp does not have to be a node of this set.
         *            If no tcp provided, the last node of the given model nodes will be set as the tcp node.
         * @param registerToModel If true, the new JointSet is registered to the model.
         * @return The newly created JointSet.
         */
        static JointSetPtr createJointSet(const ModelPtr& model,
                                                  const std::string& name,
                                                  const std::vector<ModelNodePtr>& modelNodes,
                                                  const ModelNodePtr kinematicRoot = ModelNodePtr(),
                                                  const CoordinatePtr tcp = CoordinatePtr(),
                                                  bool registerToModel = false);



        /*!
         * Get all nodes of this set.
         *
         * @return The nodes contained in this set.
         */
        const std::vector<ModelJointPtr> getJoints() const;

        /*!
         * Print out some information.
         */
        void print() const;

		ModelJointPtr& operator[](int i)
		{
			return getNode(i);
		}

		/*!
		* Get the node at position i.
		*
		* @param i The position of the node to get.
		* @return The node.
		*/
		ModelJointPtr& getNode(int i);

        /*!
         * Get the joint values of all contained joints.
         * TODO: The ordering of the values.
         *
         * @return The joint values.
         */
        std::vector<float> getJointValues() const;

        /*!
         * Get the joint values of all contained joints.
         * TODO: The ordering of the values.
         *
         * @param fillVector The vector to put the values in.
         * @param clearVector If true, all items in the vector are deleted.
         */
        void getJointValues(std::vector<float>& fillVector, bool clearVector = false) const;

        /*!
         * Get the joint values of all contained joints.
         * TODO: The ordering of the values.
         *
         * @param fillVector The vector to put the values in.
         */
        void getJointValues(Eigen::VectorXf& fillVector) const;

        /*!
         * Get the joint values of all contained joints.
         * TODO: The ordering of the values.
         *
         * @param config The config to save the joint values in.
         */
        void getJointValues(const ModelConfigPtr& config) const;

        /*!
         * Checks if the given joint values are within joint limits.
         * If not the joint values are adjusted.
         * TODO: The ordering of the values.
         *
         * @param jointValues The values to check and adjust.
         */
        void respectJointLimits(std::vector<float>& jointValues) const;

        /*!
         * Checks if the given joint values are within joint limits.
         * If not the joint values are adjusted.
         * TODO: The ordering of the values.
         *
         * @param jointValues The values to check and adjust.
         */
        void respectJointLimits(Eigen::VectorXf& jointValues) const;

        /*!
         * Checks if the jointValues are within the current joint limits.
         *
         * @param jointValues A vector of correct size.
         * @param verbose Print information if joint limits are violated.
         * @return True when all given joint values are within joint limits.
         */
        bool checkJointLimits(const std::vector<float>& jointValues, bool verbose = false) const;

        /*!
         * Checks if the jointValues are within the current joint limits.
         *
         * @param jointValues A vector of correct size.
         * @param verbose Print information if joint limits are violated.
         * @return True when all given joint values are within joint limits.
         */
        bool checkJointLimits(const Eigen::VectorXf& jointValues, bool verbose = false) const;

        /*!
         * Set joint values [rad].
         * The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
         *
         * @param jointValues A vector with joint values, size must be equal to number of joints in this RobotNodeSet.
         */
        virtual void setJointValues(const std::vector<float>& jointValues);

        /*!
         * Set joint values [rad].
         * The subpart of the robot, defined by the start joint (kinematicRoot) of rns, is updated to apply the new joint values.
         *
         * @param jointValues A vector with joint values, size must be equal to number of joints in this RobotNodeSet.
        */
        virtual void setJointValues(const Eigen::VectorXf& jointValues);

        /*!
         * Set joints that are within the given ModelConfig. Joints of this NodeSet that are not stored in jointValues remain untouched.
         *
         * @param config The config to get the joint values from.
         */
        virtual void setJointValues(const ModelConfigPtr& config);
        
        std::vector< std::string > getNodeNames() const;
        std::map< std::string, float > getJointValueMap() const;

        /*!
         * Create a XML string to represent this JointSet.
         *
         * @param tabs The number of tabs to start each line with.
         * @return The generated XML string.
         */
        virtual std::string toXML(int tabs);

		/*!
		* Clone this JointSet and register it to the new robot
		*/
		virtual ModelNodeSetPtr clone(ModelPtr newModel);

    private:
        std::vector<ModelJointPtr> joints;
    };
}

#endif // _VirtualRobot_JointSet_h_
