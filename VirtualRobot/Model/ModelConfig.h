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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelConfig_h_
#define _VirtualRobot_ModelConfig_h_

#include "../Model/Model.h"

#include "Model.h"

#include <string>
#include <vector>
#include <map>


namespace VirtualRobot
{
    /*!
        A ModelConfig is a set of joint values, associated with a model.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelConfig
    {
    public:

        struct Configuration
        {
            std::string name;   //!< The name of the model node
            float value;        //!< The corresponding value
        };

        /*!
            Constructor
            \param model The associated model.
            \param name A name, which identifies this object.
        */
        ModelConfig(ModelWeakPtr model, const std::string& name);
        ModelConfig(ModelWeakPtr model, const std::string& name, const std::map< ModelJointPtr, float >& configs);
        ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< Configuration >& configs);
        ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< std::string >& modelNodes, const std::vector< float >& values);
        ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< ModelJointPtr >& modelNodes, const std::vector< float >& values);

        /*!
            Creates a copy of this object with the given model
            \param newModel If not given, the current model is used. otherwise the newModel is used.
         */
        ModelConfigPtr clone(const ModelPtr& newModel = ModelPtr());

        /*!
            Returns name of this ModelConfig.
        */
        std::string getName() const;

        void print() const;

        /*!
            The model.
            \return A shared_ptr instance of the internally stored weak pointer.
        */
        ModelPtr getModel();

        /*!
            Appends a configuration to this instance.
            \return True on success. False if model is not present any more (may happen due to the use of weak pointers).
        */
        bool setConfig(const Configuration& c);
        bool setConfig(ModelJointPtr node, float value);
        bool setConfig(const std::string& node, float value);

        bool setTCP(const std::string &tcpName);
        bool setTCP(RobotNodePtr tcp);
        bool hasTCP() const;
        RobotNodePtr getTCP();

        /*!
            Apply the stored configurations to the corresponding model.
            ModelNodes that are not stored in this ModelConfig are not affected.
            \return True on success. False if model is not present any more (may happen due to the use of weak pointers).
        */
        bool setJointValues();

        /*!
            Usually setJointValues() is sufficient for applying the joint values. But in some cases one might want to
            apply the joint values to a cloned model. Therefore this method can be used.
        */
        bool setJointValues(ModelPtr r);

        /*!
            Check if a configuration for a ModelNode with name is stored.
        */
        bool hasConfig(const std::string& name) const;
        /*!
            Return the corresponding stored value. If no ModelNode with name is stored, 0.0f is returned.
        */
        float getConfig(const std::string& name) const;

        /*!
            Return vector of all nodes that are covered by this ModelConfig.
        */
        std::vector< ModelJointPtr > getNodes() const;

        /*!
            Returns map of ModelNodeNames with corresponding joint values.
        */
        std::map < std::string, float > getJointNameValueMap();

        /*!
            Create an XML string that defines this object.
        */
        std::string toXML(int tabs = 0);

        static std::string createXMLString(const std::map< std::string, float >& config, const std::string& name, const std::string& tcpName = std::string(), int tabs = 0);

    protected:
        std::string name;

<<<<<<< HEAD:VirtualRobot/Model/ModelConfig.h
        std::map< ModelJointPtr, float > configs;
        ModelWeakPtr model;
=======
        std::map< RobotNodePtr, float > configs;
        RobotWeakPtr robot;
        RobotNodePtr tcpNode;
>>>>>>> origin/master:VirtualRobot/RobotConfig.h
    };


} // namespace VirtualRobot

#endif // _VirtualRobot_ModelConfig_h_
