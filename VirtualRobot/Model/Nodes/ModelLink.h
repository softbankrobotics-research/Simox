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
#pragma once

#include "ModelNode.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelLink : public ModelNode
    {
    public:
        struct VIRTUAL_ROBOT_IMPORT_EXPORT Physics
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            enum CoMLocation
            {
                eCustom,            //!< Not related to 3d model, the position is set by hand
                eVisuBBoxCenter     //!< The CoM position is automatically computed from the bounding box of the collision model
            };
            enum SimulationType
            {
                eStatic,        // cannot move, but collide
                eKinematic,     // can be moved, but no dynamics
                eDynamic,       // full dynamic simulation
                eUnknown        // not specified
            };

            // methods
            Physics();
            Physics(const Eigen::Vector3f& localCoM,
            float massKg,
            float friction,
            CoMLocation comLocation,
            const Eigen::Matrix3f& inertiaMatrix,
            SimulationType simType,
            const std::vector< std::string >& ignoreCollisions);

            static std::string simulationType2String(SimulationType s);
            static SimulationType string2SimulationType(const std::string& s);

            void print() const;

            bool isSet();

            virtual std::string toXML(int tabs);

            Physics scale(float scaling) const;

            Physics clone(float scaling) const;

            Physics &operator=(Physics other);

            // data members
            Eigen::Vector3f localCoM;   //!< Defined in the local coordinate system of this object [mm]
            float massKg;               //!< The mass of this object
            float friction;             //!< Friction of this object. Use -1.0 to use simulator's default value.
            CoMLocation comLocation;    //!< Where is the CoM located
            Eigen::Matrix3f inertiaMatrix; //! in kg*m^2
            SimulationType simType;
            std::vector< std::string > ignoreCollisions; // ignore collisions with other objects (only used within collision engines)
        };

        enum VisualizationType
        {
            Full,           //!< the full model
            Collision,      //!< the collision model
            CollisionData   //!< a visualization of the collision model data that is internally used (this mode is only for debug purposes, the model is static, i.e. updates/movements/rotations are not visualiz
        };

        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         * @param visualization The visualisation of this link.
         * @param collisionModel The collision model of this link.
         * @param p The physics object of this link.
         * @param colChecker The collision checker for this link.
         */
        ModelLink(const ModelWeakPtr& model,
                  const std::string& name,
                  const Eigen::Matrix4f& localTransformation,
                  const VisualizationPtr& visualization = VisualizationPtr(),
                  const CollisionModelPtr& collisionModel = CollisionModelPtr(),
                  const Physics& p = Physics(),
                  const CollisionCheckerPtr& colChecker = CollisionCheckerPtr());

        /*!
         * Destructor.
         */
        virtual ~ModelLink() override;

        virtual void setGlobalPose(const Eigen::Matrix4f& pose) override;

        //virtual void initialize(const ModelNodePtr& parent, const std::vector<ModelNodePtr>& children) override;

        virtual NodeType getType() const override;
        virtual bool isJoint() const override
        {
            return false;
        }
        virtual bool isTranslationalJoint() const override
        {
            return false;
        }
        virtual bool isRotationalJoint() const override
        {
            return false;
        }
        virtual bool isFixedJoint() const override
        {
            return false;
        }
        virtual bool isLink() const override
        {
            return true;
        }

        /*!
         * Get the collision model of this link.
         *
         * @return The collision model.
         */
        CollisionModelPtr getCollisionModel() const;

        /*!
         * Set a new collision model.
         *
         * @param colModel The new collision model.
         * @param keepUpdateVisualization If true, update visualisation of colModel is set to the status of the previous model.
         */
        void setCollisionModel(const CollisionModelPtr& colModel, bool keepUpdateVisualization = true);

        /*!
         * Get the collision checker of this node.
         *
         * @return The collision checker.
         */
        CollisionCheckerPtr getCollisionChecker() const;

        /*!
         * Set a new visualisation.
         *
         * @param visualization The new visualisation.
         * @param keepUpdateVisualization If true, update visualisation of visualization is set to the status of the previous model.
         */
        void setVisualization(const VisualizationPtr& visualization, bool keepUpdateVisualization = true);

        /*!
         * Get visualization object.
         *
         * @param visuType Set the type of visualization.
         * @return The visualisation of this link.
         */
        VisualizationPtr getVisualization(VisualizationType visuType = VisualizationType::Full) const;

        /*!
         * Get number of faces (i.e. triangles) of this object.
         *
         * @param collisionModel Indicates weather the faces of the collision model or the full model should be returned.
         * @return The number of faces.
         */
        int getNumFaces(bool collisionModel = false);

        /*!
         * Get the physics information of this node.
         * This is a copy of the physics object and gets no updates.
         *
         * @return A physics object containing all physics information.
         */
        Physics getPhysics() const;

        /*!
         * Get the simulation type of this node.
         *
         * The simulation type is of interest in SimDynamics.
         *
         * @return The simulation type of this node.
         */
        Physics::SimulationType getSimulationType() const;

        /*!
         * Set the simulation type of this node.
         *
         * The simulation type is of interest in SimDynamics.
         *
         * @param s The new simulation type of this node.
         */
        void setSimulationType(Physics::SimulationType s);

        /*!
         * Get the names of all collision models, this model should not collide with.
         *
         * Collisions with these models are ignored by physics engine (only considered within the SimDynamics package!).
         *
         * @return A vector of all names.
         */
        std::vector<std::string> getIgnoredCollisionModels() const;

        /*!
         * Return Center of Mass in local coordinate frame. This method does not consider children.
         *
         * @return The CoM.
         */
        Eigen::Vector3f getCoMLocal() const;

        /*!
         * Return Center of Mass in global coordinates. This method does not consider children.
         *
         * @return The CoM.
         */
        Eigen::Vector3f getCoMGlobal() const;

        /*!
         * Get the mass in Kg.
         *
         * @return The mass in Kg.
         */
        float getMass() const;

        /*!
         * Set the mass.
         *
         * @param m The mass in Kg.
         */
        void setMass(float m);

        /*!
         * Get the inertia matrix in kg*m^2.
         *
         * @return The inertia matrix in kg*m^2.
         */
        Eigen::Matrix3f getInertiaMatrix() const;

        /*!
         * Set the inertia matrix.
         *
         * @param im The inertia matrix in kg*m^2.
         */
        void setInertiaMatrix(const Eigen::Matrix3f& im);
        
         /**
         * @brief If the Inertia Matrix is given at the CoM, this function returns the Inertia Matrix at the parallel shifted coordinate system.
         * The shift is done using the parallel axis theorem (https://en.wikipedia.org/wiki/Parallel_axis_theorem)
         * @param shift How the system should be shifted.
         * @return The Inertia Matrix at the shifted system
         */
        Eigen::Matrix3f getInertiaMatrix(const Eigen::Vector3f& shift);
        Eigen::Matrix3f getInertiaMatrix(const Eigen::Vector3f& shift, const Eigen::Matrix3f& rotation);

        Eigen::Matrix3f getInertiaMatrix(const Eigen::Matrix4f& transform);

        float getFriction();
        void setFriction(float friction);


        /*!
         * Creates an XML string that defines the ModelNode. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
         *
         * \see ModelIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPathRelative TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         *
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", bool storeAttachments = true) override;

        bool saveModelFiles(const std::string& modelPath);
    protected:
        virtual ModelNodePtr _clone(ModelPtr newModel, float scaling = 1.0f) override;

        bool initializePhysics();
        virtual void updatePoseInternally(bool updateChildren, bool updateAttachments) override;

    private:
        VisualizationPtr visualizationModel;
        CollisionModelPtr collisionModel;
        ModelLink::Physics physics;
        CollisionCheckerPtr collisionChecker;
    };
}
