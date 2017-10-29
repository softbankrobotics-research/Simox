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
#ifndef _VirtualRobot_CollisionModel_h_
#define _VirtualRobot_CollisionModel_h_

#include "../Model/Model.h"
#include "../Tools/MathTools.h"
#include "../Tools/BoundingBox.h"

#include <string>
#include <vector>
#include <map>
#include <set>

#if defined(VR_COLLISION_DETECTION_PQP)
#include "PQP/CollisionModelPQP.h"
#else
#include "Dummy/CollisionModelDummy.h"
#endif

namespace VirtualRobot
{
#if defined(VR_COLLISION_DETECTION_PQP)
    typedef CollisionModelPQP InternalCollisionModel;
#else
    typedef CollisionModelDummy InternalCollisionModel;
#endif
    typedef std::shared_ptr< InternalCollisionModel > InternalCollisionModelPtr;
    class CollisionChecker;

    /*!
        An abstract representation of an object that can be used for collision queries.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /*!
            Standard Constructor
            If collision checks should be done in parallel, different CollisionCheckers can be specified.
            \param visu A visualization that is used for creating an internal triangle based representation of the object.
            \param name The name of this object.
            \param colChecker If not specified, the global singleton instance is used. Only useful, when parallel collision checks should be performed.
            \param id A user id.
        */
        CollisionModel(const VisualizationNodePtr &visu, const std::string& name = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr(), int id = 666, float margin = 0.0f);
        /*!Standard Destructor
        */
        virtual ~CollisionModel();


        /*! Returns name of this model.
        */
        std::string getName();

        /*!
            Return bounding box of this object.
            \param global If set, the boundignBox is transformed to globalCoordinate system. Otherwise the local BBox is returned.
        */
        BoundingBox getBoundingBox(bool global = true);


        /*!
        The global pose defines the position of the joint in the world. This value is used for visualization.
        */
        inline Eigen::Matrix4f getGlobalPose()
        {
            return globalPose;
        }
        virtual void setGlobalPose(const Eigen::Matrix4f& pose);

        CollisionCheckerPtr getCollisionChecker()
        {
            return colChecker;
        }

#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr< CollisionModelPQP > getCollisionModelImplementation()
        {
            return collisionModelImplementation;
        }
#else
        std::shared_ptr< CollisionModelDummy > getCollisionModelImplementation()
        {
            return collisionModelImplementation;
        }
#endif


        CollisionModelPtr clone(const CollisionCheckerPtr &colChecker = CollisionCheckerPtr(), float scaling = 1.0f, bool deepVisuCopy = true);

        void setVisualization(const VisualizationNodePtr visu);

        int getId();

        /*!
            Enables/Disables the visualization updates of the collision model.
            This just handles the visualization, the collision data is not affected.
        */
        void setUpdateVisualization(bool enable);
        bool getUpdateVisualizationStatus();

        VisualizationNodePtr getVisualization();
        VisualizationNodePtr getModelDataVisualization();

        //! get number of faces (i.e. triangles) of this object
        virtual int getNumFaces();

        /*!
            Print information about this model.
        */
        virtual void print();

        /*!
            Return a vector with all vertices of this object at the current global pose.
        */
        std::vector< Eigen::Vector3f > getModelVeticesGlobal();

        TriMeshModelPtr getTriMeshModel()
        {
            return collisionModelImplementation->getTriMeshModel();
        }

        std::string toXML(const std::string& basePath, int tabs);
        std::string toXML(const std::string& basePath, const std::string& filename, int tabs);

        /*!
            Create a united collision model.
            All triangle data is copied to one model which usually improves the collision detection performance.
        */
        static CollisionModelPtr CreateUnitedCollisionModel(const std::vector<CollisionModelPtr>& colModels);


        /*!
            Saves model file to model path.
            \param modelPath The directory.
            \param filename The new filename without path.
        */
        virtual bool saveModel(const std::string& modelPath, const std::string& filename);

        virtual void scale(Eigen::Vector3f& scaleFactor);

        float getMargin() const;

        /**
         * @brief in/deflates the model. If value is <0 the model is deflated.
         * @param margin Each vertex of the model is moved about this margin along its normal.
         */
        void inflateModel(float margin);

    protected:
        // internal constructor needed for flat copy of internal collision model
        CollisionModel(const VisualizationNodePtr &visu, const std::string& name, CollisionCheckerPtr colChecker, int id, InternalCollisionModelPtr collisionModel);

        //! delete all data
        void destroyData();
        VisualizationNodePtr visualization;         // this is the modified visualization
        VisualizationNodePtr origVisualization;         // this is the original visualization
        VisualizationNodePtr modelVisualization;    // this is the visualization of the trimeshmodel
        bool updateVisualization;
        TriMeshModelPtr model;
        float margin;                        // inflates the model with this margin (in mm)
        BoundingBox bbox;

        //! the name
        std::string name;

        int id;

        CollisionCheckerPtr colChecker;

        Eigen::Matrix4f globalPose;     //< The transformation that is used for visualization and for updating the col model


#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr< CollisionModelPQP > collisionModelImplementation;
#else
        std::shared_ptr< CollisionModelDummy > collisionModelImplementation;
#endif
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CollisionModel_h_
