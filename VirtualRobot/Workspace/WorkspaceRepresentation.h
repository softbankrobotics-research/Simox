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
* @author     Peter Kaiser, Nikolaus Vahrenkamp
* @copyright  2011 Peter Kaiser, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_WorkspaceRepresentation_h_
#define _VirtualRobot_WorkspaceRepresentation_h_

#include "../Model/Model.h"
#include "WorkspaceData.h"
#include "WorkspaceDataArray.h"
#include "../Tools/MathTools.h"
#include "../XML/FileIO.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{

    /*!
            This class represents a voxelized approximation of the workspace that is covered by a kinematic chain of a robot.
            The voxel grid covers the 6d Cartesian space: xyz translations (mm) and Tait-Bryan angles (eulerXYZ, fixed frame, extrinsic) orientations.
            Older versions (<=2.5) used RPY (intrinsic) for storing orientations, but it turned out that this representation is not suitable for discretization.
            Each voxels holds a counter (uchar) that holds information, e.g. about reachability.
            The discretized data can be written to and loaded from binary files.

            The data is linked to a base coordinate system which is defined by a robot joint.
            This base system is used to align the data when the robot is moving.
            I.E. think of an arm of a humanoid where the workspace representation is linked to the shoulder.
            When the torso moves, the data representation also changes it's position according to the position of the shoulder.
    */

    class VIRTUAL_ROBOT_IMPORT_EXPORT WorkspaceRepresentation
    {
    public:
        friend class CoinVisualizationFactory;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef int32_t ioIntTypeWrite;
        typedef int32_t ioIntTypeRead;

        enum eOrientationType
        {
            RPY,
            EulerXYZ,           // intrinsic
            EulerXYZExtrinsic,  // fixed frame (standard)
            Hopf                // hopf coordinates
        };

        struct VolumeInfo
        {
            unsigned int voxelCount3D;              // overall number of 3d voxels
            unsigned int filledVoxelCount3D;        // number of filled voxels (nr of 3d position voxels for which at least one 6D cell exists that is filled)
            unsigned int borderVoxelCount3D;        // number of border voxels (3d voxels with at least 2 emoty neighbors in 3d)
            float volumeVoxel3D;                    // volume of one voxel (m^3)
            float volumeFilledVoxels3D;             // accumulated volume of all filled voxels (m^3)
            float volume3D;                         // (filledVoxelCount3D - 0.5*borderVoxelCount3D) * volumeVoxel3D
        };


        WorkspaceRepresentation(ModelPtr robot);

        /*!
            Reset all data.
        */
        virtual void reset();

        /*!
            Load the workspace data from a binary file.
            Exceptions are thrown on case errors are detected.
        */
        virtual void load(const std::string& filename);

        /*!
            Store the workspace data to a binary file.
            Exceptions are thrown on case errors are detected.
        */
        virtual void save(const std::string& filename);

        /*!
            Return corresponding entry of workspace data
        */
        unsigned char getEntry(const Eigen::Matrix4f& globalPose) const;

        //! Returns the maximum entry of a voxel.
        int getMaxEntry() const;

        //! returns the extends of a voxel at corresponding dimension.
        float getVoxelSize(int dim) const;

        //! The base node of this workspace data
        RobotNodePtr getBaseNode();

        //! The corresponding TCP
        FramePtr getTCP();

        //! The kinematic chain that is covered by this workspace data
        JointSetPtr getJointSet();

        /*!
            Initialize and reset all data.
            \param nodeSet The robot node set that should be considered for workspace (e.g. reachability) analysis.
            \param discretizeStepTranslation The extend of a voxel dimension in translational dimensions (x,y,z) [mm]
            \param discretizeStepRotation The extend of a voxel dimension in rotational dimensions (roll, pitch, yaw) [rad]
            \param minBounds The minimum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system [mm and rad]
            \param maxBounds The maximum workspace poses (x,y,z,ro,pi,ya) given in baseNode's coordinate system [mm and rad]
            \param staticCollisionModel The static collision model of the robot. This model does not move when changing the configuration of the RobotNodeSet. If not set no collisions will be checked when building the reachability data
            \param dynamicCollisionModel The dynamic collision model of the robot. This model does move when changing the configuration of the RobotNodeSet. If not set no collisions will be checked when building the reachability data.
            \param baseNode Perform the computations in the coordinate system of this node. If not set, the global pose is used (be careful, when the robot moves around global poses may not be meaningful!)
            \param tcpNode If given, the pose of this node is used for workspace calculations. If not given, the TCP node of the nodeSet is used.
            \param adjustOnOverflow If set, the 8bit data is divided by 2 when one voxel entry exceeds 255. Otherwise the entries remain at 255.
        */
        virtual void initialize(JointSetPtr nodeSet,
            float discretizeStepTranslation,
            float discretizeStepRotation,
            float minBounds[6],
            float maxBounds[6],
            LinkSetPtr staticCollisionModel = LinkSetPtr(),
            LinkSetPtr dynamicCollisionModel = LinkSetPtr(),
            ModelNodePtr baseNode = ModelNodePtr(),
            FramePtr tcpNode = FramePtr(),
            bool adjustOnOverflow = true);

        /*!
            Sets entry that corresponds to TCP pose to e, if current entry is lower than e.
            Therefore the corresponding voxel of the current TCP pose is determined and its entry is adjusted.
        */
        virtual void setCurrentTCPPoseEntryIfLower(unsigned char e);

        /*!
            Sets entry that corresponds to TCP pose to e.
            Therefore the corresponding voxel of the current TCP pose is determined and its entry is set.
        */
        virtual void setCurrentTCPPoseEntry(unsigned char e);

        /*
            Add pose to data.
            This means that the entry of the corresponding WorkspaceData voxel is increased by 1.
        */
        virtual void addPose(const Eigen::Matrix4f& globalPose);

        /*!
         * \brief addPose Adds pose and rate it with the given quality measure. Ignored in this implementation, but used in derived classes (@see Manipulability)
         * \param p
         * \param qualMeasure
         */
        virtual void addPose(const Eigen::Matrix4f& p, PoseQualityMeasurementPtr qualMeasure);

        /*!
            Clears all data
        */
        virtual void clear();

        /*!
            Generate a random configuration for the robot node set. This configuration is within the joint limits of the current robot node set.
            \param nodeSet The joints. If not given, the standard nodeSet is used.
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        virtual bool setRobotNodesToRandomConfig(JointSetPtr nodeSet = VirtualRobot::JointSetPtr(), bool checkForSelfCollisions = true);

        /*!
            Cut all data >1 to 1. This reduces the file size when saving compressed data.
        */
        virtual void binarize();

        /*!
            Checks for all voxels with entry==0 if there are neighbors with entries>0.
            If so the entry is set to the averaged value of the neighbors
            \param minNeighbors The minimum number of neighbors that have to have an entry>0
            \return The number of changed voxels.
        */
        virtual int fillHoles(unsigned int minNeighbors = 1);

        /*!
        Print status information
        */
        virtual void print();

        //! returns a random pose that is covered by the workspace data
        Eigen::Matrix4f sampleCoveredPose();

        /*!
            Returns true, if the corresponding voxel entry is not zero.
        */
        bool isCovered(const Eigen::Matrix4f& globalPose);

        /*!
            Returns true, if voxel entry is not zero.
        */
        bool isCovered(unsigned int v[6]);

        virtual int getNumVoxels(int dim) const;
        virtual float getMinBound(int dim) const;
        virtual float getMaxBound(int dim) const;

        /*!
            get entry of given voxel.
        */
        virtual unsigned char getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const;

        /*!
            Sums all angle (x3,x4,x5) entries for the given position.
        */
        virtual int sumAngleReachabilities(int x0, int x1, int x2) const;
        virtual int avgAngleReachabilities(int x0, int x1, int x2) const;

        /*!
            Searches all angle entries (x3,x4,x5) for maximum entry.
            (x0,x1,x2) is the voxel position.
        */
        virtual int getMaxEntry(int x0, int x1, int x2) const;

        /*!
            Returns the maximum workspace entry that can be achieved by an arbitrary orientation at the given position.
        */
        virtual int getMaxEntry(const Eigen::Vector3f& position_global) const;

        /*!
            Get the corresponding voxel.
            If false is returned the pose is outside the covered workspace.
        */
        virtual bool getVoxelFromPose(const Eigen::Matrix4f& globalPose, unsigned int v[6]) const;

        /*!
            Get the corresponding voxel coordinates.
            If false is returned the position is outside the covered workspace.
        */
        virtual bool getVoxelFromPosition(const Eigen::Matrix4f& globalPose, unsigned int v[3]) const;
        
        /*!
            Computes center of corresponding voxel in global coord system.
        */
        Eigen::Matrix4f getPoseFromVoxel(unsigned int v[6], bool transformToGlobalPose = true);
        Eigen::Matrix4f getPoseFromVoxel(float v[6], bool transformToGlobalPose = true);

        /*!
            Returns the maximum that can be achieved by calling sumAngleReachabilities()
        */
        virtual int getMaxSummedAngleReachablity() const;

        /*!
            Returns true if for the given 3d position is at least one entry >0
            \param x Voxel position x0
            \param y Voxel position x1
            \param z Voxel position x2
        */
        virtual bool hasEntry(unsigned int x, unsigned int y, unsigned int z);

        /*!
            Estimate a parameter setup for the given RNS by randomly set configurations and check for achieved workspace extends. The results are slightly scaled.
            \param ndoeSet
            \param steps How many loops should be performed to estimate the result. Chose a value >= 1000.
            \param storeMinBounds Workspace extend from min
            \param storeMaxBounds Workspace extend to max
            \param baseNode
            \param tcpNode
            \return True on success.
        */
        virtual bool checkForParameters(JointSetPtr nodeSet,
										float steps,
										float storeMinBounds[6],
										float storeMaxBounds[6],
										ModelNodePtr baseNode = ModelNodePtr(),
										FramePtr tcpNode = FramePtr());

        /*!
            2D data that represents a cut through the workspace representation.
            Usually the z component and the orientation of the 6D workspace data is fixed and the x and y components are iterated in order to store the resulting workspace entries.
        */
        struct WorkspaceCut2D
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Eigen::Matrix4f referenceGlobalPose;
            Eigen::MatrixXi entries;
            float minBounds[2]; // in global coord system
            float maxBounds[2]; // in global coord system

            virtual VisualizationPtr getVisualization(const ColorMap& cm, const Eigen::Vector3f &normal, float maxEntry) const;
        };
        typedef std::shared_ptr<WorkspaceCut2D> WorkspaceCut2DPtr;

        struct WorkspaceCut2DTransformation
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            int value;
            Eigen::Matrix4f transformation;
        };

        typedef std::shared_ptr<WorkspaceCut2DTransformation> WorkspaceCut2DTransformationPtr;

        /*!
            Create a horizontal cut through this workspace data. Therefore, the z component and the orientation of the reference pose (in global coordinate system) is used.
            Then the x and y components are iterated and the corresponding entires are used to fill the 2d grid.
        */
        WorkspaceCut2DPtr createCut(const Eigen::Matrix4f& referencePose, float cellSize, bool sumAngles) const;

        /*!
        * \brief createCut Create a cut at a specific height (assuming z is upwards).
        * \param heightPercent Value in [0,1]
        * \param cellSize The discretization step size of the result
        * \return
        */
        WorkspaceCut2DPtr createCut(float heightPercent, float cellSize, bool sumAngles) const;

        /*!
            Build all transformations from referenceNode to cutXY data.h Only entries>0 are considered.
            If referenceNode is set, the transformations are given in the corresponding coordinate system.
        */
        std::vector<WorkspaceCut2DTransformationPtr> createCutTransformations(WorkspaceCut2DPtr cutXY, FramePtr referenceNode = FramePtr());

        /*!
            Computes the axis aligned bounding box of this object in global coordinate system.
            Note, that the bbox changes when the robot moves.
        */
        bool getWorkspaceExtends(Eigen::Vector3f& storeMinBBox, Eigen::Vector3f& storeMaxBBox) const;

        /*!
            The bounding box in global frame.
            \param achievedValues If not set the bounding box as defined on construction is returned. If set the min/max achieved positions are used.
            \return The object oriented bounding box
        */
        MathTools::OOBB getOOBB(bool achievedValues = false) const;

        float getDiscretizeParameterTranslation();
        float getDiscretizeParameterRotation();

        RobotPtr getRobot()
        {
            return robot;
        }

        LinkSetPtr getCollisionModelStatic()
        {
            return staticCollisionModel;
        }
        LinkSetPtr getCollisionModelDynamic()
        {
            return dynamicCollisionModel;
        }
        FramePtr getTcp()
        {
            return tcpNode;
        }
        bool getAdjustOnOverflow()
        {
            return adjustOnOverflow;
        }

        /*!
            Compute the quality of the current pose and add entry to voxel data
            (if is larger than the existing entry).
        */
        virtual void addCurrentTCPPose();


        /*!
            Append a number of random TCP poses to workspace Data.
            For bigger sampling rates (loops > 50.000) you should consider this method's multithreaded pendant.
            \param loops Number of poses that should be appended
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        void addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions = true);
        /*!
            Appends a number of random TCP poses to workspace Data (multithreaded).
            This method is blocking, i.e. it returns as soon as all threads are done.
            \param loops Number of poses that should be appended
            \param numThreads number of worker threads used behind the scenes to append random TCP poses to workspace data.
            \param checkForSelfCollisions Build a collision-free configuration. If true, random configs are generated until one is collision-free.
        */
        virtual void addRandomTCPPoses(unsigned int loops, unsigned int numThreads, bool checkForSelfCollisions = true);

        void setVoxelEntry(unsigned int v[6], unsigned char e);
        void setEntry(const Eigen::Matrix4f& poseGlobal, unsigned char e);
        void setEntryCheckNeighbors(const Eigen::Matrix4f& poseGlobal, unsigned char e, unsigned int neighborVoxels);

        virtual void toLocal(Eigen::Matrix4f& p) const;
        virtual void toGlobal(Eigen::Matrix4f& p) const;
        void toLocalVec(Eigen::Vector3f& positionGlobal) const;
        void toGlobalVec(Eigen::Vector3f& positionLocal) const;

        //! Convert a 4x4 matrix to a pos + ori vector
        void matrix2Vector(const Eigen::Matrix4f& m, float x[6]) const;
        void vector2Matrix(const float x[6], Eigen::Matrix4f& m) const;
        void vector2Matrix(const Eigen::Vector3f& pos, const Eigen::Vector3f& rot, Eigen::Matrix4f& m) const;
        virtual bool getVoxelFromPose(float x[6], unsigned int v[6]) const;
        virtual bool getVoxelFromPosition(float x[3], unsigned int v[3]) const;

        /*!
            Usually not needed. Don't call this method after data has been loaded or created!
        */
        void setOrientationType(eOrientationType t);

        /*!
            Creates a deep copy of this data structure. Derived classes may overwrite this method that provides a generic interface for cloning.
        */
        virtual WorkspaceRepresentationPtr clone();

        /*!
            Returns the raw data.
        */
        WorkspaceDataPtr getData();
        bool getPoseFromVoxel(unsigned int x[], float v[]) const;

        virtual VolumeInfo computeVolumeInformation();

        VisualizationPtr getVisualization(const ColorMapPtr &cm, bool transformToGlobalPose = true, float maxZGlobal = 1e10) const;
    protected:

        /*!
            Derived classes may implement some custom data access.
        */
        virtual bool customLoad(std::ifstream&)
        {
            return true;
        }
        /*!
            Derived classes may implement some custom data access.
        */
        virtual bool customSave(std::ofstream&)
        {
            return true;
        }

        virtual void customInitialize() {}
        virtual void customPrint() {}

        //! Uncompress the data
        void uncompressData(const unsigned char* source, int size, unsigned char* dest);
        //! Compress the data
        unsigned char* compressData(const unsigned char* source, int size, int& compressedSize);

        virtual Eigen::Matrix4f getToLocalTransformation() const;
        virtual Eigen::Matrix4f getToGlobalTransformation() const;

        ModelPtr robot;
        ModelNodePtr baseNode;
        FramePtr tcpNode;
        JointSetPtr nodeSet;

        LinkSetPtr staticCollisionModel;
        LinkSetPtr dynamicCollisionModel;

        // Number of processed random configs
        int buildUpLoops;

        // Number of reported collisions
        int collisionConfigs;

        // Tells how to discretize the workspace data
        float discretizeStepTranslation;
        float discretizeStepRotation;
        float minBounds[6];
        float maxBounds[6];

        // Number of voxels in each dimension
        int numVoxels[6];

        // The smallest/greatest tcp workspace pose value reached in each dimension
        float achievedMinValues[6];
        float achievedMaxValues[6];

        // workspace extend in each dimension
        float spaceSize[6];

        WorkspaceDataPtr data;

        bool adjustOnOverflow;

        std::string type;

        int versionMajor;
        int versionMinor;

        //! Specifies how the rotation part (x[3],x[4],x[5]) of an 6D voxel entry is encoded.
        eOrientationType orientationType;

    };

} // namespace VirtualRobot

#endif // _VirtualRobot_WorkspaceRepresentation_h_
