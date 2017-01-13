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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _ORIENTEDWORKSPACE_GRID_H_
#define _ORIENTEDWORKSPACE_GRID_H_

#include "../../VirtualRobot.h"
#include "InverseReachability.h"
#include <VirtualRobot/VirtualRobotCommon.h>

namespace VirtualRobot {
	
/*!
* A 2D grid which represents a quality distribution (e.g. the reachability) at 2D positions w.r.t. one or multiple grasp(s).
* Internally the inverse workspace data (@see WorkspaceRepresentation), which encodes the 
* transformation between robot's base and grasping position, is used.
* This data is useful to quickly sample positions from where the probability that a grasp is reachable is high (see \func getRandomPos).
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT OrientedWorkspaceGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  
	/*!
		Setup the 2D grid with given extends and discretization parameter.
        Initially all entries are set to -1. 
        The grid can be filled by the fillData method or on the fly when using the fillDataLazy.
		Depending of the robot's base node the height of the grid (z axis) can be specified.
	*/
	OrientedWorkspaceGrid(float minX, float maxX, float minY, float maxY, float discretizeSize, float fDiscretizeSizeAlpha, bool lazyMode = false, float height = 0.0f);
	~OrientedWorkspaceGrid();

	/*!
		Clear data and fill it according to given pose and inverse reachability.
	*/
	bool fillData(const Eigen::Matrix4f &globalPose, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp=VirtualRobot::GraspPtr());
	bool fillDataLazy(const Eigen::Matrix4f &globalPose, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp=VirtualRobot::GraspPtr());
	
	bool fillData(const std::vector< Eigen::Matrix4f > &trajectory, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp=VirtualRobot::GraspPtr());
	bool fillDataLazy(const std::vector< Eigen::Matrix4f > &trajectory, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp=VirtualRobot::GraspPtr());

	/*!
		Randomly sample a solution within the reachability grid.
	*/
	bool sampleSolution(float &storeX,float &storeY,float &storeAlpha,int &storeEntry);

	//! returns entry of position in x/y plane (in world coords). Considering the orientation alpha
	int getEntry(float x, float y, float alpha);


	/*!
		Gets the corresponding entry of a world / global 2d position.
		\param x The x coordinate (in global coordinate system)
		\param y The y coordinate (in global coordinate system)
		\param storeEntry The corresponding entry is stored.
		\param storeGrasp A random grasp of corresponding cell is stored.
		\return True if x/y is inside this grid and an entry>0 was found, false otherwise.
	*/
	bool getEntry(float x, float y, float alpha, int &storeEntry, VirtualRobot::GraspPtr &storeGrasp);
	bool getEntry(float x, float y, float alpha, int &nStoreEntry, std::vector<VirtualRobot::GraspPtr> &storeGrasps );

	//! returns entry of discretized square (x/y/alpha)
	int getCellEntry(int cellX, int cellY, int cellAlpha);
	bool getCellEntry(int cellX, int cellY, int cellAlpha, int &nStoreEntry, VirtualRobot::GraspPtr &storeGrasp);
	bool getCellEntry(int cellX, int cellY, int cellAlpha, int &nStoreEntry, std::vector<VirtualRobot::GraspPtr> &storeGrasps );
	bool getMaxCellEntry(int cellX, int cellY, float &storeRot, int &nStoreEntry, std::vector<VirtualRobot::GraspPtr> &storeGrasps );
	bool getCellEntries( int cellX, int cellY, std::vector<float> &storeRotations, std::vector<int> &storeEntries, std::vector< std::vector<VirtualRobot::GraspPtr> > &storeGrasps );

	/*!
		sets the entry to value, if the current value is lower 
	*/
	void setEntry (float x, float y, float alpha, int value, VirtualRobot::GraspPtr grasp);
	void setCellEntry( int cellX, int cellY, int cellAlpha, int value, VirtualRobot::GraspPtr pGrasp );

	
	int getMaxEntry();
	long getCellsFilled();

	/*!
		This method sets the grid value to nValue and checks if the neighbors have a lower value and in case the value of the neighbors is set to nValue.
	*/
	void setEntryCheckNeighbors( float x, float y, float alpha, int value, VirtualRobot::GraspPtr grasp );

	//! tries to find a random position with a entry >= minEntry
	bool getRandomPos(int minEntry, float &storeXGlobal, float &storeYGlobal, float &storeAlphaGlobal, int &storeEntry, VirtualRobot::GraspPtr &storeGrasp, int maxLoops = 50);
	bool getRandomPos(int minEntry, float &storeXGlobal, float &storeYGlobal, float &storeAlphaGlobal, int &storeEntry, std::vector<VirtualRobot::GraspPtr> &storeGrasps, int maxLoops = 50);
	
	/*!
		Clear all entries.
	*/
	void reset();

	/*!
		Fill the grid with inverse reachability data generated from grasp g and object o.
	*/
	//bool fillGridData(VirtualRobot::WorkspaceRepresentationPtr ws, VirtualRobot::ManipulationObjectPtr o, VirtualRobot::GraspPtr g, VirtualRobot::RobotNodePtr baseRobotNode );


	/*!
		Move the grid to (x,y), given in global coordinate system. Sets the new center.
	*/
	void setGridPosition(float x, float y);

	/*!
		Get extends in global coord system.
	*/
	void getExtends(float &storeMinX,float &storeMaxX,float &storeMinY,float &storeMaxY );


	/*!
		Number of cells in x, y and alpha 
	*/
	void getCells(int &storeCellsX,int &storeCellsY,int &storeCellsAlpha);

	/*!
		Remove all entries.
	*/
	void clear();

    void print();

	float getHeight();

	float getDiscretizeParameterTranslation();
	float getDiscretizeParameterOrientation();
	

protected:
	/*!
		Adds data stored in reachability transformations. This data defines transformations from robot base system to grasping pose,
		so when defining a grasping pose, the inverse reachability can be represented by this grid
		Therefor the "world coordinates" of the inverse reachability distributions are computed by T_grasp * ReachTransformation^-1
	*/
	void setEntries(std::vector<VirtualRobot::WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> &wsData, Eigen::Matrix4f &graspGlobal, VirtualRobot::GraspPtr grasp);

	inline int getDataPos(int x, int y, int alpha) {return (x*gridSizeY*gridSizeAlpha + y*gridSizeAlpha + alpha);};
	bool getGridPos(float x, float y, float alpha, int &nPosX, int &nPosY, int &nPosAlpha);

    //! Lazy mode update
    bool updateEntry(float x, float y, float alpha);
	bool getPlaneExtends(InverseReachabilityPtr invReach, const std::vector< Eigen::Matrix4f > &poses, float &minX, float &maxX, float &minY, float &maxY);
	float minX, maxX; // in global coord system
	float minY, maxY; // in global coord system
	float discretizeSize;
	float discretizeSizeAlpha;

	int gridSizeX,gridSizeY,gridSizeAlpha;

	float gridExtendX,gridExtendY,gridExtendAlpha;
	long cellsFilled;
	int maxEntry;

	float height;

    bool lazyMode;
    VirtualRobot::GraspPtr lazyModeGrasp;
    InverseReachabilityPtr lazyModeInvReach;
	std::vector< Eigen::Matrix4f > lazyModeTrajectory;
    //Eigen::Matrix4f lazyModeGlobalPose;


	int *data;								// stores the quality values
	std::vector<VirtualRobot::GraspPtr>* graspLink;		// points to list of all reachable grasps

};

typedef boost::shared_ptr<OrientedWorkspaceGrid> OrientedWorkspaceGridPtr;

}
#endif // _ORIENTEDWORKSPACE_GRID_H_
