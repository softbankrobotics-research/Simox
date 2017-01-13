
#include "OrientedWorkspaceGrid.h"
#include <iostream>
#include <algorithm>
using namespace std;

#define MIN_VALUES_STORE_GRASPS 100

namespace VirtualRobot {

OrientedWorkspaceGrid::OrientedWorkspaceGrid( float fMinX, float fMaxX, float fMinY, float fMaxY, float fDiscretizeSize, float fDiscretizeSizeAlpha, bool lazyMode, float height )
{
    this->lazyMode = lazyMode;
	this->height = height;
	minX = fMinX;
	maxX = fMaxX;
	minY = fMinY;
	maxY = fMaxY;
	discretizeSize = fDiscretizeSize;
	discretizeSizeAlpha = fDiscretizeSizeAlpha;
	data = NULL;

	if (fMinX>=fMaxX || fMinY>=fMaxY)
	{
		THROW_VR_EXCEPTION ("ERROR min >= max");
	}

	gridExtendX = maxX - minX;
	gridExtendY = maxY - minY;
	gridExtendAlpha = float(2.0*M_PI);
	gridSizeX = (int)((maxX - minX) / discretizeSize) + 1;
	gridSizeY = (int)((maxY - minY) / discretizeSize) + 1;
	gridSizeAlpha = (int)((2.0 * M_PI) / discretizeSizeAlpha) + 1;


	if (gridSizeY<=0 || gridSizeX<=0 || gridExtendX<=0 || gridExtendY<=0)
	{
		THROW_VR_EXCEPTION ("ERROR negative grid size");
	}

	VR_INFO << ": creating grid with " << gridSizeX << "x" << gridSizeY << "x" << gridSizeAlpha << " = " << gridSizeX*gridSizeY*gridSizeAlpha << " entries" << endl;
	data = new int[gridSizeX*gridSizeY*gridSizeAlpha];
	graspLink = new std::vector<GraspPtr>[gridSizeX*gridSizeY*gridSizeAlpha];
    int v = 0;
    if (lazyMode)
        v = -1;
	memset(data,v,sizeof(int)*gridSizeX*gridSizeY*gridSizeAlpha);
	maxEntry = 0;
	cellsFilled = 0;
}

OrientedWorkspaceGrid::~OrientedWorkspaceGrid()
{
	delete []data;
	delete []graspLink;
}

void OrientedWorkspaceGrid::reset()
{
	delete []graspLink;
	graspLink = new std::vector<GraspPtr>[gridSizeX*gridSizeY*gridSizeAlpha];
    int v = 0;
    if (lazyMode)
        v = -1;
	memset(data,v,sizeof(int)*gridSizeX*gridSizeY*gridSizeAlpha);
	maxEntry = 0;
	cellsFilled = 0;
}

int OrientedWorkspaceGrid::getEntry( float x, float y, float alpha )
{
	if (!data)
		return 0;

	int nPosX,nPosY,nPosAlpha;
	if (!getGridPos(x,y,alpha,nPosX,nPosY,nPosAlpha))
		return 0;
    int v = data[getDataPos(nPosX,nPosY,nPosAlpha)];
    if (lazyMode && v==-1)
    {
        updateEntry(x,y,alpha);
        v = data[getDataPos(nPosX,nPosY,nPosAlpha)];
        if (v==-1) // skip rounding errors
            return 0;
    }
    return v;
}

bool OrientedWorkspaceGrid::getEntry( float x, float y, float alpha, int &storeEntry, GraspPtr &storeGrasp )
{
	if (!data)
		return false;

	int nPosX,nPosY,nPosAlpha;
	if (!getGridPos(x,y,alpha,nPosX,nPosY,nPosAlpha))
		return false;

	storeEntry = data[getDataPos(nPosX,nPosY,nPosAlpha)];
    if (lazyMode && storeEntry==-1)
    {
        updateEntry(x,y,alpha);
        storeEntry = data[getDataPos(nPosX,nPosY,nPosAlpha)];
        if (storeEntry==-1) // skip rounding errors
            return false;
    }


	int nLinks = graspLink[getDataPos(nPosX,nPosY,nPosAlpha)].size();
	if (nLinks>0)
		storeGrasp = graspLink[getDataPos(nPosX,nPosY,nPosAlpha)][rand()%nLinks]; // get random grasp
	else
		storeGrasp.reset();
	return true;
}

bool OrientedWorkspaceGrid::getGridPos(float x, float y, float alpha, int &nPosX, int &nPosY, int &nPosAlpha)
{
	nPosX = (int)(((x-minX) / gridExtendX) * gridSizeX);
	nPosY = (int)(((y-minY) / gridExtendY) * gridSizeY);
	// adjust alpha to [0..2*PI]
	while (alpha<0)
		alpha += float(2.0*M_PI);
	while (alpha>float(2.0*M_PI))
		alpha -= float(2.0*M_PI);
	nPosAlpha = (int)(((alpha) / gridExtendAlpha) * gridSizeAlpha);

	if (nPosX<0 || nPosY<0 || nPosX>=gridSizeX || nPosY>=gridSizeY || nPosAlpha>=gridSizeAlpha || nPosAlpha<0)
	{
		//cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << endl;
		return false;
	}
	return true;
}
bool OrientedWorkspaceGrid::getEntry( float x, float y, float alpha, int &storeEntry, std::vector<GraspPtr> &storeGrasps )
{
	if (!data)
		return false;

	int nPosX,nPosY,nPosAlpha;
	if (!getGridPos(x,y,alpha,nPosX,nPosY,nPosAlpha))
		return false;

	storeEntry = data[getDataPos(nPosX,nPosY,nPosAlpha)];
    if (lazyMode && storeEntry==-1)
    {
        updateEntry(x,y,alpha);
        storeEntry = data[getDataPos(nPosX,nPosY,nPosAlpha)];
        if (storeEntry==-1) // skip rounding errors
            return false;
    }
	storeGrasps = graspLink[getDataPos(nPosX,nPosY,nPosAlpha)];
	return true;
}

int OrientedWorkspaceGrid::getCellEntry( int cellX, int cellY, int cellAlpha )
{
	if (!data)
		return 0;

	if (cellX<0 || cellY<0 || cellX>=gridSizeX || cellY>=gridSizeY || cellAlpha>=gridSizeAlpha || cellAlpha<0)
	{
		//cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
		return 0;
	}
	return data[getDataPos(cellX,cellY,cellAlpha)];
}

bool OrientedWorkspaceGrid::getCellEntry( int cellX, int cellY, int cellAlpha, int &storeEntry, GraspPtr &storeGrasp )
{
	if (!data)
		return false;

	if (cellX<0 || cellY<0 || cellX>=gridSizeX || cellY>=gridSizeY || cellAlpha>=gridSizeAlpha || cellAlpha<0)
	{
		//cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
		storeEntry = -1;
		storeGrasp.reset();
		return false;
	}
	storeEntry = data[getDataPos(cellX,cellY,cellAlpha)];
	int nLinks = graspLink[getDataPos(cellX,cellY,cellAlpha)].size();
	if (nLinks>0)
		storeGrasp = graspLink[getDataPos(cellX,cellY,cellAlpha)][rand()%nLinks];
	else
		storeGrasp.reset();
	return true;
}

bool OrientedWorkspaceGrid::getCellEntry( int cellX, int cellY, int cellAlpha, int &storeEntry, std::vector<GraspPtr> &storeGrasps )
{
	if (!data)
		return false;

	if (cellX<0 || cellY<0 || cellX>=gridSizeX || cellY>=gridSizeY || cellAlpha>=gridSizeAlpha || cellAlpha<0)
	{
		//cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << endl;
		storeEntry = -1;
		storeGrasps.clear();
		return false;
	}
	storeEntry = data[getDataPos(cellX,cellY,cellAlpha)];
	storeGrasps = graspLink[getDataPos(cellX,cellY,cellAlpha)];
	return true;
}

void OrientedWorkspaceGrid::setEntry( float x, float y, float alpha, int value, GraspPtr grasp )
{
	if (!data)
		return;

	int nPosX,nPosY,nPosAlpha;
	if (!getGridPos(x,y,alpha,nPosX,nPosY,nPosAlpha))
		return;
	setCellEntry(nPosX,nPosY,nPosAlpha,value,grasp);
}

void OrientedWorkspaceGrid::setCellEntry( int cellX, int cellY, int cellAlpha, int value, GraspPtr grasp )
{
	if (!data)
		return;

	if (cellX<0 || cellY<0 || cellX>=gridSizeX || cellY>=gridSizeY || cellAlpha>=gridSizeAlpha || cellAlpha<0)
	{
		//cout << __PRETTY_FUNCTION__ << " internal error: " << nPosX << "," << nPosY << endl;
		return;
	}
	int dataPos = getDataPos(cellX,cellY,cellAlpha);
	if (data[dataPos] <= value)
	{
		if (data[dataPos]<=0)
			cellsFilled++;
		if (value>maxEntry)
			maxEntry = value;
		data[dataPos] = value;
		if (find(graspLink[dataPos].begin(),graspLink[dataPos].end(),grasp) == graspLink[dataPos].end())
			graspLink[dataPos].push_back(grasp);
	} else if (value>=MIN_VALUES_STORE_GRASPS)
	{
		if (find(graspLink[dataPos].begin(),graspLink[dataPos].end(),grasp) == graspLink[dataPos].end())
			graspLink[dataPos].push_back(grasp);
	}
}

void OrientedWorkspaceGrid::setEntryCheckNeighbors( float x, float y, float alpha, int value, GraspPtr grasp )
{
	if (!data)
		return;

	int nPosX,nPosY,nPosAlpha;
	if (!getGridPos(x,y,alpha,nPosX,nPosY,nPosAlpha))
		return;


	setCellEntry(nPosX,nPosY,nPosAlpha,value,grasp);

	int minX = nPosX-1;
	int minY = nPosY-1;
	int minA = nPosAlpha-1;
	int maxX = nPosX+1;
	int maxY = nPosY+1;
	int maxA = nPosAlpha+1;
	if (minX<0)
		minX = 0;
	if (minY<0)
		minY = 0;
	if (minA<0)
		minA = 0;
	if (maxX>gridSizeX-1)
		maxX = gridSizeX-1;
	if (maxY>gridSizeY-1)
		maxY = gridSizeY-1;
	if (maxA>gridSizeAlpha-1)
		maxA = gridSizeAlpha-1;
	
	for (int a=minX;a<=maxX;a++)
		for (int b=minY;b<=maxY;b++)
			for (int c=minA;c<=maxA;c++)
				setCellEntry(a,b,c,value,grasp);
}

void OrientedWorkspaceGrid::setEntries( std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> &wsData, Eigen::Matrix4f &graspGlobal, GraspPtr grasp )
{
	if (!data)
		return;

	cout << "TODO!!!!" << endl;
	/*
	float x,y;

	for (int i=0;i<(int)wsData.size();i++)
	{
		Eigen::Matrix4f tmpPos2 = graspGlobal * wsData[i]->transformation.inverse();
		x = tmpPos2(0,3);
		y = tmpPos2(1,3);

		setEntryCheckNeighbors(x,y,wsData[i]->value,grasp);
		//setEntry(x,y,vData[i].value);
	}*/
}

int OrientedWorkspaceGrid::getMaxEntry()
{
	return maxEntry;
	/*
	if (!data)
		return 0;

	int nMax = 0;

	for (int i=0;i<gridSizeX;i++)
		for (int j=0;j<gridSizeY;j++)
			for (int k=0;k<gridSizeAlpha;k++)
			if (data[getDataPos(i,j,k)] > nMax)
				nMax = data[getDataPos(i,j,k)];
	return nMax;*/
}


bool OrientedWorkspaceGrid::getRandomPos( int minEntry, float &storeXGlobal, float &storeYGlobal, float &storeAlphaGlobal, int &storeEntry, GraspPtr &storeGrasp, int maxLoops /*= 50*/ )
{
	if (!data)
		return false;

	int nLoop = 0;
	int nEntry = 0;
	//int x,y,alpha;
    float x,y,alpha;
    const float rnd_mult = 1.0f / RAND_MAX;
	do
	{
		//x = rand()%gridSizeX;
		//y = rand()%gridSizeY;
		//alpha = rand()%gridSizeAlpha;
        x = minX + float(rand()) * rnd_mult * gridExtendX;
        y = minY + float(rand()) * rnd_mult * gridExtendY;
        alpha = float(rand()) * rnd_mult * 2.0f*float(M_PI);

		if (getEntry(x,y,alpha,nEntry,storeGrasp))
			if (nEntry>=minEntry)
			{
				storeXGlobal = x;//minX + ((float)x+0.5f)*discretizeSize;
				storeYGlobal = y;//minY + ((float)y+0.5f)*discretizeSize;
				storeAlphaGlobal = alpha;//((float)alpha+0.5f)*discretizeSizeAlpha;
				storeEntry = nEntry;
				return true;
			}
		nLoop++;
	} while (nLoop<maxLoops);
	return false;
}


bool OrientedWorkspaceGrid::getRandomPos( int minEntry, float &storeXGlobal, float &storeYGlobal, float &storeAlphaGlobal, int &storeEntry, std::vector<GraspPtr> &storeGrasps, int maxLoops /*= 50*/ )
{
	if (!data)
		return false;

	int nLoop = 0;
	int nEntry = 0;
	//int x,y,alpha;
    float x,y,alpha;
    const float rnd_mult = 1.0f / RAND_MAX;
	do
	{
        //x = rand()%gridSizeX;
        //y = rand()%gridSizeY;
        //alpha = rand()%gridSizeAlpha;
        x = minX + float(rand()) * rnd_mult * gridExtendX;
        y = minY + float(rand()) * rnd_mult * gridExtendY;
        alpha = float(rand()) * rnd_mult * 2.0f*float(M_PI);

		if (getEntry(x,y,alpha,nEntry,storeGrasps))
			if (nEntry>=minEntry)
			{
				storeXGlobal = x;//minX + ((float)x+0.5f)*discretizeSize;
				storeYGlobal = y;//minY + ((float)y+0.5f)*discretizeSize;
				storeAlphaGlobal = alpha;//((float)alpha+0.5f)*discretizeSizeAlpha;
				storeEntry = nEntry;
				return true;
			}
		nLoop++;
	} while (nLoop<maxLoops);
	return false;
}

void OrientedWorkspaceGrid::setGridPosition( float x, float y )
{
	minX = x-gridExtendX/2.0f;
	maxX = x+gridExtendX/2.0f;
	minY = y-gridExtendY/2.0f;
	maxY = y+gridExtendY/2.0f;
}
/*
bool OrientedWorkspaceGrid::fillGridData( WorkspaceRepresentationPtr ws, ManipulationObjectPtr o, GraspPtr g, RobotNodePtr baseRobotNode )
{
	if (!ws || !o || !g)
		return false;

	VR_ERROR << "TODO..." << endl;
	return false;
	
	Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(o->getGlobalPose());

	WorkspaceRepresentation::WorkspaceCut2DPtr cutXY = ws->createCut(graspGlobal,discretizeSize);

	std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> transformations = ws->createCutTransformations(cutXY,baseRobotNode);
	setEntries(transformations,graspGlobal,g);
	return true;
}*/

void OrientedWorkspaceGrid::getExtends( float &storeMinX,float &storeMaxX,float &storeMinY,float &storeMaxY )
{
	storeMinX = minX;
	storeMaxX = maxX;
	storeMinY = minY;
	storeMaxY = maxY;
}

void OrientedWorkspaceGrid::getCells( int &storeCellsX,int &storeCellsY,int &storeCellsAlpha )
{
	storeCellsX = gridSizeX;
	storeCellsY = gridSizeY;
	storeCellsAlpha = gridSizeAlpha;
}

bool OrientedWorkspaceGrid::getMaxCellEntry( int cellX, int cellY, float &storeRot, int &nStoreEntry, std::vector<VirtualRobot::GraspPtr> &storeGrasps )
{
	int maxEntry = -1;
	std::vector<VirtualRobot::GraspPtr> gr;
	int tmpEntry;
	bool res = false;
	for (int i=0;i<gridSizeAlpha;i++)
	{
		gr.clear();
		if (getCellEntry(cellX,cellY,i,tmpEntry,gr))
		{
			if (tmpEntry>maxEntry)
			{
				storeRot = float(i) / float(gridSizeAlpha) * gridExtendAlpha;
				maxEntry = tmpEntry;
				nStoreEntry = tmpEntry;
				storeGrasps = gr;
				res = true;
			}

		}
	}

	return res;
}

bool OrientedWorkspaceGrid::getCellEntries( int cellX, int cellY, std::vector<float> &storeRotations, std::vector<int> &storeEntries, std::vector< std::vector<VirtualRobot::GraspPtr> > &storeGrasps )
{
	int maxEntry = -1;
	std::vector<VirtualRobot::GraspPtr> gr;
	int tmpEntry;
	bool res = false;
	for (int i=0;i<gridSizeAlpha;i++)
	{
		gr.clear();
		if (getCellEntry(cellX,cellY,i,tmpEntry,gr))
		{
			if (tmpEntry>0)
			{
				storeRotations.push_back( float(i) / float(gridSizeAlpha) * gridExtendAlpha);
				storeEntries.push_back(tmpEntry);
				storeGrasps.push_back(gr);
				res = true;
			}

		}
	}

	return res;
}

void OrientedWorkspaceGrid::clear()
{
	reset();
}

bool OrientedWorkspaceGrid::fillData( const Eigen::Matrix4f &globalPose, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp )
{
	std::vector< Eigen::Matrix4f > trajectory;
	trajectory.push_back(globalPose);
	return fillData(trajectory,invReach,grasp);
}


bool OrientedWorkspaceGrid::getPlaneExtends(InverseReachabilityPtr invReach, const std::vector< Eigen::Matrix4f > &poses, float &minX, float &maxX, float &minY, float &maxY)
{
	VR_ASSERT(invReach);
	VR_ASSERT(poses.size()>0);
	float minX2 = FLT_MAX;
	float maxX2 = -FLT_MAX;
	float minY2 = FLT_MAX;
	float maxY2 = -FLT_MAX;
	minX = -FLT_MAX;
	maxX = FLT_MAX;
	minY = -FLT_MAX;
	maxY = FLT_MAX;
	for (size_t i=0;i<poses.size();i++)
	{
		minX2 = FLT_MAX;
		maxX2 = -FLT_MAX;
		minY2 = FLT_MAX;
		maxY2 = -FLT_MAX;
		invReach->setGlobalPose(poses[i]);
		MathTools::OOBB oobb = invReach->getOOBB(true);
		std::vector<Eigen::Vector3f> resPoints;
		MathTools::Plane cutPlane(Eigen::Vector3f(0,0,height), Eigen::Vector3f(0,0,1.0f));
		if (MathTools::intersectOOBBPlane(oobb,cutPlane,resPoints) == MathTools::eNoIntersection)
		{
			VR_ERROR << "No intersection with floor/plane at height " << height << " (pose " << i << "), aborting ..." << endl ;
			return false;
		}

		for (size_t i=0;i<resPoints.size();i++)
		{
			if (resPoints[i](0)<minX2)
				minX2 = resPoints[i](0);
			if (resPoints[i](0)>maxX2)
				maxX2 = resPoints[i](0);
			if (resPoints[i](1)<minY2)
				minY2 = resPoints[i](1);
			if (resPoints[i](1)>maxY2)
				maxY2 = resPoints[i](1);
		}
		// check if this bbox is smaller
		if (minX2>minX)
			minX = minX2;
		if (minY2>minY)
			minY = minY2;
		if (maxX2<maxX)
			maxX = maxX2;
		if (maxY2<maxY)
			maxY = maxY2;
	}
	return true;
}
bool OrientedWorkspaceGrid::fillData( const std::vector< Eigen::Matrix4f > &trajectory, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp )
{
	if (trajectory.size()==0)
	{
		VR_ERROR << "No entries in trajectory..." << endl;
		return false;
	}
    lazyMode = false;
	float tr = invReach->getDiscretizeParameterTranslation()*0.5f;
	float rot = invReach->getDiscretizeParameterRotation()*0.5f;
    clock_t startTime = clock();
	clear();
    clock_t startTime2 = clock();
    int timeMS = (int)(startTime2-startTime);
    //cout << "Time clear: " << timeMS << endl;
    //setGridPosition(globalPose(0,3),globalPose(1,3));
	//invReach->setGlobalPose(globalPose);

	// get start, mid and endpose
	Eigen::Matrix4f startPose = trajectory[0];
	Eigen::Matrix4f midPose = trajectory[trajectory.size()/2];
	Eigen::Matrix4f endPose = trajectory[trajectory.size()-1];

	std::vector< Eigen::Matrix4f > points;
	points.push_back(startPose);
	points.push_back(endPose);
	if (!getPlaneExtends(invReach,points,minX,maxX,minY,maxY))
		return false;

	setGridPosition(midPose(0,3),midPose(1,3));
	minX -= tr;
	minY -= tr;
	maxX += tr;
	maxY += tr;
	cout << "oobb minX:" << minX << ", maxX: " << maxX << ", minY:" << minY << ", maxY:" << maxY << endl;


	float posrpy[6];
	posrpy[0] = 0; // x
	posrpy[1] = 0; // y
	posrpy[2] = height; // z
	posrpy[3] = 0; // ro
	posrpy[4] = 0; // pi
	posrpy[5] = 0; // ya


    clock_t startTime3 = clock();

	for (float x=minX;x<=maxX;x+=tr)
	{
		posrpy[0] = x;
		for (float y=minY;y<=maxY;y+=tr)
		{
			posrpy[1] = y;
			for (float alpha=float(-M_PI);alpha<=float(M_PI);alpha+=rot)
			{
				posrpy[5] = alpha;
				Eigen::Matrix4f m;
				invReach->vector2Matrix(posrpy,m);
				//MathTools::posrpy2eigen4f(posrpy,m);

				// get min entry
				unsigned char e = 255;
				for (size_t i=0;i<trajectory.size();i++)
				{
					invReach->setGlobalPose(trajectory[i]);
					unsigned char e2 = invReach->getEntry(m);
					if (e2<e)
						e = e2;
					if (e==0)
						break;
				}
				if (e>0)
				{
					setEntry(x,y,alpha,e,grasp);
					//reachGrid->setEntryCheckNeighbors(x,y,alpha,e,g);
				}


			}

		}
	}

    clock_t startTime4 = clock();
    int timeMS2 = (int)(startTime4-startTime3);
    //cout << "Time fill: " << timeMS2 << endl;

	return true;

}

bool OrientedWorkspaceGrid::fillDataLazy( const std::vector< Eigen::Matrix4f > &trajectory, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp )
{
	if (trajectory.size()==0)
	{
		VR_ERROR << "No entries in trajectory..." << endl;
		return false;
	}
	lazyMode = true;
	clock_t startTime = clock();
	clear();
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	//cout << "Time clear: " << timeMS << endl;

	// get start, mid and endpose
	Eigen::Matrix4f startPose = trajectory[0];
	Eigen::Matrix4f midPose = trajectory[trajectory.size()/2];
	Eigen::Matrix4f endPose = trajectory[trajectory.size()-1];

	setGridPosition(midPose(0,3),midPose(1,3));
	invReach->setGlobalPose(startPose);
	lazyModeInvReach = invReach;
	lazyModeGrasp = grasp;
	lazyModeTrajectory = trajectory;
	return true;
}

bool OrientedWorkspaceGrid::fillDataLazy( const Eigen::Matrix4f &globalPose, InverseReachabilityPtr invReach, VirtualRobot::GraspPtr grasp )
{
	std::vector< Eigen::Matrix4f > trajectory;
	trajectory.push_back(globalPose);
	return fillDataLazy(trajectory,invReach,grasp);
}


long OrientedWorkspaceGrid::getCellsFilled()
{
	return cellsFilled;
}

void OrientedWorkspaceGrid::print()
{
    cout << "****** OrientedWorkspaceGrid **************" << endl;
    cout << "Lazy mode:";
    if (lazyMode)
        cout << "yes" << endl;
    else
        cout << "no" << endl;
    cout << "Cells: " << gridSizeX << "x" << gridSizeY << "x" << gridSizeAlpha << "=" << gridSizeX*gridSizeY*gridSizeAlpha << endl;
    cout << "Workspace extends:" << endl;
    cout << " x: " << minX << " to " << maxX << endl;
    cout << " y: " << minY << " to " << maxY << endl;
	cout << " height: " << height << endl;
    cout << "Discretize value  translation:" << discretizeSize << ", orientation:" << discretizeSizeAlpha << endl;
    cout << "Cells filled:" << cellsFilled << endl;
    cout << "Max entry:" << maxEntry << endl;
}

bool OrientedWorkspaceGrid::updateEntry( float x, float y, float alpha )
{
    if (!lazyMode || !lazyModeInvReach || lazyModeTrajectory.size()==0)
        return false;

    float posrpy[6];
    posrpy[0] = x; // x
    posrpy[1] = y; // y
    posrpy[2] = height; // z
    posrpy[3] = 0; // ro
    posrpy[4] = 0; // pi
    posrpy[5] = alpha; // ya

    Eigen::Matrix4f m;
	if (lazyModeTrajectory.size()==1)
	{
		// one pose, invReach is already at correct pose
		lazyModeInvReach->vector2Matrix(posrpy,m); // roll and pitch are zero -> RPY == eulerXYZ
		unsigned char e = lazyModeInvReach->getEntry(m);
		setEntry(x,y,alpha,e,lazyModeGrasp);
	} else
	{
		// trajectory mode -> min entry
		unsigned char e = 255;
		for (size_t i=0;i<lazyModeTrajectory.size();i++)
		{
			lazyModeInvReach->setGlobalPose(lazyModeTrajectory[i]);
			lazyModeInvReach->vector2Matrix(posrpy,m);
			unsigned char e2 = lazyModeInvReach->getEntry(m);
			if (e2<e)
				e = e2;
		}
		setEntry(x,y,alpha,e,lazyModeGrasp);
	}
    return true;
}

float OrientedWorkspaceGrid::getHeight()
{
	return height;
}

float OrientedWorkspaceGrid::getDiscretizeParameterTranslation()
{
	return discretizeSize;
}

float OrientedWorkspaceGrid::getDiscretizeParameterOrientation()
{
	return discretizeSizeAlpha;
}


} //  namespace

