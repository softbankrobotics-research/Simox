#include "InverseReachabilityCoinVisualization.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>


namespace VirtualRobot {


SoNode* InverseReachabilityCoinVisualization::getCoinVisualizationCut(WorkspaceRepresentationPtr reachSpace, const VirtualRobot::ColorMap cm, bool transformToGlobalPose)
{
	SoSeparator *res = new SoSeparator;
	res->ref();
	SoUnits* u = new SoUnits;
	u->units = SoUnits::MILLIMETERS;
	res->addChild(u);
	if(!reachSpace)
	{
		res->unrefNoDelete();
		return res;
	}
	Eigen::Vector3f size;
	size(0) = reachSpace->getVoxelSize(0)*1.05f;
	size(1) = reachSpace->getVoxelSize(1)*1.05f;
	size(2) = reachSpace->getVoxelSize(2)*1.05f;
	float minS = size(0);
	if (size(1)<minS) minS = size(1);
	if (size(2)<minS) minS = size(2);


	SoCube *cube = new SoCube;
	cube->width.setValue(size(0));
	cube->depth.setValue(size(1));
	cube->height.setValue(size(2));

	VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
	float radius = minS*0.75f;

	/*
	Eigen::Vector3f voxelPosition;
	int step = 1;
	int maxValue = 0;
	for(int a = 0; a < reachSpace->getNumVoxels(0); a+=step)
	{
		for(int b = 0; b < reachSpace->getNumVoxels(1); b+=step)
		{
			for(int c = 0; c < reachSpace->getNumVoxels(2); c+=step)
			{
				int value = reachSpace->sumAngleReachabilities(a, b, c);
				if (value>=maxValue)
					maxValue = value;
			}
		}
	}
	Eigen::Vector3f resPos;
	for(int a = 0; a < reachSpace->getNumVoxels(0); a+=step)
	{
		voxelPosition(0) = reachSpace->getMinBound(0) + (a + 0.5f)*size(0);
		for(int b = 0; b < reachSpace->getNumVoxels(1); b+=step)
		{
			voxelPosition(1) = reachSpace->getMinBound(1) + (b + 0.5f)*size(1);

			//int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
			int cSize = reachSpace->getNumVoxels(2)/2;
			for(int c = 0; c < cSize; c+=step)
			{
				voxelPosition(2) = reachSpace->getMinBound(2) + (c + 0.5f)*size(2);

				int value = reachSpace->sumAngleReachabilities(a, b, c);
				if (value>0)
				{
					resPos = voxelPosition;
					if(transformToGlobalPose)// && reachSpace->baseNode)
					{
						reachSpace->toGlobalVec(resPos);
						//voxelPosition = reachSpace->baseNode->toGlobalCoordinateSystemVec(voxelPosition);
					}
					float intensity = (float)value;
					if (maxValue>0)
						intensity /= maxValue;
					if (intensity>1.0f)
						intensity = 1.0f;
					color = cm.getColor(intensity);

					SoNode *n = CoinVisualizationFactory::CreateVertexVisualization(resPos,radius, color.transparency,color.r,color.g,color.b);
					if (n)
						res->addChild(n);
				}
			}
		}
	}*/

	VirtualRobot::MathTools::OOBB oobb = reachSpace->getOOBB(true);
	std::vector< Eigen::Vector3f > pt = oobb.getOOBBPoints();
	Eigen::Vector3f minPt = pt[0];
	Eigen::Vector3f maxPt = pt[0];
	
	for (size_t i=0; i<pt.size(); i++)
	{
		for (int j=0;j<3;j++)
		{
			if (pt[i](j) < minPt(j))
				minPt(j) = pt[i](j);
			if (pt[i](j) > maxPt(j))
				maxPt(j) = pt[i](j);
		}
	}

	float tr = reachSpace->getDiscretizeParameterTranslation();
	Eigen::Matrix4f refPose = Eigen::Matrix4f::Identity();
	reachSpace->toGlobal(refPose);


	Eigen::Matrix4f tmpPose = refPose;
	unsigned int v[6];
#ifdef TEST_2D
	float maxX = maxPt(0);
	float maxY = maxPt(1);
	float maxZ = maxPt(2);
#else
#ifdef TEST_AT_TCP
	float maxX = maxPt(0);
	float maxY = maxPt(1);
	float maxZ = maxPt(2);
#else
	float maxX = refPose(0,3);//maxPt(0);
	float maxY = refPose(1,3);//maxPt(1)/2;
	float maxZ = refPose(2,3);//maxPt(2);
#endif
#endif

	// get max value
	int maxValue = 0;
	for (float x = minPt(0); x<=maxX; x+=tr)
	{
		tmpPose(0,3) = x;
		for (float y = minPt(1); y<=maxY; y+=tr)
		{
			tmpPose(1,3) = y;
			for (float z = minPt(2); z<=maxZ; z+=tr)
			{
				tmpPose(2,3) = z;
				if (reachSpace->getVoxelFromPose(tmpPose,v))
				{
					int value = reachSpace->sumAngleReachabilities(v[0],v[1],v[2]);
					if (value>maxValue)
						maxValue = value;
				}
			}

		}
	}



	// create visu
	for (float x = minPt(0); x<=maxX; x+=tr)
	{
		tmpPose(0,3) = x;
		for (float y = minPt(1); y<=maxY; y+=tr)
		{
			tmpPose(1,3) = y;
			for (float z = minPt(2); z<=maxZ; z+=tr)
			{
				tmpPose(2,3) = z;
				if (reachSpace->getVoxelFromPose(tmpPose,v))
				{
					int value = reachSpace->sumAngleReachabilities(v[0],v[1],v[2]);
					if (value>0)
					{
						float intensity = (float)value;
						if (maxValue>0)
							intensity /= maxValue;
						if (intensity>1.0f)
							intensity = 1.0f;
						color = cm.getColor(intensity);

						SoSeparator *n = new SoSeparator;

						// Set the vertex-position
						SoTranslation *t = new SoTranslation;
						t->translation.setValue(tmpPose(0,3), tmpPose(1,3), tmpPose(2,3));

						// Set material
						SoMaterial *m = new SoMaterial;
						m->transparency.setValue(color.transparency);


						m->diffuseColor.setValue(color.r, color.g, color.b);
						m->ambientColor.setValue(color.r, color.g, color.b);

						n->addChild(t);
						n->addChild(m);
						n->addChild(cube);

						//SoNode *n = CoinVisualizationFactory::CreateVertexVisualization(tmpPose.block(0,3,3,1),radius, color.transparency,color.r,color.g,color.b);
						//if (n)

						
						res->addChild(n);
					}
				}
			}

		}
	}


	res->unrefNoDelete();
	return res;
}


SoNode* InverseReachabilityCoinVisualization::getCoinVisualization2( OrientedWorkspaceGridPtr reachGrid, InverseReachabilityPtr invReach, VirtualRobot::ColorMap cm, bool transformToGlobalPose /*= true*/ )
{
	SoSeparator *res = new SoSeparator;
	if (!reachGrid)
		return res;
	SoUnits *u = new SoUnits;
	res->addChild(u);
	u->units.setValue(SoUnits::MILLIMETERS);
	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	float minX,maxX,minY,maxY;
	float minA = -float(M_PI);
	float maxA = float(M_PI);
	reachGrid->getExtends(minX,maxX,minY,maxY);
	gp(0,3) = minX;
	gp(1,3) = minY;

	int nX,nY,nA;

	reachGrid->getCells(nX,nY,nA);

	float sizeX = (maxX - minX) / (float)nX;
	float sizeY = (maxY - minY) / (float)nY;
	float sizeA = (maxA - minA) / (float)nA;


	float ro,gr,bl;
	SoCube *cube = new SoCube();
	cube->width = sizeX;
	cube->height = sizeY;
	cube->depth = 1.0f;
	int maxEntry = reachGrid->getMaxEntry();
	if (invReach)
		maxEntry = invReach->getMaxEntry();

	if (maxEntry==0)
		maxEntry = 1;

	SoDrawStyle *ds = new SoDrawStyle;
	ds->style = SoDrawStyle::LINES;

	// back-face culling
	SoShapeHints * shapeHints = new SoShapeHints;
	//shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
	shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
	shapeHints->shapeType = SoShapeHints::SOLID;

	SoBaseColor *bc = new SoBaseColor;
	bc->rgb.setValue(0,0,0);

	// keep a solid color
	SoLightModel * lightModel = new SoLightModel;
	lightModel->model = SoLightModel::BASE_COLOR;
	float alpha;

	float xStep = 1;
	float yStep = 1;
	int nCount = 0;

	for (int x = 0; x<nX; x+=(int)xStep)
	{
		float xPos = minX + (float)x * sizeX + 0.5f*sizeX; // center of voxel
		for (int y = 0; y<nY; y+=(int)yStep)
		{
			int v;
			std::vector<GraspPtr> grasps;
			bool ok = reachGrid->getMaxCellEntry(x,y,alpha,v,grasps);
			if (ok)
			{
				float yPos = minY + (float)y * sizeY + 0.5f*sizeY; // center of voxel
				gp(0,3) = xPos;
				gp(1,3) = yPos;

				SoSeparator *sep1 = new SoSeparator();

				SoMatrixTransform* matTr = CoinVisualizationFactory::getMatrixTransform(gp);

				float intensity = (float)v;
				intensity /= maxEntry;
				if (intensity>1.0f)
					intensity = 1.0f;				
				if (intensity<0.0f)
					intensity = 0.0f;
				VirtualRobot::VisualizationFactory::Color color = cm.getColor(intensity);

				SoMaterial *mat= new SoMaterial();

				ro = color.r;
				gr = color.g;
				bl = color.b;

				mat->diffuseColor.setValue(ro,gr,bl);
				mat->ambientColor.setValue(ro,gr,bl);
				if (intensity>0)
				{
					nCount++;

					sep1->addChild(matTr);
					sep1->addChild(mat);
					sep1->addChild(cube);

					SoSeparator *pSepLines = new SoSeparator;
					sep1->addChild(pSepLines);

					pSepLines->addChild(ds);
					pSepLines->addChild(shapeHints);
					pSepLines->addChild(lightModel);
					pSepLines->addChild(bc);

					pSepLines->addChild(cube);

					res->addChild(sep1);
				}
			}
		}
	}
	cout << "Nr of cubes drawn:" << nCount << endl;
	//res->addChild(lines);
	return res;

}

// one foot, might also be a base
void InverseReachabilityCoinVisualization::buildFootPolygon(SoSeparator *visuFootSep, VirtualRobot::RobotNodePtr footNode, VirtualRobot::ManipulabilityPtr reachSpace, Eigen::Matrix4f &trafoBaseToFoot)
{
    if (!footNode)
        return;
    if (!footNode->getCollisionModel())
        return;
    float maxFloorDist = 5.0f;
    std::vector< MathTools::ContactPoint > points;
    CollisionCheckerPtr colChecker = footNode->getCollisionModel()->getCollisionChecker();
    MathTools::Plane floor = MathTools::getFloorPlane();
    colChecker->getContacts(floor, footNode->getCollisionModel(), points, maxFloorDist);
    if (points.size()<=2)
    {
        VR_ERROR << "Not enough contacts for node " << footNode->getName() << endl;
        return;
    }

    // get 2d convex hull of points
    std::vector<Eigen::Vector3f> points3d;
    std::vector<Eigen::Vector2f> points2d;
    for (size_t i=0;i<points.size();i++)
    {
        points3d.push_back(points[i].p);
        Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(points[i].p,floor);
        points2d.push_back(pt2d);
    }

    VirtualRobot::MathTools::ConvexHull2DPtr suportPolygonFloor = MathTools::createConvexHull2D(points2d);

    // convert points back to 3d
    points3d.clear();
    for (size_t u=0;u<suportPolygonFloor->vertices.size();u++)
    {
        Eigen::Vector2f pt = suportPolygonFloor->vertices[u];
        Eigen::Vector3f pt3d(pt(0),pt(1),1.0f);// 1mm above floor
        pt3d = footNode->toLocalCoordinateSystemVec(pt3d);
        points3d.push_back(pt3d);
    }


    SoSeparator* res = new SoSeparator;
    VisualizationFactory::Color colorLine = VisualizationFactory::Color::Black();
    VisualizationFactory::Color color = VisualizationFactory::Color::Gray();
    //colorLine.transparency = 0.5f;
    float lineSize = 1.0f;
    if (!reachSpace || !reachSpace->getBaseNode())
    {
        trafoBaseToFoot.setIdentity();
    } else
    {
        // store trafos
        trafoBaseToFoot = reachSpace->getBaseNode()->toLocalCoordinateSystem(footNode->getGlobalPose());
    }

    SoSeparator *lF = new SoSeparator;
    SoMatrixTransform* mtl = CoinVisualizationFactory::getMatrixTransformScaleMM2M(trafoBaseToFoot);
    lF->addChild(mtl);
    lF->addChild(CoinVisualizationFactory::CreatePolygonVisualization(points3d,color,colorLine,lineSize));
    res->addChild(lF);

    visuFootSep->addChild(res);
    /*
    reachableFootVisuSep->removeAllChildren();
    if (reachSpace && reachSpace->getBaseNode())
    {
        Eigen::Matrix4f gp = reachSpace->getBaseNode()->getGlobalPose();
        addFootstepVisu(resultVisuSep,visuFootSep,gp);
    }*/
}

void InverseReachabilityCoinVisualization::buildFeetPolygons(SoSeparator *footLRVisuSep, VirtualRobot::RobotNodePtr footL, VirtualRobot::RobotNodePtr footR, VirtualRobot::ManipulabilityPtr reachSpace, Eigen::Matrix4f &trafoBaseToFootL, Eigen::Matrix4f &trafoBaseToFootR)
{

	if (!footL || !footR)
		return;
	if (!footL->getCollisionModel() || !footR->getCollisionModel())
		return;

    if (!footLRVisuSep)
        return;
    buildFootPolygon(footLRVisuSep, footL, reachSpace, trafoBaseToFootL);
    buildFootPolygon(footLRVisuSep, footR, reachSpace, trafoBaseToFootR);
}


void InverseReachabilityCoinVisualization::addFootstepVisu(SoSeparator* s, SoSeparator* footLRVisuSep, Eigen::Matrix4f &gp)
{
	if (!s)
		return;
	SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(gp);
	SoSeparator *resF = new SoSeparator;
	resF->addChild(mt);
	resF->addChild(footLRVisuSep);
	s->addChild(resF);
}


void InverseReachabilityCoinVisualization::buildRobotVisu(SoSeparator* robotVisuSep, RobotPtr robot)
{
	if (robotVisuSep)
		robotVisuSep->removeAllChildren();

	if (!robot || !robotVisuSep)
		return;
	boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
	SoNode* visualisationNode = NULL;
	if (visualization)
		visualisationNode = visualization->getCoinVisualization();

	if (visualisationNode)
		robotVisuSep->addChild(visualisationNode);

#if 0
    // show reach space
	if (reachSpace)
	{
#ifdef TEST_AT_TCP
		visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace,VirtualRobot::ColorMap::eHot,Eigen::Vector3f::UnitZ(),true,0,0,10);
#else
		visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace,VirtualRobot::ColorMap::eHot,true);
#endif

		//Eigen::Matrix4f tcp = Eigen::Matrix4f::Identity();
		//tcp.block(0,3,3,1) = eef->getTcp()->getGlobalPose().block(0,3,3,1);
		//visualisationNode = getCoinVisualization(reachSpace,VirtualRobot::ColorMap::eHot,tcp);

		robotVisuSep->addChild(visualisationNode);


		if (eef && eef->getTcp())
		{
			Eigen::Matrix4f tcp = eef->getTcp()->getGlobalPose();
            Eigen::AngleAxis<float> t(TEST_AT_TCP_TEST_ANGLE,Eigen::Vector3f::UnitZ());
            Eigen::Transform<float,3,Eigen::Affine> trRot(t);
            Eigen::Matrix4f graspGlobal = tcp*trRot.matrix();

            SoSeparator *s = new SoSeparator();
            SoMatrixTransform *mt = CoinVisualizationFactory::getMatrixTransform(graspGlobal);
            SoSeparator *s2 = CoinVisualizationFactory::CreateCoordSystemVisualization();
            s->addChild(mt);
            s->addChild(s2);
            robotVisuSep->addChild(s);
			WorkspaceRepresentation::WorkspaceCut2DPtr cut = reachSpace->createCut(graspGlobal,reachSpace->getDiscretizeParameterTranslation());
			visualisationNode = CoinVisualizationFactory::getCoinVisualization(cut,VirtualRobot::ColorMap::eHot, Eigen::Vector3f::UnitZ());
			robotVisuSep->addChild(visualisationNode);
		}
	}
#endif
}

//UI.checkBoxTarget->isChecked()
//UI.checkBoxFloor->isChecked()
//Eigen::Vector3f p(0,0,0);
//if (currentScene==1)
//	p(0) = -5000.0f;
void InverseReachabilityCoinVisualization::buildObjectVisu(SoSeparator *objectVisuSep, bool enable, bool showFloor, ObstaclePtr graspObject, ObstaclePtr environment, Eigen::Vector3f floorpos)
{
	if (objectVisuSep)
		objectVisuSep->removeAllChildren();
	if (!graspObject || !objectVisuSep)
		return;

	if (enable)
	{
		std::string s("Target Pose");
		graspObject->showCoordinateSystem(true, 2.0f, &s);
		boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = graspObject->getVisualization<CoinVisualization>();
		SoNode* visualisationNode = NULL;
		if (visualization)
			visualisationNode = visualization->getCoinVisualization();

		if (visualisationNode)
			objectVisuSep->addChild(visualisationNode);

		std::string s2("World Coord System");
		SoSeparator* co = CoinVisualizationFactory::CreateCoordSystemVisualization(2.0f,&s2);
		objectVisuSep->addChild(co);
	
		if (environment)
		{
			visualization = environment->getVisualization<CoinVisualization>();
			if (visualization)
				visualisationNode = visualization->getCoinVisualization();

			if (visualisationNode)
				objectVisuSep->addChild(visualisationNode);
		}
	}

	if (showFloor)
	{
		Eigen::Vector3f n(0,0,1.0f);
		SoSeparator *sf = CoinVisualizationFactory::CreatePlaneVisualization(floorpos,n,15000.0f,0.0f);
		objectVisuSep->addChild(sf);
	}
}

//UI.checkBoxReachabilityVisu->isChecked()
void InverseReachabilityCoinVisualization::buildReachVisu(SoSeparator* reachabilityVisuSep, bool enable, InverseReachabilityPtr invReach, Eigen::Matrix4f &targetPose)
{
	if (reachabilityVisuSep)
		reachabilityVisuSep->removeAllChildren();
	else
	{
		return;
	}
	if (enable && invReach)
	{
		Eigen::Matrix4f graspGlobal = targetPose;
		/*GraspPtr g;
		if (grasps && graspObject)
			g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
		if (!g)
		{
			if (!currentRobotNodeSet)
				return;
			graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
		} else
		{
			graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
		}*/

		//Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
#ifdef TEST_AT_TCP
		//if (currentRobotNodeSet)
		//	graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
		Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
		//gp.block(0,3,3,1) = reachSpace->getTcp()->getGlobalPose().block(0,3,3,1);
		gp = reachSpace->getTcp()->getGlobalPose();
        Eigen::AngleAxis<float> t(TEST_AT_TCP_TEST_ANGLE,Eigen::Vector3f::UnitZ());
        Eigen::Transform<float,3,Eigen::Affine> trRot(t);
        graspGlobal = gp*trRot.matrix();
#endif
		invReach->setGlobalPose(graspGlobal);

		//cout << "TEST: NO POSE" << endl;
		//invReach->setGlobalPose(Eigen::Matrix4f::Identity());
		//cout << "TEST: invReach at Platform" << endl;
		//RobotNodePtr rn = robot->getRobotNode("Platform");
		//invReach->setGlobalPose(rn->getGlobalPose());
			
		//cout << "TCP:" <<  eef->getTcp()->getName() << endl;
		/*Eigen::Matrix4f tcp = eef->getTcp()->getGlobalPose();
		tcp.block(0,3,3,1) = graspObject->getGlobalPose().block(0,3,3,1);
		*/
		//Eigen::Matrix4f tcp = Eigen::Matrix4f::Identity();
		//tcp.block(0,3,3,1) = eef->getTcp()->getGlobalPose().block(0,3,3,1);
            //graspObject->getGlobalPose().block(0,3,3,1);
		//invReach->setGlobalPose(tcp);
        // tcp(2,3) = 0; // floor
        //SoNode* visualisationNode = getCoinVisualization(invReach,VirtualRobot::ColorMap::eHot,tcp);


		// looks good for test data block
		//cout << "TEST: invReach at tcp pose" << endl;
		//invReach->setGlobalPose(eef->getTcp()->getGlobalPose());

#ifdef TEST_AT_TCP
		SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(invReach,VirtualRobot::ColorMap::eHot,Eigen::Vector3f::UnitY(),true,0,0,10000);
#else
//        SoNode* visualisationNode = getCoinVisualizationCut(invReach,VirtualRobot::ColorMap::eHot,true);
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(invReach, VirtualRobot::ColorMap::eHot, true);
#endif
		if (visualisationNode)
			reachabilityVisuSep->addChild(visualisationNode);

		/*MathTools::OOBB oobb = invReach->getOOBB();

		SoSeparator* oobbSep = CoinVisualizationFactory::CreateOOBBVisualization(oobb);
		reachabilityVisuSep->addChild(oobbSep);*/
	}
}

//UI.checkBoxReachabilityMapVisu->isChecked()

void InverseReachabilityCoinVisualization::buildReachMapVisu(SoSeparator* reachabilityMapVisuSep, bool enable, InverseReachabilityPtr invReach, OrientedWorkspaceGridPtr reachGrid, SoSeparator* reachableFootVisuSep, SoSeparator *footLRVisuSep)
{
	if (reachabilityMapVisuSep)
		reachabilityMapVisuSep->removeAllChildren();
	else 
		return;

	if (enable && reachGrid)
	{
		cout << "Updating reach grid visu...";
		SoNode* visualisationNode = getCoinVisualization(reachGrid,invReach,VirtualRobot::ColorMap::eHot,true,reachableFootVisuSep,footLRVisuSep);

		if (visualisationNode)
			reachabilityMapVisuSep->addChild(visualisationNode);
		cout << " done." << endl;
		/*if (invReach)
		{
			MathTools::OOBB oobb = invReach->getOOBB();
			SoSeparator* oobbSep = CoinVisualizationFactory::CreateOOBBVisualization(oobb);
			reachabilityMapVisuSep->addChild(oobbSep);
		}*/
	}
}

//reachSpace->getDiscretizeParameterTranslation()*0.55f
SoNode* InverseReachabilityCoinVisualization::getCoinVisualization( OrientedWorkspaceGridPtr reachGrid, InverseReachabilityPtr invReach, VirtualRobot::ColorMap cm, bool transformToGlobalPose, SoSeparator* reachableFootVisuSep, SoSeparator *footLRVisuSep )
{
    bool addFootsteps = false;
	SoSeparator *res = new SoSeparator;
    SoUnits *u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    res->addChild(u);
	if (!reachGrid)
		return res;
	reachableFootVisuSep->removeAllChildren();
	float arrowSize = reachGrid->getDiscretizeParameterTranslation()*0.55f;

	Eigen::Matrix4f basePose = Eigen::Matrix4f::Identity();
	if (transformToGlobalPose && reachGrid)// &&reachSpace && reachSpace->getBaseNode())
	{
		basePose(2,3) = reachGrid->getHeight();//reachSpace->getBaseNode()->getGlobalPose();
        SoMatrixTransform* matTrBase = CoinVisualizationFactory::getMatrixTransform(basePose);
		res->addChild(matTrBase);
	}

	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	float minX,maxX,minY,maxY;
	float minA = -float(M_PI);
	float maxA = float(M_PI);
	reachGrid->getExtends(minX,maxX,minY,maxY);
	gp(0,3) = minX;
	gp(1,3) = minY;

	int nX,nY,nA;

	reachGrid->getCells(nX,nY,nA);

	float sizeX = (maxX - minX) / (float)nX;
	float sizeY = (maxY - minY) / (float)nY;
	float sizeA = (maxA - minA) / (float)nA;


	float ro,gr,bl;
	SoCube *cube = new SoCube();
	cube->width = sizeX;
	cube->height = sizeY;
	cube->depth = 1.0f;
	int maxEntry = reachGrid->getMaxEntry();
	if (invReach)
		maxEntry = invReach->getMaxEntry();

	if (maxEntry==0)
		maxEntry = 1;


//    SoSeparator* arrow = CoinVisualizationFactory::CreateArrow(Eigen::Vector3f::UnitX(),arrowSize,3.0f,VisualizationFactory::Color::None());
    SoSeparator* arrow = CoinVisualizationFactory::CreateArrow(Eigen::Vector3f::UnitY(),arrowSize,3.0f,VisualizationFactory::Color::None());

	SoDrawStyle *ds = new SoDrawStyle;
	ds->style = SoDrawStyle::LINES;

	// back-face culling
	SoShapeHints * shapeHints = new SoShapeHints;
	//shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
	shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
	shapeHints->shapeType = SoShapeHints::SOLID;

	SoBaseColor *bc = new SoBaseColor;
	bc->rgb.setValue(0,0,0);

	// keep a solid color
	SoLightModel * lightModel = new SoLightModel;
	lightModel->model = SoLightModel::BASE_COLOR;
	float alpha;

	float xStep = 3;
	float yStep = 3;
#ifdef TEST_AT_TCP
	xStep = 1;
	yStep = 1;
#endif

    int nCount = 0;

	for (int x = 0; x<nX; x+=(int)xStep)
	{
		float xPos = minX + (float)x * sizeX + 0.5f*sizeX; // center of voxel
		for (int y = 0; y<nY; y+=(int)yStep)
		{

			std::vector<int> entries;
			std::vector<float> rots;
			std::vector< std::vector<GraspPtr> > graspsAll;
			bool ok = reachGrid->getCellEntries(x,y,rots,entries,graspsAll);
			if (ok && rots.size()>0)
			for (int k=0;k<(int)rots.size();k++)
			{
				alpha = rots[k];
				int v = entries[k];
				std::vector<GraspPtr> grasps = graspsAll[k];

				float yPos = minY + (float)y * sizeY + 0.5f*sizeY; // center of voxel
				gp(0,3) = xPos;
				gp(1,3) = yPos;

				SoSeparator *sep1 = new SoSeparator();
				/*
				Eigen::AngleAxis<float> t(alpha,Eigen::Vector3f(0,0,1));
				Eigen::Transform<float,3,Eigen::Affine> trRot(t);
				Eigen::Matrix4f gp2 = gp * trRot.matrix(); // ok*/
				Eigen::Matrix4f gp2;
				float resPos[6];
				resPos[0] = xPos;
				resPos[1] = yPos;
				resPos[2] = 0;
				resPos[3] = 0;
				resPos[4] = 0;
				resPos[5] = alpha;
                
				MathTools::posrpy2eigen4f(resPos,gp2);

                SoMatrixTransform* matTr = CoinVisualizationFactory::getMatrixTransform(gp2);

				float intensity = (float)v;
				intensity /= maxEntry;
                if (intensity>1.0f)
                    intensity = 1.0f;				
                if (intensity<0.0f)
                    intensity = 0.0f;
				VirtualRobot::VisualizationFactory::Color color = cm.getColor(intensity);

				SoMaterial *mat= new SoMaterial();

				ro = color.r;
				gr = color.g;
				bl = color.b;

				mat->diffuseColor.setValue(ro,gr,bl);
				mat->ambientColor.setValue(ro,gr,bl);
				if (intensity>0)
				{
                    nCount++;

					sep1->addChild(matTr);
					sep1->addChild(mat);
					sep1->addChild(arrow);
		
					
					//sep1->addChild(cube);

                    SoSeparator *pSepLines = new SoSeparator;
					sep1->addChild(pSepLines);

					pSepLines->addChild(ds);
					pSepLines->addChild(shapeHints);
					pSepLines->addChild(lightModel);
					pSepLines->addChild(bc);

                    pSepLines->addChild(cube);
					

					res->addChild(sep1);
                    if (addFootsteps)
					{
						Eigen::Matrix4f p = basePose*gp2;
						addFootstepVisu(reachableFootVisuSep,footLRVisuSep,p);
					}
				}
			}
		}
	}
    cout << "Nr of arrows drawn:" << nCount << endl;
	//res->addChild(lines);
	return res;

}


}
