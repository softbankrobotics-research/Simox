#include "InverseReachability.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace VirtualRobot {

InverseReachability::InverseReachability(VirtualRobot::WorkspaceRepresentationPtr ws, float factorTranslation, float factorOrientation) 
: VirtualRobot::WorkspaceRepresentation (ws->getRobot())
{
	type = "InverseReachability";
	dataPose.setIdentity();
	this->ws = ws;
	buildData(factorTranslation,factorOrientation);
}

InverseReachability::InverseReachability( VirtualRobot::WorkspaceRepresentationPtr ws, const std::string &filename )
: VirtualRobot::WorkspaceRepresentation (ws->getRobot())
{
    type = "InverseReachability";
    dataPose.setIdentity();
    this->ws = ws;
    load(filename);
}

InverseReachability::InverseReachability( VirtualRobot::RobotPtr robot, const std::string &filename )
    : VirtualRobot::WorkspaceRepresentation(robot)
{
    type = "InverseReachability";
    dataPose.setIdentity();
    load(filename);
}

bool InverseReachability::isReachable( const Eigen::Matrix4f &globalPose )
{
	return isCovered(globalPose);
}

Eigen::Matrix4f InverseReachability::sampleReachablePose()
{
    return sampleCoveredPose();
}


Eigen::Matrix4f InverseReachability::getToLocalTransformation() const
{
	//if (baseNode)
	//	return baseNode->toLocalCoordinateSystem(dataPose.inverse());
	//else
		return dataPose.inverse();
}

Eigen::Matrix4f InverseReachability::getToGlobalTransformation() const
{
	//if (baseNode)
	//	return baseNode->toGlobalCoordinateSystem(dataPose);
	//else
		return dataPose;
}

void InverseReachability::setGlobalPose( const Eigen::Matrix4f &gp )
{
	dataPose = gp;
}


//#define PRINT_INV_DEBUG
void InverseReachability::addInverseData(Eigen::Matrix4f &m, unsigned char e)
{
    float xL[6];
    unsigned int vL[6];
    Eigen::Matrix4f m2;
#ifdef PRINT_INV_DEBUG
    matrix2Vector(m,xL);
    cout << "Center local ws pose:";
    for (int i=0;i<6;i++)
        cout << xL[i] << ",";
    cout << endl;

#endif
    m2 = m.inverse(); // inverse transformation
    matrix2Vector(m2,xL);

#ifdef PRINT_INV_DEBUG
    cout << "Center INV  inverse pose:";
    for (int i=0;i<6;i++)
        cout << xL[i] << ",";
    cout << endl;
#endif

    // get voxel and set data
    if (getVoxelFromPose(xL,vL))
    {
        // check for achieved values
        for (int i=0;i<6;i++)
        {
            float voxelSize;
            if (i<3)
                voxelSize = getDiscretizeParameterTranslation();
            else
                voxelSize = getDiscretizeParameterRotation();
            float minXTest = xL[i] - voxelSize;
            float maxXTest = xL[i] + voxelSize;
            if (minXTest<getMinBound(i))
                minXTest = getMinBound(i);
            if (maxXTest>getMaxBound(i))
                maxXTest = getMaxBound(i);


            if (minXTest < achievedMinValues[i])
                achievedMinValues[i] = minXTest;
            if (maxXTest > achievedMaxValues[i])
                achievedMaxValues[i] = maxXTest;
        }
#ifdef PRINT_INV_DEBUG
        cout << "Center INV  voxel:";
        for (int i=0;i<6;i++)
            cout << vL[i] << ",";
        cout << endl;
#endif
        // only if higher
        if (data->get(vL) < e)
        {
            data->setDatum(vL,e);
            //data->setDatumCheckNeighbors(vL,e,1);
        }
    }
}

void InverseReachability::buildData(float factorTranslation, float factorOrientation)
{
    VR_ASSERT(ws);
    VR_ASSERT(factorTranslation>0);
    VR_ASSERT(factorOrientation>0);

	float minBounds[6];
	float maxBounds[6];

	// check max length

	float maxL[3];
    for (int i=0;i<3;i++)
		if (fabs(ws->getMinBound(i))>fabs(ws->getMaxBound(i)))
			maxL[i] = fabs(ws->getMinBound(i));
		else
			maxL[i] = fabs(ws->getMaxBound(i));

    // get the biggest possible translation from tcp to base
	float maxDiag = sqrtf(maxL[0]*maxL[0] + maxL[1]*maxL[1] + maxL[2]*maxL[2]);
	for (int i=0;i<3;i++)
	{
        // the biggest possible translation from tcp to base determines the size of the bounding box
        minBounds[i] = -maxDiag;
		maxBounds[i] = maxDiag;
		//maxBounds[i] = -ws->getMinBound(i); // invert
		//minBounds[i] = -ws->getMaxBound(i);
	}
    for (int i=3;i<6;i++)
    {
        minBounds[i] = ws->getMinBound(i);
        maxBounds[i] = ws->getMaxBound(i);
    }
    /*minBounds[3] = float(-M_PI);
    minBounds[4] = float(-M_PI);// /2.0f;
	minBounds[5] = float(-M_PI);
	maxBounds[3] = float(M_PI);
    maxBounds[4] = float(M_PI);// /2.0f;
    maxBounds[5] = float(M_PI);*/

	initialize(ws->getNodeSet(),
		ws->getDiscretizeParameterTranslation()*factorTranslation,//*1.5f, 
		ws->getDiscretizeParameterRotation()*factorOrientation, 
		minBounds, 
		maxBounds,
		ws->getCollisionModelStatic(),
		ws->getCollisionModelDynamic(),
		ws->getBaseNode(),
		ws->getTcp(),
		ws->getAdjustOnOverflow());

	// fill with inverted data

#if 0
	// wrong assumption here: we need to invert the transformations (including the rotation!)
	int a,b,c,d,e,f;
	for (a=0;a<ws->getNumVoxels(0);a++)
		for (b=0;b<ws->getNumVoxels(1);b++)
			for (c=0;c<ws->getNumVoxels(2);c++)
				for (d=0;d<ws->getNumVoxels(3);d++)
					for (e=0;e<ws->getNumVoxels(4);e++)
						for (f=0;f<ws->getNumVoxels(5);f++)
						{
							data->setDatum(	numVoxels[0]-a-1,
											numVoxels[1]-b-1,
											numVoxels[2]-c-1,
											numVoxels[3]-d-1,
											numVoxels[4]-e-1,
											numVoxels[5]-f-1,
											ws->getVoxelEntry(a,b,c,d,e,f)
											);
						}

#else
	Eigen::Matrix4f m;
	unsigned int v[6];
    //float xL[6];
    //unsigned int vL[6];
    int nPos = 1;
    int nPos2 = 1;
	for (v[0]=0;v[0]<(unsigned int)ws->getNumVoxels(0);v[0]+=nPos)
	{
		cout << "Starting x=" << v[0] << endl;
		for (v[1]=0;v[1]<(unsigned int)ws->getNumVoxels(1);v[1]+=nPos)
			for (v[2]=0;v[2]<(unsigned int)ws->getNumVoxels(2);v[2]+=nPos)
			{
				if (ws->hasEntry(v[0],v[1],v[2]))
				{
				for (v[3]=0;v[3]<(unsigned int)ws->getNumVoxels(3);v[3]+=nPos2)
					for (v[4]=0;v[4]<(unsigned int)ws->getNumVoxels(4);v[4]+=nPos2)
						for (v[5]=0;v[5]<(unsigned int)ws->getNumVoxels(5);v[5]+=nPos2)
						{
							unsigned char e = ws->getVoxelEntry(v[0],v[1],v[2],v[3],v[4],v[5]);

							if (e>0)
							{                            
                                m = ws->getPoseFromVoxel(v,false);
                                addInverseData(m,e);

//                                // add inv data according to voxel extends
//                                float v2[6];

//                                float minX[6];
//                                float maxX[6];
//                                for (int i=0;i<6;i++)
//                                {
//                                    minX[i] = float(v[i]);
//                                    if (i<3)
//                                        minX[i] /= factorTranslation;
//                                    else
//                                        minX[i] /= factorOrientation;

//                                    maxX[i] = minX[i] + 1.0f;
//                                    if (minX[i]<0)
//                                        minX[i] = 0;
//                                    if (maxX[i]>=(float)data->getSize(i))
//                                        maxX[i] = float(data->getSize(i)-1);
//                                    VR_ASSERT(minX[i]<(float)data->getSize(i));
//                                    VR_ASSERT(maxX[i]<(float)data->getSize(i));
//                                    VR_ASSERT(minX[i]>=0);
//                                    VR_ASSERT(maxX[i]>=0);
//                                }
//                                float vStep = 0.5f;
//                                for (v2[0]=minX[0]; v2[0]<=maxX[0]; v2[0]+=vStep)
//                                    for (v2[1]=minX[1]; v2[1]<=maxX[1]; v2[1]+=vStep)
//                                        for (v2[2]=minX[2]; v2[2]<=maxX[2]; v2[2]+=vStep)
//                                            for (v2[3]=minX[3]; v2[3]<=maxX[3]; v2[3]+=vStep)
//                                                for (v2[4]=minX[4]; v2[4]<=maxX[4]; v2[4]+=vStep)
//                                                    for (v2[5]=minX[5]; v2[5]<=maxX[5]; v2[5]+=vStep)
//                                                    {
//                                                        m = ws->getPoseFromVoxel(v2,false);
//#ifdef PRINT_INV_DEBUG
//                                                            cout << "Voxel:";
//                                                            for (int i=0;i<6;i++)
//                                                                cout << v2[i] << ",";
//                                                            cout << endl;
//#endif

//                                                        addInverseData(m,e);
//                                                    }


                            }
                        }

     /*
#if 0
								cout << endl << "INV: ws voxel:";
								for (int i=0;i<6;i++)
									cout << v[i] << ",";
								cout << endl;
#endif
								m = ws->getPoseFromVoxel(v,false);

#ifdef PRINT_INV_DEBUG
								matrix2Vector(m,xL);
								cout << "Center local ws pose:";
								for (int i=0;i<6;i++)
									cout << xL[i] << ",";
								cout << endl;

#endif
								
								m = m.inverse().eval(); // inverse transformation
								matrix2Vector(m,xL);
#ifdef PRINT_INV_DEBUG
								cout << "Center INV  inverse pose:";
								for (int i=0;i<6;i++)
									cout << xL[i] << ",";
								cout << endl;
#endif
								

								// get voxel and set data
								if (getVoxelFromPose(xL,vL))
								{
									// check for achieved values
									for (int i=0;i<6;i++)
									{
										float voxelSize;
										if (i<3)
											voxelSize = getDiscretizeParameterTranslation();
										else
											voxelSize = getDiscretizeParameterRotation();
										float minXTest = xL[i] - voxelSize;
										float maxXTest = xL[i] + voxelSize;
										if (minXTest<getMinBound(i))
											minXTest = getMinBound(i);
										if (maxXTest>getMaxBound(i))
											maxXTest = getMaxBound(i);

										
										if (minXTest < achievedMinValues[i])
											achievedMinValues[i] = minXTest;
										if (maxXTest > achievedMaxValues[i])
											achievedMaxValues[i] = maxXTest;
									}
#ifdef PRINT_INV_DEBUG
									cout << "Center INV  voxel:";
									for (int i=0;i<6;i++)
										cout << vL[i] << ",";
#endif
									// only if higher
									if (data->get(vL) < e)
									{	
										data->setDatum(vL,e);
										//data->setDatumCheckNeighbors(vL,e,1);
									}
								}

// add inv data according to voxel extends 
#if 1
								float v2[6];
								//for (int i=0;i<6;i++)
								//	v2[i] = v[i];

								float minX[6];
								float maxX[6];
								for (int i=0;i<6;i++)
								{
									minX[i] = float(v[i]);
									maxX[i] = float(v[i]) + 1.0f;
									if (minX[i]<0)
										minX[i] = 0;
									if (maxX[i]>=(float)data->getSize(i))
										maxX[i] = float(data->getSize(i)-1);
								}
                                float vStep = 0.5f;
								for (v2[0]=minX[0]; v2[0]<=maxX[0]; v2[0]+=vStep)
									for (v2[1]=minX[1]; v2[1]<=maxX[1]; v2[1]+=vStep)
										for (v2[2]=minX[2]; v2[2]<=maxX[2]; v2[2]+=vStep)
											for (v2[3]=minX[3]; v2[3]<=maxX[3]; v2[3]+=vStep)
												for (v2[4]=minX[4]; v2[4]<=maxX[4]; v2[4]+=vStep)
													for (v2[5]=minX[5]; v2[5]<=maxX[5]; v2[5]+=vStep)
													{
														m = ws->getPoseFromVoxel(v2,false);
#ifdef PRINT_INV_DEBUG
                                                        cout << "----------" << endl;
                                                        matrix2Vector(m,xL);
                                                        cout << "local ws pose:";
                                                        for (int i=0;i<6;i++)
                                                            cout << xL[i] << ",";
                                                        cout << endl;

#endif
														m = m.inverse().eval(); // inverse transformation
														matrix2Vector(m,xL);
#ifdef PRINT_INV_DEBUG
                                                        cout << "INV  inverse pose:";
                                                        for (int i=0;i<6;i++)
                                                            cout << xL[i] << ",";
                                                        cout << endl;
#endif
														if (getVoxelFromPose(xL,vL))
														{
															for (int i=0;i<6;i++)
															{
																float voxelSize;
																if (i<3)
																	voxelSize = getDiscretizeParameterTranslation();
																else
																	voxelSize = getDiscretizeParameterRotation();
																float minXTest = xL[i] - voxelSize;
																float maxXTest = xL[i] + voxelSize;
																if (minXTest<getMinBound(i))
																	minXTest = getMinBound(i);
																if (maxXTest>getMaxBound(i))
																	maxXTest = getMaxBound(i);


																if (minXTest < achievedMinValues[i])
																	achievedMinValues[i] = minXTest;
																if (maxXTest > achievedMaxValues[i])
																	achievedMaxValues[i] = maxXTest;
															}
#ifdef PRINT_INV_DEBUG
                                                            cout << "INV  voxel:";
                                                            for (int i=0;i<6;i++)
                                                                cout << vL[i] << ",";
                                                            cout << endl;
#endif
															if (data->get(vL) < e)
																data->setDatum(vL,e);
														}
													}
#endif
							}
					}
                */
				}
			}
	print();
	}
	print();

#endif

}


}
