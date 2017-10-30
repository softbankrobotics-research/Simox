
#include "CollisionModelPQP.h"

#include "../CollisionChecker.h"
#include "CollisionCheckerPQP.h"
#include "../../Visualization/TriMeshModel.h"

namespace VirtualRobot
{

    CollisionModelPQP::CollisionModelPQP(const TriMeshModelPtr &modelData, CollisionCheckerPtr colChecker, int id)
        : CollisionModelImplementation(modelData, colChecker, id)
    {
        if (!colChecker)
        {
            colChecker = CollisionChecker::getGlobalCollisionChecker();
        }

        if (!colChecker)
        {
            VR_WARNING << "no col checker..." << endl;
        }
        else
        {
            colCheckerPQP = colChecker->getCollisionCheckerImplementation();
        }

        createPQPModel();
    }

    CollisionModelPQP::CollisionModelPQP(const CollisionModelPQP &orig) :
        CollisionModelImplementation(orig.modelData, NULL, orig.id)
    {
        pqpModel = orig.pqpModel;
    }


    CollisionModelPQP::~CollisionModelPQP()
    {
    }


    void CollisionModelPQP::destroyData()
    {
        pqpModel.reset();
    }

    void CollisionModelPQP::createPQPModel()
    {
        if (!modelData)
        {
            VR_WARNING << "no model data in PQP!" << endl;
            return;
        }

        PQP::PQP_REAL a[3];
        PQP::PQP_REAL b[3];
        PQP::PQP_REAL c[3];

        pqpModel.reset(new PQP::PQP_Model());
        pqpModel->BeginModel(modelData->faces.size());

        for (unsigned int i = 0; i < modelData->faces.size(); i++)
        {
            a[0] = modelData->vertices[modelData->faces[i].id1][0];
            a[1] = modelData->vertices[modelData->faces[i].id1][1];
            a[2] = modelData->vertices[modelData->faces[i].id1][2];
            b[0] = modelData->vertices[modelData->faces[i].id2][0];
            b[1] = modelData->vertices[modelData->faces[i].id2][1];
            b[2] = modelData->vertices[modelData->faces[i].id2][2];
            c[0] = modelData->vertices[modelData->faces[i].id3][0];
            c[1] = modelData->vertices[modelData->faces[i].id3][1];
            c[2] = modelData->vertices[modelData->faces[i].id3][2];
            pqpModel->AddTri(a, b, c, this->id);
        }

        pqpModel->EndModel();
    }

    void CollisionModelPQP::print()
    {
        cout << "   CollisionModelPQP: ";

        if (pqpModel)
        {
            cout << pqpModel->num_tris << " triangles." << endl;
            float mi[3];
            float ma[3];

            if (pqpModel->num_tris > 0)
            {
                for (int j = 0; j < 3; j++)
                {
                    mi[j] = ma[j] = pqpModel->tris[0].p1[j];
                }

                for (int i = 0; i < pqpModel->num_tris; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (pqpModel->tris[i].p1[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p1[j];
                        }

                        if (pqpModel->tris[i].p2[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p2[j];
                        }

                        if (pqpModel->tris[i].p3[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p3[j];
                        }

                        if (pqpModel->tris[i].p1[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p1[j];
                        }

                        if (pqpModel->tris[i].p2[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p2[j];
                        }

                        if (pqpModel->tris[i].p3[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p3[j];
                        }
                    }
                }

                cout << "    Min point: (" << mi[0] << "," << mi[1] << "," << mi[2] << ")" << endl;
                cout << "    Max point: (" << ma[0] << "," << ma[1] << "," << ma[2] << ")" << endl;
            }
        }
        else
        {
            cout << "no model." << endl;
        }

        cout << endl;
    }

    std::shared_ptr<CollisionModelImplementation> CollisionModelPQP::clone(bool deepCopy) const
    {
        std::shared_ptr<CollisionModelPQP> p(new CollisionModelPQP(*this));
        if(deepCopy)
        {
            p->createPQPModel();
        }

        VR_ASSERT(this->pqpModel);
        return p;
    }


} // namespace
