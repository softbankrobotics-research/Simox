#include "PhysicsAttachment.h"

#include "../ModelLink.h"
#include "../../../Visualization/TriMeshModel.h"
#include "../../../Visualization/VisualizationFactory.h"

#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

namespace VirtualRobot
{
    PhysicsAttachment::PhysicsAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation, const std::string &visualizationType)
            : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
        initVisualization();
    }

    void PhysicsAttachment::initVisualization()
    {
        if (!getParent()) return;

        // since this class can only be attached to links, casting is safe here.
        ModelLinkPtr node = std::static_pointer_cast<ModelLink>(getParent());
        if (node->getMass() <= 0)
        {
            VR_WARNING << "Trying to attach '" << getName() << "' to Link '" << node->getName() << "' with zero or negative mass! Skipping visualization..." << std::endl;
            return;
        }

        VisualizationFactoryPtr factory = visualizationType.empty() ? VisualizationFactory::getGlobalVisualizationFactory()
                                                                    : VisualizationFactory::fromName(visualizationType, nullptr);

        if (!factory)
        {
            VR_WARNING << "Error while retrieving VisualizationFactory with name '" << visualizationType << "'! Skipping visualization..." << std::endl;
            return;
        }


        VisualizationPtr visuNodeCoM;
        // visualize CoM
        {
            VisualizationPtr comModel1 = factory->createSphere(7.05f);
            comModel1->setColor(Visualization::Color::Red());
            VisualizationPtr comModel2 = factory->createBox(10.0f, 10.0f, 10.0f);
            comModel2->setColor(Visualization::Color::Blue());
            std::vector<VisualizationPtr> v;
            v.push_back(comModel1);
            v.push_back(comModel2);
            visuNodeCoM = factory->createVisualisationSet(v);

            std::stringstream ss;
            ss << "COM: " << getName();
            std::string t = ss.str();
            VisualizationPtr vText = factory->createText(t, true, 0, 10.0f, 0);
            vText->setColor(Visualization::Color::Blue());
            v.clear();
            v.push_back(visuNodeCoM);
            v.push_back(vText);
            visuNodeCoM = factory->createVisualisationSet(v);

            Eigen::Vector3f comLocation = node->getCoMLocal();
            Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
            m.block(0, 3, 3, 1) = comLocation;
            visuNodeCoM->applyDisplacement(m);
        }

        VisualizationPtr visuNodeInertia;
        // visualize inertia
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(node->getInertiaMatrix());

            if (eigensolver.info() == Eigen::Success)
            {
                float xl = (float)(eigensolver.eigenvalues().rows() > 0 ? eigensolver.eigenvalues()(0) : 1e-6);
                float yl = (float)(eigensolver.eigenvalues().rows() > 1 ? eigensolver.eigenvalues()(1) : 1e-6);
                float zl = (float)(eigensolver.eigenvalues().rows() > 2 ? eigensolver.eigenvalues()(2) : 1e-6);

                if (fabs(xl) < 1e-6)
                {
                    xl = 1e-6f;
                }

                if (fabs(yl) < 1e-6)
                {
                    yl = 1e-6f;
                }

                if (fabs(zl) < 1e-6)
                {
                    zl = 1e-6f;
                }

                xl = 1.0f / sqrtf(xl);
                yl = 1.0f / sqrtf(yl);
                zl = 1.0f / sqrtf(zl);
                float le = sqrtf(xl * xl + yl * yl + zl * zl);
                float scaleFactor = 5.0f;
                float axesSize = 4.0f * le * scaleFactor / 20.0f;

                if (axesSize > 4.0f)
                {
                    axesSize = 4.0f;
                }

                if (axesSize < 0.4f)
                {
                    axesSize = 0.4f;
                }

                xl *= scaleFactor;
                yl *= scaleFactor;
                zl *= scaleFactor;

                float minSize = 10.0f;
                float maxSize = 50.0f;
                float maxAx = xl;

                if (yl > xl && yl > zl)
                {
                    maxAx = yl;
                }

                if (zl > xl && zl > xl)
                {
                    maxAx = zl;
                }

                if (maxAx < minSize)
                {
                    if (maxAx < 1e-6f)
                    {
                        maxAx = 1e-6f;
                    }

                    xl = xl / maxAx * minSize;
                    yl = yl / maxAx * minSize;
                    zl = zl / maxAx * minSize;
                }

                if (maxAx > maxSize)
                {
                    xl = xl / maxAx * maxSize;
                    yl = yl / maxAx * maxSize;
                    zl = zl / maxAx * maxSize;
                }

                visuNodeInertia = factory->createEllipse(xl, yl, zl);
                // TODO show axes
                // visuNodeInertia = factory->createEllipse(xl, yl, zl, true, axesSize, axesSize * 2.0f);

                Eigen::Vector3f comLocation = node->getCoMLocal();
                Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
                m.block(0, 3, 3, 1) = comLocation;
                m.block(0, 0, 3, 3) = eigensolver.eigenvectors().block(0, 0, 3, 3); // rotate according to EV
                visuNodeInertia->applyDisplacement(m);
            }
            else
            {
                VR_WARNING << "Could not determine eigenvectors of inertia matrix." << endl;
            }
        }

        std::vector<VisualizationPtr> visus;
        visus.push_back(visuNodeCoM);
        visus.push_back(visuNodeInertia);

        setVisualization(factory->createVisualisationSet(visus));
    }

    void PhysicsAttachment::setParent(const VirtualRobot::ModelNodePtr &node)
    {
        ModelNodeAttachment::setParent(node);
        initVisualization();
    }

    bool PhysicsAttachment::isAttachable(const VirtualRobot::ModelNodePtr &node)
    {
        return node->isLink();
    }
}
