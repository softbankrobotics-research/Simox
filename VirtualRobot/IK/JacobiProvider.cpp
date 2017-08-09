#include <Eigen/Geometry>
#include "JacobiProvider.h"
#include "../VirtualRobotException.h"
#include "../Tools/MathTools.h"

#include <algorithm>

using namespace Eigen;

//#define CHECK_PERFORMANCE

namespace VirtualRobot
{

    JacobiProvider::JacobiProvider(JointSetPtr rns, InverseJacobiMethod invJacMethod) :
        name("JacobiProvvider"), rns(rns), inverseMethod(invJacMethod)
    {
        initialized = false;
    }

    JacobiProvider::~JacobiProvider()
    {
    }

    MatrixXd JacobiProvider::getJacobianMatrixD()
    {
        return getJacobianMatrix().cast<double>();
    }

    MatrixXd JacobiProvider::getJacobianMatrixD(FramePtr tcp)
    {
        return getJacobianMatrix(tcp).cast<double>();
    }

    Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix(FramePtr tcp)
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
        MatrixXf Jacobian = this->getJacobianMatrix(tcp);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
        Eigen::MatrixXf res = computePseudoInverseJacobianMatrix(Jacobian);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1 << ", time2: " << diffClock2 << endl;
#endif
        return res;
    }

    Eigen::MatrixXd JacobiProvider::getPseudoInverseJacobianMatrixD(FramePtr tcp)
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif
        MatrixXd Jacobian = this->getJacobianMatrixD(tcp);
#ifdef CHECK_PERFORMANCE
        clock_t startT2 = clock();
#endif
        Eigen::MatrixXd res = computePseudoInverseJacobianMatrixD(Jacobian);
#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock1 = (float)(((float)(startT2 - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        float diffClock2 = (float)(((float)(endT - startT2) / (float)CLOCKS_PER_SEC) * 1000.0f);
        cout << "getPseudoInverseJacobianMatrix time1:" << diffClock1 << ", time2: " << diffClock2 << endl;
#endif
        return res;
    }


    Eigen::MatrixXf JacobiProvider::getPseudoInverseJacobianMatrix()
    {
        MatrixXf Jacobian = this->getJacobianMatrix();
        return computePseudoInverseJacobianMatrix(Jacobian);
        //return getPseudoInverseJacobianMatrix(rns->getTCP());
    }

    Eigen::MatrixXd JacobiProvider::getPseudoInverseJacobianMatrixD()
    {
        MatrixXd Jacobian = this->getJacobianMatrixD();
        return computePseudoInverseJacobianMatrixD(Jacobian);
    }

    Eigen::MatrixXf JacobiProvider::computePseudoInverseJacobianMatrix(const Eigen::MatrixXf& m) const
    {
        return computePseudoInverseJacobianMatrix(m, 0.0f);
    }

    MatrixXd JacobiProvider::computePseudoInverseJacobianMatrixD(const MatrixXd& m) const
    {
        return computePseudoInverseJacobianMatrixD(m, 0.0);
    }

    Eigen::MatrixXf JacobiProvider::computePseudoInverseJacobianMatrix(const Eigen::MatrixXf& m, float invParameter) const
    {
        Eigen::MatrixXf result(m.cols(), m.rows());
        updatePseudoInverseJacobianMatrix(result, m, invParameter);
        return result;
    }

    Eigen::MatrixXd JacobiProvider::computePseudoInverseJacobianMatrixD(const Eigen::MatrixXd& m, double invParameter) const
    {
        Eigen::MatrixXd result(m.cols(), m.rows());
        updatePseudoInverseJacobianMatrixD(result, m, invParameter);
        return result;
    }

    void JacobiProvider::updatePseudoInverseJacobianMatrix(Eigen::MatrixXf& invJac, const Eigen::MatrixXf& m, float invParameter) const
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif

        VR_ASSERT(m.rows() > 0 && m.cols() > 0 && invJac.cols() == m.rows() && invJac.rows() == m.cols());

        switch (inverseMethod)
        {
            case eTranspose:
            {
                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXf W = jointWeights.asDiagonal();
                    Eigen::MatrixXf W_1 = W.inverse();
                    invJac = W_1 * m.transpose() * (m * W_1 * m.transpose()).inverse();
                }
                else
                {
                    invJac = m.transpose() * (m * m.transpose()).inverse();
                }

                break;
            }

            case eSVD:
            {
                float pinvtoler = 0.00001f;

                if (invParameter != 0.0f)
                {
                    pinvtoler = invParameter;
                }

                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXf W_12(jointWeights.rows(), jointWeights.rows());
                    W_12.setZero();

                    for (int i = 0; i < jointWeights.rows(); i++)
                    {
                        THROW_VR_EXCEPTION_IF(jointWeights(i) <= 0.f, "joint weights cannot be negative or zero");
                        W_12(i, i) = sqrt(1 / jointWeights(i));
                    }

                    invJac = W_12 * MathTools::getPseudoInverse(m * W_12, pinvtoler);
                }
                else
                {
                    invJac = MathTools::getPseudoInverse(m, pinvtoler);
                }

                break;
            }

            case eSVDDamped:
            {
                float pinvtoler = 1.0f;

                if (invParameter != 0.0f)
                {
                    pinvtoler = invParameter;
                }

                invJac = MathTools::getPseudoInverseDamped(m, pinvtoler);
                break;
            }

            default:
                THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
        }

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        //if (diffClock>10.0f)
        cout << "Inverse Jacobi time:" << diffClock << endl;
#endif
    }


    void JacobiProvider::updatePseudoInverseJacobianMatrixD(Eigen::MatrixXd& invJac, const Eigen::MatrixXd& m, double invParameter) const
    {
#ifdef CHECK_PERFORMANCE
        clock_t startT = clock();
#endif

        VR_ASSERT(m.rows() > 0 && m.cols() > 0 && invJac.cols() == m.rows() && invJac.rows() == m.cols());

        switch (inverseMethod)
        {
            case eTranspose:
            {
                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXd W = jointWeights.cast<double>().asDiagonal();
                    Eigen::MatrixXd W_1 = W.inverse();
                    invJac = W_1 * m.transpose() * (m * W_1 * m.transpose()).inverse();
                }
                else
                {
                    invJac = m.transpose() * (m * m.transpose()).inverse();
                }

                break;
            }

            case eSVD:
            {
                double pinvtoler = 0.00001;

                if (invParameter != 0.0f)
                {
                    pinvtoler = invParameter;
                }

                if (jointWeights.rows() == m.cols())
                {
                    Eigen::MatrixXd W_12(jointWeights.rows(), jointWeights.rows());
                    W_12.setZero();

                    for (int i = 0; i < jointWeights.rows(); i++)
                    {
                        THROW_VR_EXCEPTION_IF(jointWeights(i) <= 0.f, "joint weights cannot be negative or zero");
                        W_12(i, i) = sqrt(1 / jointWeights(i));
                    }

                    invJac = W_12 * MathTools::getPseudoInverseD(m * W_12, pinvtoler);
                }
                else
                {
                    invJac = MathTools::getPseudoInverseD(m, pinvtoler);
                }

                break;
            }

            case eSVDDamped:
            {
                double pinvtoler = 1.0;

                if (invParameter != 0.0)
                {
                    pinvtoler = invParameter;
                }

                invJac = MathTools::getPseudoInverseDampedD(m, pinvtoler);
                break;
            }

            default:
                THROW_VR_EXCEPTION("Inverse Jacobi Method nyi...");
        }

#ifdef CHECK_PERFORMANCE
        clock_t endT = clock();
        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        //if (diffClock>10.0f)
        cout << "Inverse Jacobi time:" << diffClock << endl;
#endif
    }



    VirtualRobot::JointSetPtr JacobiProvider::getJointSet()
    {
        return rns;
    }

    void JacobiProvider::setJointWeights(const Eigen::VectorXf& jointWeights)
    {
        this->jointWeights = jointWeights;
    }

    void JacobiProvider::print()
    {
        cout << "IK solver:" << name << endl;
        cout << "==========================" << endl;
        cout << "RNS:" << rns->getName() << " with " << rns->getSize() << " joints" << endl;
    }

    bool JacobiProvider::isInitialized()
    {
        return initialized;
    }

} // namespace VirtualRobot
