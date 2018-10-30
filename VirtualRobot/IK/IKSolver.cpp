#include "IKSolver.h"

namespace VirtualRobot
{
    IKSolver::IKSolver(const JointSetPtr &rns) :
		rns(rns)
    {
		if (rns)
			tcp = rns->getTCP();
		verbose = false;
    }


    VirtualRobot::FramePtr IKSolver::getTcp() const
	{
		return tcp;
	}

    VirtualRobot::JointSetPtr IKSolver::getJointSet() const
	{
		return rns;
	}

	void IKSolver::setVerbose(bool enable)
	{
        verbose = enable;
    }

    bool IKSolver::isVerbose() const
    {
        return verbose;
    }

} // namespace VirtualRobot
