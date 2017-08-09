#include "IKSolver.h"

namespace VirtualRobot
{
    IKSolver::IKSolver(JointSetPtr rns) :
		rns(rns)
    {
		if (rns)
			tcp = rns->getTCP();
		verbose = false;
    }


	VirtualRobot::FramePtr IKSolver::getTcp()
	{
		return tcp;
	}

	VirtualRobot::JointSetPtr IKSolver::getJointSet()
	{
		return rns;
	}

	void IKSolver::setVerbose(bool enable)
	{
		verbose = enable;
	}

} // namespace VirtualRobot
