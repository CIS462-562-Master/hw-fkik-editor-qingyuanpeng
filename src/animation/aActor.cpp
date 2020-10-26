#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	AJoint* root = m_pSkeleton->getRootNode();
	vec3 pos = root->getGlobalTranslation();
	// 2.	Set the y component of the guide position to 0
	pos[1] = 0.0;
	m_Guide.setGlobalTranslation(pos);
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	guideTargetPos = (guideTargetPos - pos).Normalize();
	vec3 up = vec3(0.0, 1.0, 0.0);
	mat3 rot = mat3(up.Cross(guideTargetPos), up, guideTargetPos).Transpose();

	/*vec3 v = pos.Cross(guideTargetPos);
	double s = v.Length();
	double c = Dot(pos, guideTargetPos);
	vec3 vx1 = vec3(0.0,  -v[2],  v[1]);
	vec3 vx2 = vec3(v[2], 0.0  , -v[0]);
	vec3 vx3 = vec3(-v[1], v[0],   0.0);
	mat3 vx = mat3(vx1, vx2, vx3);
	mat3 rot = IdentityMat3 + vx + vx * vx * (1.0 - c) / (s * s);
	*/

	m_Guide.setGlobalRotation(rot);
	m_pSkeleton->update();
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	AJoint* root = m_pSkeleton->getRootNode();
	vec3 pos = root->getLocalTranslation();
	//pos[1] += (leftHeight + rightHeight) / 2.0;

	if (leftHeight < rightHeight) {
		pos[1] += leftHeight;
	}
	else {
		pos[1] += rightHeight;
	}

	root->setLocalTranslation(pos);
	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 

	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal		
		ATarget tgtLeft = ATarget();

		vec3 leftPos = leftFoot->getGlobalTranslation() + leftHeight;
		tgtLeft.setLocalTranslation(leftPos);

		m_pSkeleton->update();

		m_IKController->IKSolver_Limb(m_IKController->mLfootID, tgtLeft);
	}
	if (rotateRight)
	{
		ATarget tgtRight = ATarget();

		vec3 rightPos = rightFoot->getGlobalTranslation() + rightHeight;
		tgtRight.setLocalTranslation(rightPos);

		m_pSkeleton->update();
		m_IKController->IKSolver_Limb(m_IKController->mRfootID, tgtRight);
	}

	vec3 col1 = leftFoot->getLocalRotation()[0];
	vec3 col3 = leftNormal.Cross(col1);
	mat3 rotLeft = mat3(col1.Normalize(), leftNormal, col3).Transpose();
	leftFoot->setLocalRotation(rotLeft);

	col1 = rightFoot->getLocalRotation()[0];
	col3 = rightNormal.Cross(col1);
	mat3 rotRight = mat3(col1.Normalize(), rightNormal, col3).Transpose();
	rightFoot->setLocalRotation(rotRight);

	m_pSkeleton->update();
}
