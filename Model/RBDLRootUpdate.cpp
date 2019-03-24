#include <stdexcept>
#include "Model/RBDLRootUpdate.h"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

namespace Leph
{
/**
 * Return body string name with given
 * id (fixed body with added discriminator)
 */
static std::string bodyName(RBDL::Model& model, size_t bodyId)
{
  if (bodyId == 0)
  {
    return "OLDROOT_link";
  }
  for (const auto& body : model.mBodyNameMap)
  {
    if (body.second == bodyId)
    {
      return body.first;
    }
  }
  throw std::logic_error("RBDLRootUpdate unable to find body name");
}

/**
 * Build root updated new model
 * Iterate through all fixed body of
 * given RBDL movable body id
 */
static void rootUpdateFixed(RBDL::Model& modelOld, size_t bodyId, size_t parentId,
                            RBDLMath::SpatialTransform tranformToFrame, RBDL::Model& modelNew, size_t parentIdNewModel)
{
  for (size_t i = 0; i < modelOld.mFixedBodies.size(); i++)
  {
    if (modelOld.mFixedBodies[i].mMovableParent == bodyId && i + modelOld.fixed_body_discriminator != parentId)
    {
      // Build new body and copy from old model
      // Massless fixed body (already take into
      // account in movable parent)
      RBDL::Body body = RBDL::Body(0.0, RBDLMath::Vector3d(0.0, 0.0, 0.0), RBDLMath::Vector3d(0.0, 0.0, 0.0));
      RBDL::Joint joint =
          RBDL::Joint(RBDL::JointTypeFixed, bodyName(modelOld, i + modelOld.fixed_body_discriminator) + "_fixed");
      modelNew.AddBody(parentIdNewModel, tranformToFrame * modelOld.mFixedBodies[i].mParentTransform, joint, body,
                       bodyName(modelOld, i + modelOld.fixed_body_discriminator));
    }
  }
}

/**
 * Build root updated new model
 * Recursively iterate RBDL tree structure
 * from parent to children
 */
static void rootUpdateForward(RBDL::Model& modelOld, size_t bodyId, size_t parentId,
                              RBDLMath::SpatialTransform transformToFrame, RBDL::Model& modelNew,
                              size_t parentIdNewModel)
{
  // copy body inertia
  RBDL::Body body = RBDL::Body(modelOld.mBodies[bodyId].mMass, modelOld.mBodies[bodyId].mCenterOfMass,
                               modelOld.mBodies[bodyId].mInertia);
  RBDL::Joint joint(modelOld.mJoints[bodyId]);
  // Manually compute the transformation from last
  // frame to current frame.
  // Last parent frame and the next currently buit frame
  // are both a children of root in old model.
  // rotation (E) and translation (t).
  RBDLMath::SpatialTransform transform = modelOld.X_T[bodyId];
  transform.E = transform.E * transformToFrame.E;
  transform.r = -transformToFrame.r + transform.r;
  transform.r = transformToFrame.E * transform.r;
  size_t bodyIdNewModel = modelNew.AddBody(parentIdNewModel, transform, joint, body, bodyName(modelOld, bodyId));

  // Iterate through fixed body
  // of current movable body
  rootUpdateFixed(modelOld, bodyId, parentId, transformToFrame, modelNew, bodyIdNewModel);

  // Iterate through children
  for (size_t i = 0; i < modelOld.mu[bodyId].size(); i++)
  {
    rootUpdateForward(modelOld, modelOld.mu[bodyId][i], bodyId, RBDLMath::Xtrans(RBDLMath::Vector3d(0.0, 0.0, 0.0)),
                      modelNew, bodyIdNewModel);
  }
}

/**
 * Build root updated new model
 * Recursively iterate RBDL tree structure
 * from children to parent
 */
static void rootUpdateBackward(RBDL::Model& modelOld, size_t bodyId, size_t parentId,
                               RBDLMath::SpatialTransform tranformToJoint, RBDL::Model& modelNew,
                               size_t parentIdNewModel)
{
  // Compute center of mass position
  // in new frame
  RBDLMath::VectorNd dofs = RBDLMath::VectorNd::Zero(modelOld.dof_count);
  RBDLMath::Vector3d ptComBase =
      RBDL::CalcBodyToBaseCoordinates(modelOld, dofs, bodyId, modelOld.mBodies[bodyId].mCenterOfMass);
  RBDLMath::Vector3d ptComFrame = RBDL::CalcBaseToBodyCoordinates(modelOld, dofs, parentId, ptComBase);
  // Build and inverse tranformation from
  // old model body
  RBDL::Body body = RBDL::Body(modelOld.mBodies[bodyId].mMass, ptComFrame, modelOld.mBodies[bodyId].mInertia);
  // For backward iteration joints axis have to be inverted
  RBDL::Joint joint = modelOld.mJoints[parentId];
  if (joint.mJointType == RBDL::JointTypeRevolute || joint.mJointType == RBDL::JointTypeRevoluteX ||
      joint.mJointType == RBDL::JointTypeRevoluteY || joint.mJointType == RBDL::JointTypeRevoluteZ)
  {
    // If joint axis are inverted, do not forget
    // to update RBDL joint type
    //(May be is there specific optimization here)
    joint.mJointType = RBDL::JointTypeRevolute;
    joint.mJointAxes[0].head(3) *= -1;
  }
  else
  {
    throw std::logic_error("RBDLRootUpdate sign update for not implemented joint type");
  }
  size_t bodyIdNewModel =
      modelNew.AddBody(parentIdNewModel, tranformToJoint, joint, body, bodyName(modelOld, parentId));

  // Iterate through fixed body
  // of current movable mody
  rootUpdateFixed(modelOld, bodyId, parentId, modelOld.X_T[parentId].inverse(), modelNew, bodyIdNewModel);

  // Iterate though parent if no root
  if (bodyId != 0)
  {
    rootUpdateBackward(modelOld, modelOld.lambda[bodyId], bodyId, modelOld.X_T[parentId].inverse(), modelNew,
                       bodyIdNewModel);
  }
  // Iterate through children
  for (size_t i = 0; i < modelOld.mu[bodyId].size(); i++)
  {
    if (modelOld.mu[bodyId][i] != parentId)
    {
      rootUpdateForward(modelOld, modelOld.mu[bodyId][i], bodyId, modelOld.X_T[parentId], modelNew, bodyIdNewModel);
    }
  }
}

RBDL::Model RBDLRootUpdate(RBDL::Model& modelOld, size_t newRootBodyId, bool addFloatingBase)
{
  // Retrieve the body id of movable parent
  // in case of fixed body
  size_t newRootBodyMovableId = newRootBodyId;
  RBDLMath::SpatialTransform transformToBody = RBDLMath::Xtrans(RBDLMath::Vector3d(0.0, 0.0, 0.0));
  if (newRootBodyId >= modelOld.fixed_body_discriminator)
  {
    size_t fixedId = newRootBodyId - modelOld.fixed_body_discriminator;
    newRootBodyMovableId = modelOld.mFixedBodies[fixedId].mMovableParent;
    transformToBody = modelOld.mFixedBodies[fixedId].mParentTransform.inverse();
  }

  // Compute center of mass position
  // in movable body frame
  RBDLMath::VectorNd dofs = RBDLMath::VectorNd::Zero(modelOld.dof_count);
  RBDLMath::Vector3d ptComBase = RBDL::CalcBodyToBaseCoordinates(modelOld, dofs, newRootBodyMovableId,
                                                                 modelOld.mBodies[newRootBodyMovableId].mCenterOfMass);
  RBDLMath::Vector3d ptComFrame = RBDL::CalcBaseToBodyCoordinates(modelOld, dofs, newRootBodyId, ptComBase);

  // Initialize new model with new root
  // created as fixed body of RBDL root
  RBDL::Model modelNew;

  // Select fixed base or floating base
  RBDL::Joint jointRoot;
  if (addFloatingBase)
  {
    jointRoot = RBDL::Joint(
        RBDLMath::SpatialVector(0.0, 0.0, 0.0, 1.0, 0.0, 0.0), RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), RBDLMath::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        RBDLMath::SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0), RBDLMath::SpatialVector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "base");
  }
  else
  {
    jointRoot = RBDL::Joint(RBDL::JointTypeFixed, "root");
  }

  RBDL::Body bodyRoot = RBDL::Body(modelOld.mBodies[newRootBodyMovableId].mMass, ptComFrame,
                                   modelOld.mBodies[newRootBodyMovableId].mInertia);
  size_t rootIdNewModel =
      modelNew.AddBody(0, RBDLMath::Xtrans(RBDLMath::Vector3d(0.0, 0.0, 0.0)), jointRoot, bodyRoot, "base_link");

  // Iterate on all other fixed bodies
  rootUpdateFixed(modelOld, newRootBodyMovableId, newRootBodyMovableId, transformToBody, modelNew, rootIdNewModel);
  // Iterate on all new root children
  for (size_t i = 0; i < modelOld.mu[newRootBodyMovableId].size(); i++)
  {
    rootUpdateForward(modelOld, modelOld.mu[newRootBodyMovableId][i], newRootBodyMovableId,
                      RBDLMath::Xtrans(RBDLMath::Vector3d(0.0, 0.0, 0.0)), modelNew, rootIdNewModel);
  }
  // Iterate on new root parent
  if (modelOld.lambda[newRootBodyMovableId] != 0)
  {
    rootUpdateBackward(modelOld, modelOld.lambda[newRootBodyMovableId], newRootBodyMovableId, transformToBody, modelNew,
                       rootIdNewModel);
  }

  // Set gravity
  modelNew.gravity = RBDLMath::Vector3d(0.0, 0.0, -9.81);

  return modelNew;
}

}  // namespace Leph
