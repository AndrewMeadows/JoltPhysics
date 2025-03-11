// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/Body/Body.h>

/*
class JPH_EXPORT PhysicsStepListenerContext
{
public:
	float					mDeltaTime;								///< Delta time of the current step
	bool					mIsFirstStep;							///< True if this is the first step
	bool					mIsLastStep;							///< True if this is the last step
	PhysicsSystem *			mPhysicsSystem;							///< The physics system that is being stepped
};

/// A listener class that receives a callback before every physics simulation step
class JPH_EXPORT PhysicsStepListener
{
public:
	/// Ensure virtual destructor
	virtual					~PhysicsStepListener() = default;

	/// Called before every simulation step (received inCollisionSteps times for every PhysicsSystem::Update(...) call)
	/// This is called while all body and constraint mutexes are locked. You can read/write bodies and constraints but not add/remove them.
	/// Multiple listeners can be executed in parallel and it is the responsibility of the listener to avoid race conditions.
	/// The best way to do this is to have each step listener operate on a subset of the bodies and constraints
	/// and making sure that these bodies and constraints are not touched by any other step listener.
	/// Note that this function is not called if there aren't any active bodies or when the physics system is updated with 0 delta time.
	virtual void			OnStep(const PhysicsStepListenerContext &inContext) = 0;
};
*/

class RocketController : public PhysicsStepListener
{
public:
	RocketController(Body* inBody);

	virtual ~RocketController();

	/// OnStep() is called right before each simulation substep and is where all thrust and torque influences are applied.
	virtual void OnStep(const PhysicsStepListenerContext &inContext) override;

	/// Set input from data acquisition
	/// @param inDeltaToTarget Rotation in local-frame that would align target to forward
	void SetDeltaToTarget(const Quat &inDeltaToTarget);

	void SetMaxThrustRatio(float inThrust);
	float GetMaxThrustRatio() const { return mMaxThrustRatio; }

	void SetMaxYawAndPitchAcceleration(float inAccel);
	float GetMaxYawAndPitchAcceleration() const;

	void SetMaxCruisingSpeed(float inSpeed);
	float GetMaxCruisingSpeed() const { return mMaxCruisingSpeed; }

	/// @param inFactor 1 is normal gravity, 0 is no gravity
	void SetGravityFactor(float inFactor );
	bool GetGravityFactor() const;

	/// @param inValue angular throttle inputs {yaw, pitch, roll} each in range [-1, 1]. Will be scaled by max angular accelerations
	void SetAngularThrottle(Vec3 inValue);
	Vec3 GetAngularThrottle() const { return mAngularThrottle; }

	/// @param inValue linear motor throttle input in range [0, 1]
	void SetLinearThrottle(float inValue);
	float GetLinearThrottle() const { return mLinearThrottle; }

	///
	virtual void SetThrottleInputs(float inLinearThrottle, const Vec3& inAngularThrottle);
private:
	/// To make mMaxThrustRatio and mMaxCruisingSpeed agree with each other we compute a second-order damping model
	/// to use during the integration step.
	virtual void UpdateDampingModel();

protected:
	Body * mBody { nullptr };			///< pointer to Body
	MassProperties mMassProperties;		///< Cached in ctor

	// rocket motor parameters
	float mMaxThrustRatio { 5.0f };							///< Max thrust/weight ratio when under full motor (max G's of acceleration)
	Vec3 mMaxAngularAcceleration { 2.0f * JPH_PI, 2.0f * JPH_PI, 0.5f * JPH_PI };	///< { pitch, yaw, roll }

	float mMaxCruisingSpeed { 60.0f };				///< meters/sec

	// we use a 2nd-order damping model that tries to agree with mMaxCruiseSpeed and mMaxThrustRatio
	// (see UpdateDampingModel() implementation for details)
	float mFirstOrderLinearDamping { 0.0f };
	float mSecondOrderLinearDamping { 0.0f };

	// rocket steering input from sensor: rotation to make
	// TODO: finish implementingn this
	Quat mLastDeltaToTarget { 0.0f, 0.0f, 0.0f, 1.0f };		///< last input from sensor: delta rotation to align target to forward

	// alternative steering input from driver: angular acceleration
	Vec3 mAngularThrottle { 0.0f, 0.0f, 0.0f };				///< direction with magnitude in range [0, 1]

	// steering input: forward throttle
	float mLinearThrottle { 0.0f };							///< forward throttle in range [0, 1]

	// rocket state: estimated
	// Until the next input from measurement the rocket controller can only estimate its orientation and the target's delta
	Quat mEstimatedDeltaToTarget { 0.0f, 0.0f, 0.0f, 1.0f };	///< estimated delta rotation that would align target to forward
	Vec3 mEstimatedAngularVelocity { 0.0f, 0.0f, 0.0f };		///< estimated angular velocity
	float mTimeSinceInput { 0.0f };								///< simulation time since mLastDeltaToTarget was set
};
