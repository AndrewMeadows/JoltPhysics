// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <algorithm>
#include <Tests/Rocket/RocketController.h>
#include <Jolt/Physics/Body/Body.h>

constexpr float GRAVITY_MAGNITUDE = 9.8f;

RocketController::RocketController(Body* inBody)
	:	mBody(inBody)
{
	mMassProperties = mBody->GetShape()->GetMassProperties();
	// configure the MotionProperties
	MotionProperties* motion_properties = mBody->GetMotionProperties();
	motion_properties->SetLinearDamping(0.0f); // we use custom damping
	constexpr float MAX_LINEAR_SPEED = 125.0f; // 450 km/hr
	motion_properties->SetMaxLinearVelocity(MAX_LINEAR_SPEED);
	constexpr float MAX_ANGULAR_SPEED = 4.0f * JPH_PI; // 2 rev/sec
	motion_properties->SetMaxAngularVelocity(MAX_ANGULAR_SPEED);

	// assume zero-gravity for now
	motion_properties->SetGravityFactor(0.0f);

	UpdateDampingModel();
	// HACK: high angular damping
	motion_properties->SetAngularDamping(0.8f);
}

RocketController::~RocketController()
{
	mBody = nullptr;
}

void RocketController::SetDeltaToTarget(const Quat &inDeltaToTarget)
{
	mLastDeltaToTarget = inDeltaToTarget;
	mTimeSinceInput = 0.0f;
}

void RocketController::OnStep(const PhysicsStepListenerContext &inContext)
{
	// split world-frame linear velocity into forward and side components relative to rocket fwd
	Vec3 fwd = mBody->GetRotation().RotateAxisZ();		// world-frame
	Vec3 velocity = mBody->GetLinearVelocity();
	Vec3 fwd_velocity = velocity.Dot(fwd) * fwd;
	Vec3 side_velocity = velocity - fwd_velocity;

	// before we apply the linear motor we apply damping and we use a second-order anisotropic damping model:
	// (a) for the component parallel to fwd it is low but second-order function of speed...
	float speed = fwd_velocity.Length();
	constexpr float ALMOST_ZERO = 1.0e-3f; // avoid the divide-by-Very-Small number problem
	if (speed > ALMOST_ZERO)
	{
		float damping_force = mFirstOrderLinearDamping * speed + mSecondOrderLinearDamping * speed * speed;
		mBody->AddImpulse((- damping_force * inContext.mDeltaTime) * fwd);
	}

	// ...whereas (b) for the perpendicular component it is relatively high and uses simple exponential decay,
	// Note: we perform a velocity-space adjustment which models critical damping
	// TODO: make side damping timescale a per-rocket configurable parameter
	constexpr float SIDE_DAMPING_TIMESCALE = 0.4f; // exponential decay reaches 95% after three timescales
	Vec3 delta_velocity = (- inContext.mDeltaTime / SIDE_DAMPING_TIMESCALE) * side_velocity;
	mBody->AddImpulse(mMassProperties.mMass * delta_velocity);

	// Note: for angular damping we currently rely on the one from JoltPhysics engine
	// which is (according to the comments) critically damped exponential

	if (mLinearThrottle != 0.0f)
	{
		// Note:
		// linear_impulse = mass * delta_velocity
		//                = mass * (throttle * thrust_ratio * g * delta_time)

		// Note: multiplicaton operator on Vec3 is component-wise
		float delta_speed = mLinearThrottle * mMaxThrustRatio * GRAVITY_MAGNITUDE * inContext.mDeltaTime;

		// thrust is in the local-frame and always points parallel to fwd
		// but we need to apply delta_impulse in the world-frame
		Vec3 delta_velocity = delta_speed * fwd;
		mBody->AddImpulse(mMassProperties.mMass * delta_velocity);
	}

	if (mAngularThrottle != Vec3::sZero())
	{
		// similar to the linear case:
		// angular_impulse = inertia_tensor * delta_angular_velocity
		//                 = inertia_tensor * (mAngularThrottle * mMaxAngularAccelration * delta_time)
		// Note: multiplicaton operator on Vec3 is component-wise
		Vec3 angular_impulse = mMassProperties.mInertia * (inContext.mDeltaTime * mAngularThrottle * mMaxAngularAcceleration);
		mBody->AddAngularImpulse(mBody->GetRotation() * angular_impulse);
	}
}

void RocketController::SetMaxThrustRatio(float inThrust)
{
	mMaxThrustRatio = inThrust;
	UpdateDampingModel();
}

void RocketController::SetMaxYawAndPitchAcceleration(float inAccel)
{
	mMaxAngularAcceleration.SetX(inAccel);		// pitch
	mMaxAngularAcceleration.SetY(inAccel);		// yaw
}

float RocketController::GetMaxYawAndPitchAcceleration() const
{
	return mMaxAngularAcceleration.GetX();
}

void RocketController::SetMaxCruisingSpeed(float inSpeed)
{
	mMaxCruisingSpeed = inSpeed;
	UpdateDampingModel();
}

void RocketController::SetGravityFactor(float inFactor )
{
	MotionProperties* mp = mBody->GetMotionProperties();
	mp->SetGravityFactor(inFactor);
}

bool RocketController::GetGravityFactor() const
{
	MotionProperties* mp = mBody->GetMotionProperties();
	return mp->GetGravityFactor();
}

void RocketController::SetAngularThrottle(Vec3 inValue)
{
	mAngularThrottle = Vec3::sClamp(inValue, -Vec3::sOne(), Vec3::sOne());

	/*
	Vec3 old_angular_throttle = mAngularThrottle;
	// for easier steerability we adjust the angular damping according to whether
	// it is being actively steered (low damping) or not (high damping)
	if (mAngularThrottle != old_angular_throttle)
	{
		constexpr float LOW_DAMPING = 0.0f;
		constexpr float HIGH_DAMPING = 0.5f;
		float damping = mAngularThrottle == Vec3::sZero() ? HIGH_DAMPING : LOW_DAMPING;

		// angular damping: dw/dt = -c * w. c must be between 0 and 1 but is usually close to 0.
		MotionProperties* motion_properties = mBody->GetMotionProperties();
		motion_properties->SetAngularDamping(damping);
	}
	*/
}

void RocketController::SetLinearThrottle(float inValue)
{
	mLinearThrottle = Clamp(inValue, 0.0f, 1.0f);
}

void RocketController::SetThrottleInputs(float inLinearThrottle, const Vec3& inAngularThrottle)
{
	mLinearThrottle = Clamp(inLinearThrottle, 0.0f, 1.0f);
	mAngularThrottle = Vec3::sClamp(inAngularThrottle, -Vec3::sOne(), Vec3::sOne());
}

void RocketController::UpdateDampingModel()
{
	if (!mBody)
	{
		// can't proceed unless we know the mass of the body
		return;
	}

	// The second-order equation for damping is:
	//
	//     damping_force_magnitude = A * V + B * V^2
	//
	// Where A and B are some constants and V is linear speed.
	// Given some max_thrust and max_cruising_speed there exists an (A,B) coefficient pair that is constent.
	// We can compute the coefficients if we have at least three points on the quadratic curve.
	// Two of the points are trivial: V = 0 and V = max_cruising_speed, but the third requires measurement
	// or assumption.  Since we don't have measured data we will proceed with assumption:
	// at 1/2 max_cruising_speed the damping_force_magnitude is 1/3 max.
	//
	// The derivation of the equations for A and B are left as an exercise for the reader.

	float mass = mMassProperties.mMass;
	float a = mMaxCruisingSpeed;
	float b = mMaxCruisingSpeed * mMaxCruisingSpeed;
	float c = mMaxThrustRatio * GRAVITY_MAGNITUDE * mass;

	float d = 0.5f * a;
	float e = 0.25f * b;
	float f = 0.333333f * c;

	mSecondOrderLinearDamping = (d * c - a * f) / (d * b - a * e);
	mFirstOrderLinearDamping = (c - b * mSecondOrderLinearDamping) / a;
}
