// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <algorithm>
#include <Tests/Rocket/MissileController.h>
#include <Jolt/Physics/Body/Body.h>

MissileController::MissileController(Body* inBody) : RocketController(inBody)
{
}

MissileController::~MissileController()
{
}

void MissileController::OnStep(const PhysicsStepListenerContext &inContext)
{
	// Missile uses a velocity model for control: it is simpler but less physically correct.

	Vec3 fwd = mBody->GetRotation().RotateAxisZ(); // world-frame

	// linear motor
	Quat rotation = mBody->GetRotation();
	Vec3 velocity = mBody->GetLinearVelocity();
	Vec3 target_velocity = (mLinearThrottle * mMaxCruisingSpeed) * fwd;
	Vec3 delta_velocity = target_velocity - velocity;
	float tau = inContext.mDeltaTime / mLinearMotorTimescale;
	velocity += tau * delta_velocity;

	// anisotropic damping
	Vec3 fwd_velocity = velocity.Dot(fwd) * fwd;
	Vec3 side_velocity = velocity - fwd_velocity;
	float side_speed = side_velocity.Length();
	if (side_speed > 1.0f)
	{
		side_velocity *= (1.0f - tau);
		velocity = fwd_velocity + side_velocity;
	}

	// adjust linear velocity
	mBody->SetLinearVelocity(velocity);

	// angular motor
	velocity = mBody->GetAngularVelocity();
	target_velocity = mMaxAngularSpeed * (rotation * mAngularThrottle);
	delta_velocity = target_velocity - velocity;
	tau = inContext.mDeltaTime / mAngularMotorTimescale;
	velocity += tau * delta_velocity;

	if (mAngularThrottle == Vec3::sZero())
	{
		// vertical attractor helps pull Missile upright when not being steered
		Vec3 up = rotation.RotateAxisY();
		Vec3 axis = up.Cross(Vec3::sAxisY());
		float axis_length = axis.Length();
		if (axis_length > 0.01f)
		{
			velocity += tau * axis;
		}
	}

	// adjust angular velocity
	mBody->SetAngularVelocity(velocity);
}

void MissileController::UpdateDampingModel()
{
}
