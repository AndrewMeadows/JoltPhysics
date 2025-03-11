// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/Rocket/RocketTest.h>

#include <Tests/Rocket/RocketShapeUtil.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Application/DebugUI.h>
#include <Layers.h>
#include <Renderer/DebugRendererImp.h>

static bool sFooValue = false; // placeholder for UI checkbox example

JPH_IMPLEMENT_RTTI_VIRTUAL(RocketTest)
{
	JPH_ADD_BASE_CLASS(RocketTest, SceneLoader)
}

RocketTest::RocketTest()
{
    // start with non-zero forward throttle
    mForward = 0.75f;
}

RocketTest::~RocketTest()
{
	mPhysicsSystem->RemoveStepListener(mMissile);
	mMissile = nullptr;

	for (RocketList::const_iterator itr = mRockets.begin(); itr < mRockets.end(); ++itr)
	{
		mPhysicsSystem->RemoveStepListener(*itr);
	}
	mRockets.clear();
}

void RocketTest::Initialize()
{
	SceneLoader::Initialize();

	// Create missile
	const float rocket_length = 3.5f; // X
	const float rocket_width = 2.8f;
	const float rocket_height = 0.5f;
	Vec3 dimensions(rocket_width, rocket_height, rocket_length);

	Vec3 position(0.0f, 20.0f, 0.0f);

	RefConst<Shape> missile_shape = RocketShapeUtil::CreateMissileShape(dimensions);
	BodyCreationSettings missile_body_settings(missile_shape, position, Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);

	mMissileBody = mBodyInterface->CreateBody(missile_body_settings);
	mBodyInterface->AddBody(mMissileBody->GetID(), EActivation::Activate);

	// Add missile to system
	mMissile = new MissileController(mMissileBody);
	mPhysicsSystem->AddStepListener(mMissile);

	constexpr float MISSLE_START_SPEED = 80.0f; // m/sec
	Vec3 fwd = mMissileBody->GetRotation().RotateAxisZ();
	mMissileBody->GetMotionProperties()->SetLinearVelocity(MISSLE_START_SPEED * fwd);
	mMissile->SetLinearThrottle(0.5f);

	UpdateCameraPivot();
}

void RocketTest::ProcessInput(const ProcessInputParams &inParams)
{
	// Steering
	float left = 0.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Left))
		left = 1.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Right))
		left = -1.0f;

	float up = 0.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Up))
		up += 1.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Down))
		up -= 1.0f;

	// Throttle
	float forward = 0.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Num1))
		forward -= 1.0f;
	if (inParams.mKeyboard->IsKeyPressed(EKey::Num3))
		forward += 1.0f;

	// compute Ramp and Damp slew rates
	const float TIME_TO_RAMP_INPUT = 2.0f; // sec
	const float ramp = inParams.mDeltaTime / TIME_TO_RAMP_INPUT;

	//const float TIME_TO_DAMP_INPUT = 1.0f; // sec
	//const float damp = inParams.mDeltaTime / TIME_TO_DAMP_INPUT;

	// Compute inputs using Ramp and Damp values
	if (left == 0.0f)
	{
		//float sign = (mYaw > 0.0f) ? 1.0 : -1.0f;
		//mYaw = sign * max(0.0f, abs(mYaw) - damp);
		mYaw = 0.0f;
	}
	else
	{
		mYaw = Clamp(mYaw + left * ramp, -1.0f, 1.0f);
	}
	if (up == 0.0f)
	{
		//float sign = (mPitch > 0.0f) ? -1.0 : 1.0f; // negate sign for right-hand-rule about left axis
		//mPitch = sign * max(0.0f, abs(mPitch) - damp);
		mPitch = 0.0f;
	}
	else
	{
		mPitch = Clamp(mPitch + up * ramp, -1.0f, 1.0f);
	}

	// Forward has cruise control: it is only ramped, not damped
	if (forward != 0.0f)
	{
		mForward = Clamp(mForward + forward * ramp, 0.0f, 1.0f);
	}
}

void RocketTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	SceneLoader::PrePhysicsUpdate(inParams);

	UpdateCameraPivot();

	// On user input, assure that the rocket is active
	if (mYaw != 0.0f || mPitch != 0.0f || mRoll != 0.0f || mForward != 0.0f)
		mBodyInterface->ActivateBody(mMissileBody->GetID());

	// Pass the input on to the missile
	mMissile->SetAngularThrottle(Vec3(mPitch, mYaw, mRoll));
	mMissile->SetLinearThrottle(mForward);
}

void RocketTest::SaveInputState(StateRecorder &inStream) const
{
	inStream.Write(mPitch);
	inStream.Write(mYaw);
	inStream.Write(mRoll);
	inStream.Write(mForward);
}

void RocketTest::RestoreInputState(StateRecorder &inStream)
{
	inStream.Read(mPitch);
	inStream.Read(mYaw);
	inStream.Read(mRoll);
	inStream.Read(mForward);
}

void RocketTest::GetInitialCamera(CameraState &ioState) const
{
	// Position camera behind rocket
	RVec3 cam_tgt = RVec3(0, 0, 5);
	ioState.mPos = RVec3(0, 2.5f, -5);
	ioState.mForward = Vec3(cam_tgt - ioState.mPos).Normalized();
}

void RocketTest::UpdateCameraPivot()
{
	// Pivot is center of rocket and rotates with rocket around Y axis only
	Vec3 fwd = mMissileBody->GetRotation().RotateAxisZ();
	fwd.SetY(0.0f);
	float len = fwd.Length();
	if (len != 0.0f)
		fwd /= len;
	else
		fwd = Vec3::sAxisZ();
	Vec3 up = Vec3::sAxisY();
	Vec3 right = up.Cross(fwd);
	mCameraPivot = RMat44(Vec4(right, 0), Vec4(up, 0), Vec4(fwd, 0), mMissileBody->GetPosition());
}

void RocketTest::CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu)
{
	SceneLoader::CreateSettingsMenu(inUI, inSubMenu);

	inUI->CreateCheckBox(inSubMenu, "Set sFooValue", sFooValue, [](UICheckBox::EState inState) { sFooValue = inState == UICheckBox::STATE_CHECKED; });
	inUI->CreateTextButton(inSubMenu, "Accept", [this]() { RestartTest(); });
}
