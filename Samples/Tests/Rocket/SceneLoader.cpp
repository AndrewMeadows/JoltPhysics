// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/Rocket/SceneLoader.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Layers.h>
#include <Application/DebugUI.h>
#include <Utils/Log.h>
#include <Utils/AssetStream.h>
#include <Renderer/DebugRendererImp.h>

JPH_IMPLEMENT_RTTI_VIRTUAL(SceneLoader)
{
	JPH_ADD_BASE_CLASS(SceneLoader, Test)
}

const char *SceneLoader::sScenes[] =
{
	"Flat",
	"Flat With Slope",
	"Steep Slope",
	"Step",
	"Dynamic Step",
	"Loop",
#ifdef JPH_OBJECT_STREAM
	"Terrain1",
#endif // JPH_OBJECT_STREAM
};

const char *SceneLoader::sSceneName = "Flat";

void SceneLoader::Initialize()
{
	if (strcmp(sSceneName, "Flat") == 0)
	{
		float box_width = 200.0f;
		float box_height = 1.0f;
		int num_boxes_per_side = 30;
		float y = 0.5f * box_height;
		float x = -0.5f * box_width * num_boxes_per_side;
		Vec3 dimensions = Vec3(box_width, box_height, box_width);
		for (int i = 0; i < num_boxes_per_side; ++i)
		{
			float z = -0.5f * box_width * num_boxes_per_side;
			for (int j = 0; j < num_boxes_per_side; ++j)
			{
				Vec3 position(x, y, z);
				z += box_width;
				// Flat test floor
				Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(dimensions, 0.0f), position, Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
				floor.SetFriction(1.0f);
				mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);
			}
			x += box_width;
		}

		// Load a race track to have something to assess speed and steering behavior
		LoadRaceTrack("Racetracks/Zandvoort.csv");
	}
	else if (strcmp(sSceneName, "Flat With Slope") == 0)
	{
		const float cSlopeStartDistance = 100.0f;
		const float cSlopeLength = 100.0f;
		const float cSlopeAngle = DegreesToRadians(30.0f);

		// Flat test floor
		Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(1000.0f, 1.0f, 1000.0f), 0.0f), RVec3(0.0f, -1.0f, 0.0f), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
		floor.SetFriction(1.0f);
		mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);

		Body &slope_up = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(25.0f, 1.0f, cSlopeLength), 0.0f), RVec3(0.0f, cSlopeLength * Sin(cSlopeAngle) - 1.0f, cSlopeStartDistance + cSlopeLength * Cos(cSlopeAngle)), Quat::sRotation(Vec3::sAxisX(), -cSlopeAngle), EMotionType::Static, Layers::NON_MOVING));
		slope_up.SetFriction(1.0f);
		mBodyInterface->AddBody(slope_up.GetID(), EActivation::DontActivate);

		Body &slope_down = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(25.0f, 1.0f, cSlopeLength), 0.0f), RVec3(0.0f, cSlopeLength * Sin(cSlopeAngle) - 1.0f, cSlopeStartDistance + 3.0f * cSlopeLength * Cos(cSlopeAngle)), Quat::sRotation(Vec3::sAxisX(), cSlopeAngle), EMotionType::Static, Layers::NON_MOVING));
		slope_down.SetFriction(1.0f);
		mBodyInterface->AddBody(slope_down.GetID(), EActivation::DontActivate);
	}
	else if (strcmp(sSceneName, "Steep Slope") == 0)
	{
		// Steep slope test floor (20 degrees = 36% grade)
		Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(1000.0f, 1.0f, 1000.0f), 0.0f), RVec3(0.0f, -1.0f, 0.0f), Quat::sRotation(Vec3::sAxisX(), DegreesToRadians(-20.0f)), EMotionType::Static, Layers::NON_MOVING));
		floor.SetFriction(1.0f);
		mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);
	}
	else if (strcmp(sSceneName, "Step") == 0)
	{
		// Flat test floor
		Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(1000.0f, 1.0f, 1000.0f), 0.0f), RVec3(0.0f, -1.0f, 0.0f), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
		floor.SetFriction(1.0f);
		mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);

		// A 5cm step rotated under an angle
		constexpr float cStepHeight = 0.05f;
		Body &step = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(5.0f, 0.5f * cStepHeight, 5.0f), 0.0f), RVec3(-2.0f, 0.5f * cStepHeight, 60.0f), Quat::sRotation(Vec3::sAxisY(), -0.3f * JPH_PI), EMotionType::Static, Layers::NON_MOVING));
		step.SetFriction(1.0f);
		mBodyInterface->AddBody(step.GetID(), EActivation::DontActivate);
	}
	else if (strcmp(sSceneName, "Dynamic Step") == 0)
	{
		// Flat test floor
		Body &floor = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(1000.0f, 1.0f, 1000.0f), 0.0f), RVec3(0.0f, -1.0f, 0.0f), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
		floor.SetFriction(1.0f);
		mBodyInterface->AddBody(floor.GetID(), EActivation::DontActivate);

		// A dynamic body that acts as a step to test sleeping behavior
		constexpr float cStepHeight = 0.05f;
		Body &step = *mBodyInterface->CreateBody(BodyCreationSettings(new BoxShape(Vec3(15.0f, 0.5f * cStepHeight, 15.0f), 0.0f), RVec3(-2.0f, 0.5f * cStepHeight, 30.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING));
		step.SetFriction(1.0f);
		mBodyInterface->AddBody(step.GetID(), EActivation::Activate);
	}
	else if (strcmp(sSceneName, "Loop") == 0)
	{
		CreateFloor();

		TriangleList triangles;
		const int cNumSegments = 100;
		const float cLoopWidth = 20.0f;
		const float cLoopRadius = 20.0f;
		const float cLoopThickness = 0.5f;
		Vec3 prev_center = Vec3::sZero();
		Vec3 prev_center_bottom = Vec3::sZero();
		for (int i = 0; i < cNumSegments; ++i)
		{
			float angle = i * 2.0f * JPH_PI / (cNumSegments - 1);
			Vec3 radial(0, -Cos(angle), Sin(angle));
			Vec3 center = Vec3(-i * cLoopWidth / (cNumSegments - 1), cLoopRadius, cLoopRadius) + cLoopRadius * radial;
			Vec3 half_width(0.5f * cLoopWidth, 0, 0);
			Vec3 center_bottom = center + cLoopThickness * radial;
			if (i > 0)
			{
				// Top surface
				triangles.push_back(Triangle(prev_center + half_width, prev_center - half_width, center - half_width));
				triangles.push_back(Triangle(prev_center + half_width, center - half_width, center + half_width));

				// Bottom surface
				triangles.push_back(Triangle(prev_center_bottom + half_width, center_bottom - half_width, prev_center_bottom - half_width));
				triangles.push_back(Triangle(prev_center_bottom + half_width, center_bottom + half_width, center_bottom - half_width));

				// Sides
				triangles.push_back(Triangle(prev_center + half_width, center + half_width, prev_center_bottom + half_width));
				triangles.push_back(Triangle(prev_center_bottom + half_width, center + half_width, center_bottom + half_width));
				triangles.push_back(Triangle(prev_center - half_width, prev_center_bottom - half_width, center - half_width));
				triangles.push_back(Triangle(prev_center_bottom - half_width, center_bottom - half_width, center - half_width));
			}
			prev_center = center;
			prev_center_bottom = center_bottom;
		}
		MeshShapeSettings mesh(triangles);
		mesh.SetEmbedded();

		Body &loop = *mBodyInterface->CreateBody(BodyCreationSettings(&mesh, RVec3::sZero(), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING));
		loop.SetFriction(1.0f);
		mBodyInterface->AddBody(loop.GetID(), EActivation::Activate);
	}
#ifdef JPH_OBJECT_STREAM
	else
	{
		// Load scene
		Ref<PhysicsScene> scene;
		AssetStream stream(String(sSceneName) + ".bof", std::ios::in | std::ios::binary);
		if (!ObjectStreamIn::sReadObject(stream.Get(), scene))
			FatalError("Failed to load scene");
		for (BodyCreationSettings &body : scene->GetBodies())
			body.mObjectLayer = Layers::NON_MOVING;
		scene->FixInvalidScales();
		scene->CreateBodies(mPhysicsSystem);
	}
#endif // JPH_OBJECT_STREAM
}

void SceneLoader::LoadRaceTrack(const char *inFileName)
{
	// Open the track file
	AssetStream asset_stream(inFileName, std::ios::in);
	std::istream &stream = asset_stream.Get();

	// Ignore header line
	String line;
	std::getline(stream, line);

	// Read coordinates
	struct Segment
	{
		RVec3				mCenter;
		float				mWidthLeft;
		float				mWidthRight;
	};
	Array<Segment> segments;
	Real x, y;
	float wl, wr;
	char c;
	RVec3 track_center = RVec3::sZero();
	while (stream >> x >> c >> y >> c >> wl >> c >> wr)
	{
		RVec3 center(x, 0, -y);
		segments.push_back({ center, wl, wr });
		track_center += center;
	}
	if (!segments.empty())
		track_center /= (float)segments.size();

	// Convert to line segments
	RVec3 prev_tleft = RVec3::sZero(), prev_tright = RVec3::sZero();
	for (size_t i = 0; i < segments.size(); ++i)
	{
		const Segment &segment = segments[i];
		const Segment &next_segment = segments[(i + 1) % segments.size()];

		// Calculate left and right point of the track
		Vec3 fwd = Vec3(next_segment.mCenter - segment.mCenter);
		Vec3 right = fwd.Cross(Vec3::sAxisY()).Normalized();
		RVec3 tcenter = segment.mCenter - track_center + Vec3(0, 0.1f, 0); // Put a bit above the floor to avoid z fighting
		RVec3 tleft = tcenter - right * segment.mWidthLeft;
		RVec3 tright = tcenter + right * segment.mWidthRight;
		mTrackData.push_back({ tleft, tright });

		// Connect left and right point with the previous left and right point
		if (i > 0)
		{
			mTrackData.push_back({ prev_tleft, tleft });
			mTrackData.push_back({ prev_tright, tright });
		}

		prev_tleft = tleft;
		prev_tright = tright;
	}
}

void SceneLoader::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	// Render the track
	for (const Line &l : mTrackData)
		mDebugRenderer->DrawLine(l.mStart, l.mEnd, Color::sBlack);
}

void SceneLoader::CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu)
{
	inUI->CreateTextButton(inSubMenu, "Select Scene", [this, inUI]() {
		UIElement *scene_name = inUI->CreateMenu();
		for (uint i = 0; i < size(sScenes); ++i)
			inUI->CreateTextButton(scene_name, sScenes[i], [this, i]() { sSceneName = sScenes[i]; RestartTest(); });
		inUI->ShowMenu(scene_name);
	});
}

