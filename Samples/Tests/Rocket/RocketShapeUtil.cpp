
#include <Tests/Rocket/RocketShapeUtil.h>

/// Build a convex-hull vaugely shaped like missile
RefConst<Shape> RocketShapeUtil::CreateMissileShape(const Vec3& inDimensions)
{
	Array<Vec3> points;

	// +Z is Forward, +Y is Up, +X is Left

	// Nose
	points.push_back(Vec3( 0.0f,  0.0f, 0.5f));
	points.push_back(Vec3( 0.1f,  0.3f, 0.4f));
	points.push_back(Vec3(-0.1f,  0.3f, 0.4f));
	points.push_back(Vec3( 0.1f, -0.3f, 0.4f));
	points.push_back(Vec3(-0.1f, -0.3f, 0.4f));

	// Fusilage
	points.push_back(Vec3(0.0f,  0.5f, -0.3f));

	// tail
	points.push_back(Vec3(0.0f,  0.0f, -0.5f));

	// wing tips
	points.push_back(Vec3( 0.5f,  0.0f, -0.4f));
	points.push_back(Vec3(-0.5f,  0.0f, -0.4f));

	for (Array<Vec3>::iterator itr = points.begin(); itr < points.end(); ++itr)
	{
		(*itr) = (*itr) * inDimensions;
	}

	return ConvexHullShapeSettings(points).Create().Get();
}
