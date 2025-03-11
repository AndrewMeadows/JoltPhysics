// RocketShapeUtil.h
//

#pragma once

#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

namespace RocketShapeUtil
{
	/// Build a convex-hull vaugely shaped like missile
	/// @param inDimensions bounding box of final shape
	RefConst<Shape> CreateMissileShape(const Vec3& inDimensions);
};
