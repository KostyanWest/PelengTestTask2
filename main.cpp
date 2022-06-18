#include "projector.hpp"

#include <iostream>
#include <iomanip>



static constexpr float pi = 3.14159265358979323846f;

static void TestPng()
{
	std::cout << std::fixed << std::setprecision( 6 );
	Projector proj;
	// 1687x949px, 75° FOV
	proj.Setup( 1687, 949, std::atan( 1687.0f / (949.0f / std::tan( pi / 2.4f / 2 )) ) * 2, pi / 2.4f );
	// pitch -37°, yaw 10°, 2m height
	proj.Transform( -pi / 180.0f * 37.0f, pi / 18.0f, 2.0f );
	for (const Vec4& in : {
			Vec4{ -1, 0, -2, 1 },
			Vec4{ -1, 0, -1, 1 },
			Vec4{ -2, 0, -1, 1 },
			Vec4{ -1, 1, -2, 1 },
			Vec4{ -1, 1, -1, 1 },
			Vec4{ -2, 1, -1, 1 },
			Vec4{ -2, 1, -2, 1 },
		})
	{
		Vec4 out = proj.FromWorldToViewport( in );
		std::cout << out.vec[0] << ' ' << out.vec[1] << ' ' << out.vec[2] << std::endl;
	}

	proj.PlanePoints(
		{ -1, 0, -3, 1 },
		{ -2, 0, -4, 1 },
		{ +1, 0, -2, 1 }
	);
	for (const Vec2& in : {
		Vec2{ 708, 539 },
		Vec2{ 608, 735 },
		Vec2{ 353, 690 },
		})
	{
		Vec4 out = proj.FromViewportToWorldPlane( in.vec[0], in.vec[1] );
		std::cout << out.vec[0] << ' ' << out.vec[1] << ' ' << -out.vec[2] << std::endl;
	}

	proj.PlanePoints(
		{ -1, 1, -3, 1 },
		{ -2, 1, -4, 1 },
		{ +1, 1, -2, 1 }
	);
	for (const Vec2& in : {
		Vec2{ 673, 343 },
		Vec2{ 515, 515 },
		Vec2{ 177, 473 },
		Vec2{ 433, 324 },
		})
	{
		Vec4 out = proj.FromViewportToWorldPlane( in.vec[0], in.vec[1] );
		std::cout << out.vec[0] << ' ' << out.vec[1] << ' ' << -out.vec[2] << std::endl;
	}
}

int main()
{
	std::cout << std::fixed << std::setprecision( 6 );
	Projector proj;
	// 1920x1080px, 60°h, 40°v
	proj.Setup( 1920, 1080, pi / 3, pi / 3.5f );
	// pitch -37°, yaw 10°, 3m height
	proj.Transform( -pi / 180.0f * 37.0f, pi / 18.0f, 3.0f );
	// some points on the plane
	proj.PlanePoints(
		{ 1, 0, -1, 1 },
		{ 0, 0, -2, 1 },
		{ 0, 0, -1, 1 }
	);
	for (const Vec2& in : {
		Vec2{ 1117, 1080 },
		Vec2{ 1161,  523 },
		Vec2{ 1015,  303 },
		Vec2{  991,  174 },
		Vec2{ 1054,   98 },
		Vec2{ 1167,   70 },
		Vec2{ 1189,   32 },
		Vec2{ 1160,    0 }
		})
	{
		Vec4 out = proj.FromViewportToWorldPlane( in.vec[0], in.vec[1] );
		std::cout << out.vec[0] << ' ' << out.vec[1] << ' ' << -out.vec[2] << std::endl;
	}
	return 0;
}
