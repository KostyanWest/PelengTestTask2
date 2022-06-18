#include "projector.hpp"

#include "Eigen/Dense"
#include <cmath>



void Projector::Setup( int screenWidth, int screenHeigth, float hAngle, float vAngle ) noexcept
{
	float near = 0.1f;
	float far = 100.0f;
	float right = near * std::tan( hAngle / 2 );
	float top = near * std::tan( vAngle / 2 );

	// form camera coordinates to normalized device coordinates
	Eigen::Matrix4f projectionMatrix;
	projectionMatrix <<
		near / right, 0.0f, 0.0f, 0.0f,
		0.0f, near / top, 0.0f, 0.0f,
		0.0f, 0.0f, (near + far) / (near - far), 2 * far * near / (near - far),
		0.0f, 0.0f, -1.0f, 0.0f;

	// from normalized device coordinates to viewport coordinates
	Eigen::Matrix4f transformMatrix;
	transformMatrix <<
		screenWidth / 2.0f, 0.0f, 0.0f, screenWidth / 2.0f,
		0.0f, screenHeigth / -2.0f, 0.0f, screenHeigth / 2.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	reinterpret_cast<Eigen::Matrix4f*>(cameraToViewportMatrix)->noalias() = transformMatrix * projectionMatrix;
}

void Projector::Transform( float pitch, float yaw, float height ) noexcept
{
	float yawCos = std::cos( -yaw );
	float yawSin = std::sin( -yaw );
	// y axis rotate (counter-clockwise)
	Eigen::Matrix4f yawMatrix;
	yawMatrix <<
		yawCos, 0.0f, yawSin, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		-yawSin, 0.0f, yawCos, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	float pitchCos = std::cos( -pitch );
	float pitchSin = std::sin( -pitch );
	// x axis rotate (counter-clockwise)
	Eigen::Matrix4f pitchMatrix;
	pitchMatrix <<
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, pitchCos, -pitchSin, 0.0f,
		0.0f, pitchSin, pitchCos, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	// set height
	Eigen::Matrix4f translateMatrix;
	translateMatrix <<
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, -height,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	const Eigen::Matrix4f worldToCameraMatrix = pitchMatrix * yawMatrix * translateMatrix;

	reinterpret_cast<Eigen::Matrix4f*>(this->worldToViewportMatrix)->noalias() =
		*reinterpret_cast<Eigen::Matrix4f*>(cameraToViewportMatrix) * worldToCameraMatrix;

	reinterpret_cast<Eigen::Matrix4f*>(viewportToWorldMatrix)->noalias() =
		reinterpret_cast<Eigen::Matrix4f*>(this->worldToViewportMatrix)->inverse();
}

void Projector::PlanePoints( const Vec4& p1, const Vec4& p2, const Vec4& p3 ) noexcept
{
	const Eigen::Matrix4f& worldToViewportMatrix =
		*reinterpret_cast<Eigen::Matrix4f*>(this->worldToViewportMatrix);

	// some points on the plane
	Eigen::Vector4f planeP1 = worldToViewportMatrix * (*reinterpret_cast<const Eigen::Vector4f*>(p1.vec));
	planeP1 /= planeP1( 3 );
	Eigen::Vector4f planeP2 = worldToViewportMatrix * (*reinterpret_cast<const Eigen::Vector4f*>(p2.vec));
	planeP2 /= planeP2( 3 );
	Eigen::Vector4f planeP3 = worldToViewportMatrix * (*reinterpret_cast<const Eigen::Vector4f*>(p3.vec));
	planeP3 /= planeP3( 3 );

	// the plane is defined by the normal and the dot product of the normal and any point on the plane
	reinterpret_cast<Eigen::Vector4f*>(planeNormal)->noalias() = (planeP1 - planeP3).cross3( planeP2 - planeP3 );
	planeDotProduct = reinterpret_cast<Eigen::Vector4f*>(planeNormal)->dot( planeP3 );
}

Vec4 Projector::FromWorldToViewport( const Vec4& vector ) const noexcept
{
	Vec4 result {};
	const Eigen::Vector4f& worldVector = *reinterpret_cast<const Eigen::Vector4f*>(vector.vec);
	Eigen::Vector4f& viewportVector = *reinterpret_cast<Eigen::Vector4f*>(result.vec);
	viewportVector = *reinterpret_cast<const Eigen::Matrix4f*>(worldToViewportMatrix) * worldVector;
	viewportVector /= viewportVector( 3 );
	return result;
}

Vec4 Projector::FromViewportToWorldPlane( float x, float y ) const noexcept
{
	Vec4 result {};
	Eigen::Vector4f& worldVector = *reinterpret_cast<Eigen::Vector4f*>(result.vec);
	Eigen::Vector4f viewportVector;
	viewportVector << x, y, (planeDotProduct - planeNormal[0] * x - planeNormal[1] * y) / planeNormal[2], 1.0f;
	worldVector.noalias() = *reinterpret_cast<const Eigen::Matrix4f*>(viewportToWorldMatrix) * viewportVector;
	worldVector /= worldVector( 3 );
	return result;
}
