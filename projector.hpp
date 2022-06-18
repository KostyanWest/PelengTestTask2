#pragma once



struct Vec2
{
	float vec[2];
};

struct Vec4
{
	float vec[4];
};

class Projector
{
public:
	void Setup( int screenWidth, int screenHeigth, float hAngle, float vAngle ) noexcept;
	void Transform( float pitch, float yaw, float height ) noexcept;
	void PlanePoints( const Vec4& p1, const Vec4& p2, const Vec4& p3 ) noexcept;

	Vec4 FromWorldToViewport( const Vec4& vector ) const noexcept;
	Vec4 FromViewportToWorldPlane( float x, float y ) const noexcept;

private:
	float cameraToViewportMatrix[16] {};
	float worldToViewportMatrix[16] {};
	float viewportToWorldMatrix[16] {};
	float planeNormal[4] {};
	float planeDotProduct {};
};
