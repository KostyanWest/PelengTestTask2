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
	void Setup( int screenWidth, int screenHeigth, float hAngle, float vAngle );

	void Transform( float pitch, float yaw, float height );

	Vec4 FromScreenToWorld( float x, float y );

private:
	float cameraToViewportMatrix[16] {};
	float viewportToWorldMatrix[16] {};
	float planeNormal[4] {};
	float planeDotProduct {};
};
