#include <iostream>
#include <iomanip>
#include <cmath>



static constexpr float pi = 3.14159265358979323846f;


struct Vec2
{
	float x, y;
};


struct Vec3
{
	Vec3 operator+( float s ) const noexcept
	{
		return Vec3{ x + s, y + s, z + s };
	}

	Vec3 operator-( float s ) const noexcept
	{
		return Vec3{ x - s, y - s, z - s };
	}

	Vec3 operator*( float s ) const noexcept
	{
		return Vec3{ x * s, y * s, z * s };
	}

	Vec3 operator/( float s ) const noexcept
	{
		return Vec3{ x / s, y / s, z / s };
	}


	Vec3 operator+( const Vec3& v ) const noexcept
	{
		return Vec3{ x + v.x, y + v.y, z + v.z };
	}

	Vec3 operator-( const Vec3& v ) const noexcept
	{
		return Vec3{ x - v.x, y - v.y, z - v.z };
	}

	float operator*( const Vec3& v ) const noexcept
	{
		return x * v.x + y * v.y + z * v.z;
	}


	float x, y, z;
};


struct Mat3
{
	constexpr Mat3() noexcept
		: ox{ 1.0f, 0.0f, 0.0f }, oy{ 0.0f, 1.0f, 0.0f }, oz{ 0.0f, 0.0f, 1.0f }
	{
	}

	constexpr Mat3( Vec3 x, Vec3 y, Vec3 z ) noexcept
		: ox{ x }, oy{ y }, oz{ z }
	{
	}

	// x axis rotate matrix (counter-clockwise)
	static Mat3 RotateX( float angle ) noexcept
	{
		float cos = std::cos( angle );
		float sin = std::sin( angle );

		// векторы-столбцы
		return Mat3(
			Vec3{ 1.0f, 0.0f, 0.0f },
			Vec3{ 0.0f,  cos,  sin },
			Vec3{ 0.0f, -sin,  cos }
		);
	}

	// y axis rotate matrix (counter-clockwise)
	static Mat3 RotateY( float angle ) noexcept
	{
		float cos = std::cos( angle );
		float sin = std::sin( angle );

		// векторы-столбцы
		return Mat3(
			Vec3{  cos, 0.0f, -sin },
			Vec3{ 0.0f, 1.0f, 0.0f },
			Vec3{  sin, 0.0f,  cos }
		);
	}

	// z axis rotate matrix (counter-clockwise)
	static Mat3 RotateZ( float angle ) noexcept
	{
		float cos = std::cos( angle );
		float sin = std::sin( angle );

		// векторы-столбцы
		return Mat3(
			Vec3{  cos,  sin, 0.0f },
			Vec3{ -sin,  cos, 0.0f },
			Vec3{ 0.0f, 0.0f, 1.0f }
		);
	}

	Vec3 operator*( const Vec3& v ) const noexcept
	{
		return Vec3{ RowX() * v, RowY() * v, RowZ() * v };
	}

	Mat3 operator*( const Mat3& m ) const noexcept
	{
		// векторы-столбцы
		return Mat3(
			Vec3{ RowX() * m.ox, RowY() * m.ox, RowZ() * m.ox },
			Vec3{ RowX() * m.oy, RowY() * m.oy, RowZ() * m.oy },
			Vec3{ RowX() * m.oz, RowY() * m.oz, RowZ() * m.oz }
		);
	}

	Vec3 RowX() const noexcept
	{
		return Vec3{ ox.x, oy.x, oz.x };
	}

	Vec3 RowY() const noexcept
	{
		return Vec3{ ox.y, oy.y, oz.y };
	}

	Vec3 RowZ() const noexcept
	{
		return Vec3{ ox.z, oy.z, oz.z };
	}

	// каждый вектор - это столбец, а не строка (!) матрицы 3х3
	Vec3 ox, oy, oz;
};



/*
@brief Позволяет генерировать лучи от "камеры" в направлении координат экрана.
"Камера" всегда располагается в начале координат.
По умолчанию "камера" смотрит вдоль оси Oy, X координата экрана направлена вдоль оси Ox,
а Y координата - против оси Oz.
*/
class RayGenerator
{
public:
	/*
	@param screenWidth Ширина экрана в пикселях.
	@param screenHeigth Высота экрана в пикселях.
	@param hAngle Угол обзора камеры по горизонтали в радианах.
	@param vAngle Угол обзора камеры по вертикали в радианах.
	*/
	explicit RayGenerator(
		int screenWidth,
		int screenHeigth,
		float hAngle,
		float vAngle
	) noexcept
		: hBias{ -screenWidth / 2.0f }
		, hMultiplayer{ std::tan( hAngle / 2 ) * 2.0f / screenWidth }
		, vBias{ -screenHeigth / 2.0f }
		, vMultiplayer{ -std::tan( vAngle / 2 ) * 2.0f / screenHeigth }
		, rotor{}
	{
	}

	/*
	@brief адаёт поворот "камеры" с помощью углов Эйлера.
	@param pitch Угол поворота вдоль оси Ox в радианах (крен).
	@param yaw Угол поворота вокруг оси Oz в радианах (тангаж).
	*/
	void Rotate( float pitch, float yaw ) noexcept
	{
		rotor = Mat3::RotateZ( yaw ) * Mat3::RotateX( pitch );
	}

	/*
	@brief Генерирует луч от начала координат в направлении координат экрана.
	@param windowCoords Координаты экрана в пикселях, в направлении которых указывает луч.
	@return Луч от начала координат. Если не применено вращение с помощью метода Rotate(),
	координата Y луча будет всегда равна 1.
	*/
	Vec3 Generate( const Vec2& windowCoords ) const noexcept
	{
		return rotor * Vec3{ (windowCoords.x + hBias) * hMultiplayer, 1.0f, (windowCoords.y + vBias) * vMultiplayer };
	}

private:
	float hBias;
	float hMultiplayer;
	float vBias;
	float vMultiplayer;
	Mat3 rotor;
};



/*
@brief Обнаоуживает пересечение луча с плоскостью Oxy.
@param start Начало луча.
@param direction Направление луча.
@return Точка пересечения луча с плоскостью Oxy.
*/
Vec3 CastRay(const Vec3& start, const Vec3& direction) noexcept
{
	// немного математики:
	// положим плоскость проходит через точку [0, 0, 0],
	// тогда условие для пересечения луча запишем через скалярное произведение 
	//     (start + d * direction) * normal == 0
	//     d == -(start * normal) / (direction * normal)
	// для плоскости Oxy нормаль normal = [0, 0, 1]
	//     d == -start.z / direction.z

	float d = -start.z / direction.z;
	return  direction * d + start;
}



int main()
{
	std::cout << std::fixed << std::setprecision( 6 );

	RayGenerator generator(
		1920,                 // width 1920px
		1080,                 // height 1080px
		pi / 3,               // 60° h fov
		pi / 4.5f );          // 40° v fov

	generator.Rotate(
		-pi / 180.0f * 37.0f, // pitch -37°
		pi / 18.0f );         // yaw 10°

	Vec3 cameraPos{ 0.0f, 0.0f, 3.0f };

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
		Vec3 hit = CastRay( cameraPos, generator.Generate( in ) );
		std::cout << hit.x << '\t' << hit.y << '\t' << hit.z << std::endl;
	}

	return 0;
}
