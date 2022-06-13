#include "projector.hpp"

#include <iostream>
#include <conio.h>

static constexpr float pi = 3.14159265358979323846f;

int main()
{
    Projector proj;
    // 1920x1080px, 60°h, 40°v
    proj.Setup( 1920, 1080, 3.14f / 3, 3.14f / 3.5f );
    // pitch -37°, yaw -10°, 3m height
    proj.Transform( -pi / 180.0f * 37.0f, -pi / 18.0f, 3.0f );
    for (const Vec2& in : {
        Vec2{1117, 1080},
        Vec2{1161, 523},
        Vec2{1015, 303},
        Vec2{991, 174},
        Vec2{1161, 523},
        Vec2{1054, 98},
        Vec2{1167, 70},
        Vec2{1189, 32},
        Vec2{1160, 0}
        })
    {
        Vec4 out = proj.FromScreenToWorld( in.vec[0], in.vec[1] );
        std::cout << out.vec[0] << ' ' << -out.vec[2] << std::endl;
    }
    _getch();
    return 0;
}

