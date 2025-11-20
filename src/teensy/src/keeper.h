#pragma once

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
    float x;
    float y;


    Vec2 operator-(const Vec2 &other) const {
        return { x - other.x, y - other.y };
    }
};
#endif

void keeper (Vec2 player,Vec2 ball);




