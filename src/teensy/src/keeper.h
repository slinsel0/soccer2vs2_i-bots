#pragma once

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1

struct Vec2 {
    float x;
    float y;

    Vec2 operator-(const Vec2 &other) const {
        return { x - other.x, y - other.y };
    }

    Vec2 operator+(const Vec2 &other) const {
        return { x + other.x, y + other.y };
    }

    Vec2 operator/(float s) const {
        return { x / s, y / s };
    }
};

#endif

Vec2 keeper(Vec2 Player, Vec2 ball);   // ← nur Deklaration