#pragma once

#include <iostream>
#include <cmath>

// approximated arc tan. func.
float fastAtan2(float y, float x) { // faster than std::atan2(3 ~ 5times), error get bigger around 0.005rad(≈0.3°)
    const float oneQuaterPi = 3.14159265358979323846/4.0f;
    const float threeQuaterPi = 3.0f*3.14159265358979323846/4.0f;
    float r, angle;
    float absY = std::fabs(y) + 1e-10f; // prevent division by zero
    if (x < 0.0f) {
        r = (x + absY) / (absY - x);
        angle = threeQuaterPi;
    }
    else {
        r = (x - absY) / (x + absY);
        angle = oneQuaterPi;
    }
    angle += (0.1963f * r * r - 0.9817f) * r;
    return (y < 0.0f) ? -angle : angle;
}