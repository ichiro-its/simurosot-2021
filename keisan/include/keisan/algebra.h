#ifndef _ALGEBRA_H
#define _ALGEBRA_H

#include "vector2.h"

namespace keisan
{
	float piValue();
	float rad2Deg();
	float deg2Rad();

	float normalizeDeg(float val);
	float normalizeRad(float val);

	float deltaAngle(float a, float b);

	float sqr(float val);
	float sqrt(float val);

	float random();
	float random(float val);
	float randomRange(float a, float b);

	int random(int val);
	int randomRange(int a, int b);

	float infinityFloat();

	float sign(float val);
	float abs(float val);

	float minValue(float a, float b);
	float maxValue(float a, float b);

	float lerp(float a, float b, float rate);

	bool valueInside(float value, float min, float max);
	bool valueOutside(float value, float min, float max);

	float clampValue(float val, float min, float max);
	float smoothValue(float a, float b, float percent);
	float curveValue(float val, float min, float max, float exponential);

	float mapValue(float source_val, float source_min, float source_max, float target_min, float target_max);

	float distance(float x, float y);
	float direction(float x, float y);
	float direction(Vector2 a, Vector2 b);
}

#endif
