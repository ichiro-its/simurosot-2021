#include <math.h>
#include <limits>
#include <cstdlib>

#include "keisan/algebra.h"

float keisan::piValue()
{
	return 3.14159265359;
}

float keisan::rad2Deg()
{
	return 180.0 / keisan::piValue();
}

float keisan::deg2Rad()
{
	return keisan::piValue() / 180.0;
}

float keisan::normalizeDeg(float val)
{
	while (val < -180.0)
		val += 360.0;

	while (val > 180.0)
		val -= 360.0;

	return val;
}

float keisan::normalizeRad(float val)
{
	while (val < -keisan::piValue())
		val += 2 * keisan::piValue();

	while (val > keisan::piValue())
		val -= 2 * keisan::piValue();

	return val;
}

float keisan::deltaAngle(float a, float b)
{
	a = normalizeDeg(a);
	b = normalizeDeg(b);

	float aa = a - b;
	float bb = aa + ((aa < 0.0) ? 360.0 : -360.0);
	return (fabs(aa) < fabs(bb)) ? aa : bb;
}

float keisan::sqr(float val)
{
	return val * val;
}

float keisan::sqrt(float val)
{
	return pow(val, 0.5);
}

float keisan::random()
{
	return keisan::clampValue((float)std::rand() / (float)RAND_MAX, 0.0, 1.0);
}

float keisan::random(float val)
{
	return keisan::random() * val;
}

float keisan::randomRange(float a, float b)
{
	return a + random(b - a);
}

int keisan::random(int val)
{
	if (val == 0)
		return 0;

	return std::rand() % val;
}

int keisan::randomRange(int a, int b)
{
	return a + random(b - a);
}

float keisan::infinityFloat()
{
	return std::numeric_limits<float>::infinity();
}

float keisan::sign(float val)
{
	if (val == 0)
		return 1.0;

	return val / keisan::abs(val);
}

float keisan::abs(float val)
{
	return (val < 0.0) ? -val : val;
}

float keisan::minValue(float a, float b)
{
	return (a <= b) ? a : b;
}

float keisan::maxValue(float a, float b)
{
	return (a >= b) ? a : b;
}

float keisan::lerp(float a, float b, float rate)
{
	return (a + ((b - a) * rate));
}

bool keisan::valueInside(float value, float min, float max)
{
	return (value > min && value < max);
}

bool keisan::valueOutside(float value, float min, float max)
{
	return (value < min || value > max);
}

float keisan::clampValue(float val, float min, float max)
{
	return keisan::maxValue(keisan::minValue(val, max), min);
}

float keisan::smoothValue(float a, float b, float percent)
{
	return ((1.0 - percent) * a) + (percent * b);
}

float keisan::curveValue(float val, float min, float max, float exponential)
{
	val = keisan::minValue(val, keisan::maxValue(min, max));
	val = keisan::maxValue(val, keisan::minValue(min, max));
	return min + ((max - min) * (pow(val - min, exponential) / pow(max - min, exponential)));
}

float keisan::mapValue(float source_val, float source_min, float source_max, float target_min, float target_max)
{
	source_val = keisan::minValue(source_val, keisan::maxValue(source_min, source_max));
	source_val = keisan::maxValue(source_val, keisan::minValue(source_min, source_max));
	return target_min + ((source_val - source_min) * ((target_max - target_min) / (source_max - source_min)));
}

float keisan::distance(float x, float y)
{
	return keisan::sqrt(keisan::sqr(x) + keisan::sqr(y));
}

float keisan::direction(float x, float y)
{
	return keisan::normalizeRad(atan2(y, x));
}

float keisan::direction(Vector2 a, Vector2 b)
{
	return keisan::direction(b.getX() - a.getX(), b.getY() - a.getY());
}