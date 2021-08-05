#include "keisan/vector2.h"
#include "keisan/algebra.h"

using namespace keisan;

Vector2::Vector2()
{
	x_value_ = 0;
	y_value_ = 0;
}

Vector2::Vector2(float x, float y)
{
	x_value_ = x;
	y_value_ = y;
}

float Vector2::length()
{
	return keisan::sqrt(keisan::sqr(x_value_) + keisan::sqrt(y_value_));
}

void Vector2::normalize()
{
	x_value_ = x_value_ / length();
	y_value_ = y_value_ / length();
}

Vector2 &Vector2::operator=(Vector2 &vector)
{
	x_value_ = vector.getX();
	y_value_ = vector.getY();

	return *this;
}

Vector2 &Vector2::operator+=(Vector2 &vector)
{
	x_value_ += vector.getX();
	y_value_ += vector.getY();

	return *this;
}

Vector2 &Vector2::operator-=(Vector2 &vector)
{
	x_value_ -= vector.getX();
	y_value_ -= vector.getY();

	return *this;
}

Vector2 &Vector2::operator+=(float value)
{
	x_value_ += value;
	y_value_ += value;

	return *this;
}

Vector2 &Vector2::operator-=(float value)
{
	x_value_ -= value;
	y_value_ -= value;

	return *this;
}

Vector2 &Vector2::operator*=(float value)
{
	x_value_ *= value;
	y_value_ *= value;

	return *this;
}

Vector2 &Vector2::operator/=(float value)
{
	x_value_ /= value;
	y_value_ /= value;

	return *this;
}

Vector2 &Vector2::operator+(Vector2 &vector)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ + vector.getX());
	result->setY(y_value_ + vector.getY());

	return *result;
}

Vector2 &Vector2::operator-(Vector2 &vector)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ - vector.getX());
	result->setY(y_value_ - vector.getY());

	return *result;
}

Vector2 &Vector2::operator+(float value)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ + value);
	result->setY(y_value_ + value);

	return *result;
}

Vector2 &Vector2::operator-(float value)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ - value);
	result->setY(y_value_ - value);

	return *result;
}

Vector2 &Vector2::operator*(float value)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ * value);
	result->setY(y_value_ * value);

	return *result;
}

Vector2 &Vector2::operator/(float value)
{
	Vector2 *result = new Vector2();
	result->setX(x_value_ / value);
	result->setY(y_value_ / value);

	return *result;
}
