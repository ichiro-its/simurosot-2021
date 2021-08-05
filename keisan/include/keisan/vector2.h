#ifndef _VECTOR2_H
#define _VECTOR2_H

namespace keisan
{
	class Vector2
	{
	private:
		float x_value_;
		float y_value_;

	public:
		Vector2();
		Vector2(float x, float y);

		~Vector2() {}

		float getX() { return x_value_; }
		float getY() { return y_value_; }

		void setX(float value) { x_value_ = value; }
		void setY(float value) { y_value_ = value; }

		float length();
		void normalize();

		Vector2 &operator=(Vector2 &vector);
		Vector2 &operator+=(Vector2 &vector);
		Vector2 &operator-=(Vector2 &vector);
		Vector2 &operator+=(float value);
		Vector2 &operator-=(float value);
		Vector2 &operator*=(float value);
		Vector2 &operator/=(float value);

		Vector2 &operator+(Vector2 &vector);
		Vector2 &operator-(Vector2 &vector);
		Vector2 &operator+(float value);
		Vector2 &operator-(float value);
		Vector2 &operator*(float value);
		Vector2 &operator/(float value);
	};
}

#endif
