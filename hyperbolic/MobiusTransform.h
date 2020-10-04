#pragma once
#ifndef _MOBIUS_TRANSFORM_H_
#define _MOBIUS_TRANSFORM_H_

#include <complex>

template <typename T>
class MobiusTransform
{
public:
	MobiusTransform() : a(1, 0), b(), c(), d(1, 0) {}
	
	MobiusTransform(
		std::complex<T> a,
		std::complex<T> b,
		std::complex<T> c,
		std::complex<T> d)
		: a(a), b(b), c(c), d(d) {}

	inline std::complex<T> operator()(const std::complex<T>& z) const
	{
		return (a * z + b) / (c * z + d);
	}

	inline MobiusTransform& operator+= (const MobiusTransform& m)
	{
		*this = *this + m;
		return *this;
	}

	inline MobiusTransform& operator-= (const MobiusTransform& m)
	{
		*this = *this - m;
		return *this;
	}

	inline MobiusTransform operator- () const
	{
		return MobiusTransform(d, -b, -c, a);
	}

	/// (Mo_0 + Mo_1)(z) := Mo_1(Mo_0(z)) =: Mo_1 o Mo_0(z)
	/// (Mo_0 + ... + Mo_n)(z) := Mo_n o ... o Mo_0(z)
	template <typename U>
	friend MobiusTransform<U> operator+(const MobiusTransform<U>& lhs, const MobiusTransform<U>& rhs);

	template <typename U>
	friend MobiusTransform<U> operator-(const MobiusTransform<U>& lhs, const MobiusTransform<U>& rhs);

	template <typename U>
	friend std::ostream& operator<<(std::ostream& out, const MobiusTransform<U>& m);

protected:
	// w = \frac{az + b}{cz + d}
	std::complex<T> a, b, c, d;
};

template <typename T>
inline MobiusTransform<T> operator+(const MobiusTransform<T>& lhs, const MobiusTransform<T>& rhs)
{
	std::complex<T> a = lhs.a * rhs.a + lhs.c * rhs.b; // a1*a2 + c1*b2
	std::complex<T> b = lhs.b * rhs.a + lhs.d * rhs.b; // b1*a2 + d1*b2
	std::complex<T> c = lhs.a * rhs.c + lhs.c * rhs.d; // a1*c2 + c1*d2
	std::complex<T> d = lhs.b * rhs.c + lhs.d * rhs.d; // b1*c2 + d1*d2
	return MobiusTransform<T>(a, b, c, d);
}

template<typename T>
inline MobiusTransform<T> operator-(const MobiusTransform<T>& lhs, const MobiusTransform<T>& rhs)
{
	return lhs + (-rhs);
}

template<typename T>
inline std::ostream& operator<<(std::ostream& out, const MobiusTransform<T>& m)
{
	out << "(" << m.a << " Z + " << m.b << ") / (" << m.c << " Z + " << m.d << ")";
	return out;
}

#endif // !_MOBIUS_TRANSFORM_H_
