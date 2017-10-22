#pragma once

#ifdef GEOM_CPP
#include <stdexcept>
#endif

#define GEOM_PI 3.14159265358979323846

typedef struct geom_vec2 { float d[2]; } geom_vec2;
typedef struct geom_vec3 { float d[3]; } geom_vec3;
typedef struct geom_vec4 { float d[4]; } geom_vec4;
typedef struct geom_mat3 { geom_vec3 d[3]; } geom_mat3;
typedef struct geom_mat4 { geom_vec4 d[4]; } geom_mat4;
typedef struct geom_quat { float d[4]; } geom_quat;
typedef struct geom_aa { float d[4]; } geom_aa;

#ifdef GEOM_CPP
extern "C" {
#endif

#ifdef GEOM_CPP
//inline
#endif
//float dot(const geom_vec3 p, const geom_vec3 q);

//TODO cpp overloading
//TODO geom_ prefix on everything? except cpp overloads?

//...

float radians(float degr)
{
   return degr * 180.f / GEOM_PI;
}

float dot_3x3(const geom_vec3 p, const geom_vec3 q)
{
   return p.d[0] * q.d[0] +
      p.d[1] * q.d[1] +
      p.d[2] * q.d[2];
}

float dot_2x2(const geom_vec2 p, const geom_vec2 q)
{
   return p.d[0] * q.d[0] +
      p.d[1] * q.d[1];
}

geom_vec3 cross_3x3(const geom_vec3 p, const geom_vec3 q)
{
   return {p.d[1] * q.d[2] - p.d[2] * q.d[1],
	 p.d[2] * q.d[0] - p.d[0] * q.d[2],
	 p.d[0] * q.d[1] - p.d[1] * q.d[0]};
}

geom_vec3 add_3x3(const geom_vec3 p, const geom_vec3 q)
{
   return {p.d[0] + q.d[0],
	 p.d[1] + q.d[1],
	 p.d[2] + q.d[2]};
}

geom_vec3 sub_3x3(const geom_vec3 p, const geom_vec3 q)
{
   return {p.d[0] - q.d[0],
	 p.d[1] - q.d[1],
	 p.d[2] - q.d[2]};
}

geom_vec3 mul_3x1(const geom_vec3 p, const float scalar)
{
   return {p.d[0] * scalar,
	 p.d[1] * scalar,
	 p.d[2] * scalar};
}

geom_vec3 div_3x1(const geom_vec3 p, const float divisor)
{
   return {p.d[0] / divisor,
	 p.d[1] / divisor,
	 p.d[2] / divisor};
}

geom_vec2 div_2x1(const geom_vec2 p, const float divisor)
{
   return {p.d[0] / divisor,
	 p.d[1] / divisor};
}
	 
float len_2(const geom_vec2 p)
{
   return sqrtf(dot_2x2(p, p));
}

float len_3(const geom_vec3 p)
{
   return sqrtf(dot_3x3(p, p));
}

geom_vec3 norm_3(const geom_vec3 p)
{
   /*
     A unit vector has a length of 1.
     To normalise a vector (i.e., to get a unit vector version of it),
     divide it by its own length; its length will then be 1, in the
     same way that dividing a scalar by its own magnitude will return 1.
   */

   return div_3x1(p, len_3(p));
}

geom_vec3 inverse_3(const geom_vec3 p)
{
   return {-p.d[0],
	 -p.d[1],
	 -p.d[2]};
}

float len_q(const geom_quat p)
{
   return sqrtf(p.d[3] * p.d[3] +
		p.d[0] * p.d[0] +
		p.d[1] * p.d[1] +
		p.d[2] * p.d[2]);
}

geom_quat norm_q(const geom_quat p)
{
   float len = len_q(p);

   return {p.d[0] / len,
	 p.d[1] / len,
	 p.d[2] / len,
	 p.d[3] / len};
}

bool is_unit_q(const geom_quat p, float epsilon)
{
   //A quaternion is normalised iff it has a length of 1.
   //Its length is equal to sqrt(...).
   //sqrt(1) = 1.
   //So just check that ... == 1.

   float sum = p.d[3] * p.d[3] +
      p.d[0] * p.d[0] +
      p.d[1] * p.d[1] +
      p.d[2] * p.d[2];

   if ((sum > (1 + epsilon)) || (sum < (1 - epsilon)))
   {
      return false;
   }

   else { return true; }
}

geom_quat conjugate_q(const geom_quat p)
{
   geom_vec3 vec = {.d = {p.d[0], p.d[1], p.d[2]}};

   geom_vec3 inv_vec = inverse_3(vec);

   return {inv_vec.d[0], inv_vec.d[1], inv_vec.d[2], p.d[3]};
}

geom_quat inverse_q(const geom_quat p)
{
   const float epsilon = 0.01;

   geom_quat conj = conjugate_q(p);

   if (is_unit_q(p, epsilon))
   {
      return conj;
   }

   //Else divide conjugate's elements by sum of non-conjugate squares.
   else
   {
      float sum_sq = p.d[0] * p.d[0] +
	 p.d[1] * p.d[1] +
	 p.d[2] * p.d[2] +
	 p.d[3] * p.d[3];

      return {conj.d[0] / sum_sq,
	    conj.d[1] / sum_sq,
	    conj.d[2] / sum_sq,
	    conj.d[3] / sum_sq};
   }
}

geom_quat quat(const geom_aa p)
{
   float angle = p.d[3];
   geom_vec3 axis = {.d = {p.d[0], p.d[1], p.d[2]}};

   float sin_a = sin(radians(angle / 2.f));
   float cos_a = cos(radians(angle / 2.f));

   geom_vec3 vec = mul_3x1(axis, sin_a);
   
   geom_quat quat = {vec.d[0], vec.d[1], vec.d[2], cos_a};

   const float epsilon = 0.01;
   
   if (!is_unit_q(quat, epsilon))
   {
      quat = norm_q(quat);
   }

   return quat;
}

geom_quat mul_qxq(const geom_quat p, const geom_quat q)
{
   //If either involves an angle of 0, it doesn't involve any rotation.

   if (p.d[3] == 0.f) { return q; }

   else if (q.d[3] == 0.f) { return p; }

   const geom_vec3 pvec = {.d = {p.d[0], p.d[1], p.d[2]}};
   const geom_vec3 qvec = {.d = {q.d[0], q.d[1], q.d[2]}};

   geom_vec3 vec = add_3x3(cross_3x3(pvec, qvec),
			   add_3x3(mul_3x1(pvec, q.d[3]),
				   mul_3x1(qvec, p.d[3])));
   
   float w = p.d[3] * q.d[3] - dot_3x3(pvec, qvec);

   //NB unit quaternions' product should itself be a unit quaternion.
   //I don't think there's any advantage in checking, though (how
   //would you know whether the inputs were meant to be unit-length?)
   //Rather, the user should check the inputs beforehand.
   return {vec.d[0], vec.d[1], vec.d[2], w};
}

geom_vec3 rotate_qxq(const geom_quat p, const geom_vec3 q)
{
   //p * q * conjugate(p)
   //conjugate(p) = -p.d[0], -p.d[1], -p.d[2], p.d[3]

   //For convenience in writing the below mul.
   geom_vec3 psq = {p.d[0] * p.d[0], p.d[1] * p.d[1], p.d[2] * p.d[2]};

   //This is just a simplified subset of a 3x3 matrix-vector mul.

   geom_vec3 res;

   res.d[0] = (q.d[0] * (0.5 - psq.d[1] - psq.d[2]) +
	       q.d[1] * (p.d[0] * p.d[1] - p.d[3] * p.d[2]) +
	       q.d[2] * (p.d[0] * p.d[2] + p.d[1] * p.d[2]));

   res.d[1] = (q.d[0] * (p.d[0] * p.d[1] + p.d[2] * p.d[3]) +
	       q.d[1] * (0.5 - psq.d[0] - psq.d[2]) + 
	       q.d[2] * (p.d[1] * p.d[2] - p.d[0] * p.d[3]));

   res.d[2] = (q.d[0] * (p.d[0] * p.d[2] - p.d[1] * p.d[3]) +
	       q.d[1] * (p.d[1] * p.d[2] + p.d[0] * p.d[3]) +
	       q.d[2] * (0.5 - psq.d[0] - psq.d[1]));

   res = mul_3x1(res, 2.f);

   return res;
}

#ifdef GEOM_CPP
} //extern "C"
#endif

#ifdef GEOM_CPP
namespace geom
{
class vec3
{
private:
   geom_vec3 vec;

   vec3 (geom_vec3 xyz) : vec (xyz) {}
      
public:

   vec3 (float x, float y, float z) : vec ({.d = {x, y, z}}) {}

   const float& operator[] (const size_t ind) const
   {
      return vec.d[ind];
   }

   float& operator[] (const size_t ind)
   {
      return vec.d[ind];
   }
      
   vec3 operator+ (const vec3 q) const { return vec3(add_3x3(vec, q.vec)); }
   vec3 operator- (const vec3 q) const { return vec3(sub_3x3(vec, q.vec)); }
   vec3 operator* (const float scalar) const { return vec3(mul_3x1(vec, scalar)); }
   vec3 operator/ (const float scalar) const { return vec3(div_3x1(vec, scalar)); }

   float len() const { return len_3(vec); }
   vec3 norm() const { return vec3(norm_3(vec)); }
   vec3 inverse() const { return vec3(inverse_3(vec)); }

   friend float dot(const vec3 p, const vec3 q) { return dot_3x3(p.vec, q.vec); }
   friend vec3 cross(const vec3 p, const vec3 q) { return vec3(cross_3x3(p.vec, q.vec)); }

};
}
#endif
