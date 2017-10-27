#pragma once

//include cmath?

#ifdef GEOM_CPP
#include <stdexcept>
#include <cmath> //for fmod
//TODO remove
#include <iostream>
   using namespace std;
#endif

#define GEOM_PI 3.14159265358979323846

typedef struct geom_vec2 { float d[2]; } geom_vec2;
typedef struct geom_vec3 { float d[3]; } geom_vec3;
typedef struct geom_vec4 { float d[4]; } geom_vec4;
typedef struct geom_mat3 { float d[9]; } geom_mat3;
typedef struct geom_mat4 { float d[16]; } geom_mat4;
typedef struct geom_quat { float d[4]; } geom_quat;
typedef struct geom_aa { float d[4]; } geom_aa;

#ifdef GEOM_CPP
//inline
#endif
//float dot(const geom_vec3 p, const geom_vec3 q);

#ifdef GEOM_CPP
extern "C" {
#endif

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

int is_unit_2(const geom_vec2 p, float epsilon)
{
   float len = len_2(p);

   return (!((len > (1.f + epsilon)) || (len < (1.f - epsilon))));
}

int is_unit_3(const geom_vec3 p, float epsilon)
{
   float len = len_3(p);

   return (!((len > (1.f + epsilon)) || (len < (1.f - epsilon))));
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

int is_unit_q(const geom_quat p, float epsilon)
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
      return 0;
   }

   else { return 1; }
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

geom_quat make_q(const geom_aa p)
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
#if 0
   //'Hamiltonian' multiplication
   return {.d =
	 {p.d[3] * q.d[0] + p.d[0] * q.d[3] + p.d[1] * q.d[2] - p.d[2] * q.d[1],
	  p.d[3] * q.d[1] - p.d[0] * q.d[2] + p.d[1] * q.d[3] + p.d[2] * q.d[0],
	  p.d[3] * q.d[2] + p.d[0] * q.d[1] - p.d[1] * q.d[0] + p.d[2] * q.d[3],
	  //w
	  p.d[3] * q.d[3] - p.d[0] * q.d[0] - p.d[1] * q.d[1] - p.d[2] * q.d[2]}};
#endif

   /*
     Note w = 4-place dot; xyz = p.d[3] * q.xyz (the left-hand column
     of mults) + q.d[3] * p.xyz (the q.d[3] mults which shift place) +
     cross(p.xyz, q.xyz) (the pairs of subs)...!
   
     So things are simplified by checking if either [3] is 0.
   */

   geom_vec3 pxyz = {p.d[0], p.d[1], p.d[2]};
   geom_vec3 qxyz = {q.d[0], q.d[1], q.d[2]};

   geom_vec3 xyz = cross_3x3(pxyz, qxyz);

   //?: can't initialise
   geom_vec3 zero = {.d = {0.f, 0.f, 0.f}};

   geom_vec3 pxyzw = p.d[3] ? mul_3x1(qxyz, p.d[3]) : zero;
   geom_vec3 qxyzw = q.d[3] ? mul_3x1(pxyz, q.d[3]) : zero;

   xyz = add_3x3(xyz, add_3x3(pxyzw, qxyzw));

   float w = p.d[3] * q.d[3] - dot_3x3(pxyz, qxyz);

   return {.d = {xyz.d[0], xyz.d[1], xyz.d[2], w}};
}

#if 0
geom_vec3 rotate_qx3(const geom_quat p, const geom_vec3 q)
{
   //p * q * conjugate(p)
   //conjugate(p) = -p.d[0], -p.d[1], -p.d[2], p.d[3]

   //For convenience in writing the below mul.
   geom_vec3 psq = {p.d[0] * p.d[0], p.d[1] * p.d[1], p.d[2] * p.d[2]};

   geom_vec3 res;

   //I believe this is based on matrix multiplication of the matrix
   //gotten from the quaternion. Possibly meant to be an optimisation
   //based on number of ops; yet to check.

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
#endif

geom_vec3 rotate_qx3(const geom_quat p, const geom_vec3 q)
{
   //= quat * vec * quat.conjugate

   geom_quat conj = conjugate_q(p);

   //Simplifying first (right hand) mult because vec.w = 0

   geom_quat first = {.d =
		      {q.d[0] * conj.d[3] + q.d[1] * conj.d[2] - q.d[2] * conj.d[1],
		       -q.d[0] * conj.d[2] + q.d[1] * conj.d[3] + q.d[2] * conj.d[0],
		       q.d[0] * conj.d[1] - q.d[1] * conj.d[0] + q.d[2] * conj.d[3],
		       -q.d[0] * conj.d[0] - q.d[1] * conj.d[1] + q.d[2] * conj.d[2]}};
   
   //Note: first[.xyz] = cross(q, conj) + q * conj[3] (the diagonal).
   
   geom_quat result = mul_qxq(p, first);

   return {.d = {result.d[0], result.d[1], result.d[2]}};
}

geom_mat3 mul_9x9(const geom_mat3 p, const geom_mat3 q)
{
   //This is clearer if you don't double it up as I have
   return {.d = {p.d[0] * q.d[0] + p.d[3] * q.d[1] +
		 p.d[6] * q.d[2], p.d[1] * q.d[0] +
		 p.d[4] * q.d[1] + p.d[7] * q.d[2],
		 p.d[2] * q.d[0] + p.d[5] * q.d[1] +
		 p.d[8] * q.d[2], p.d[0] * q.d[3] + //
		 p.d[3] * q.d[4] + p.d[6] * q.d[5],
		 p.d[1] * q.d[3] + p.d[4] * q.d[4] +
		 p.d[7] * q.d[5], p.d[2] * q.d[3] +
		 p.d[5] * q.d[4] + p.d[8] * q.d[5],
		 p.d[0] * q.d[6] + p.d[3] * q.d[7] + //
		 p.d[6] * q.d[8], p.d[1] * q.d[6] +
		 p.d[4] * q.d[7] + p.d[7] * q.d[8],
		 p.d[2] * q.d[6] + p.d[5] * q.d[7] +
		 p.d[8] * q.d[8]}};
}

geom_vec3 mul_9x3(const geom_mat3 p, const geom_vec3 q)
{
   //Just the first row of mul_9x9.
   return {.d = {p.d[0] * q.d[0] + p.d[3] * q.d[1] +
		 p.d[6] * q.d[2], p.d[1] * q.d[0] +
		 p.d[4] * q.d[1] + p.d[7] * q.d[2],
		 p.d[2] * q.d[0] + p.d[5] * q.d[1] +
		 p.d[8] * q.d[2]}};
}

geom_mat4 mul_16x16(const geom_mat4 p, const geom_mat4 q)
{
   return {.d = {p.d[0] * q.d[0] + p.d[4] * q.d[1] +
		 p.d[8] * q.d[2] + p.d[12] * q.d[3],
		 p.d[1] * q.d[0] + p.d[5] * q.d[1] +
		 p.d[9] * q.d[2] + p.d[13] * q.d[3],
		 p.d[2] * q.d[0] + p.d[6] * q.d[1] +
		 p.d[10] * q.d[2] + p.d[14] * q.d[3],
		 p.d[3] * q.d[0] + p.d[7] * q.d[1] +
		 p.d[11] * q.d[2] + p.d[15] * q.d[3],
		 p.d[0] * q.d[4] +  p.d[4] * q.d[5] + //
		 p.d[8] * q.d[6] + p.d[12] * q.d[7],
		 p.d[1] * q.d[4] + p.d[5] * q.d[5] +
		 p.d[9] * q.d[6] + p.d[13] * q.d[7],
		 p.d[2] * q.d[4] + p.d[6] * q.d[5] +
		 p.d[10] * q.d[6] + p.d[14] * q.d[7],
		 p.d[3] * q.d[4] + p.d[7] * q.d[5] +
		 p.d[11] * q.d[6] + p.d[15] * q.d[7],
		 p.d[0] * q.d[8] + p.d[4] * q.d[9] + //
		 p.d[8] * q.d[10] + p.d[12] * q.d[11],
		 p.d[1] * q.d[8] + p.d[5] * q.d[9] +
		 p.d[9] * q.d[10] + p.d[13] * q.d[11],
		 p.d[2] * q.d[8] + p.d[6] * q.d[9] +
		 p.d[10] * q.d[10] + p.d[14] * q.d[11],
		 p.d[3] * q.d[8] + p.d[7] * q.d[9] +
		 p.d[11] * q.d[10] + p.d[15] * q.d[11],
		 p.d[0] * q.d[12] + p.d[4] * q.d[13] + //
		 p.d[8] * q.d[14] + p.d[12] * q.d[15],
		 p.d[1] * q.d[12] + p.d[5] * q.d[13] +
		 p.d[9] * q.d[14] + p.d[13] * q.d[15],
		 p.d[2] * q.d[12] + p.d[6] * q.d[13] +
		 p.d[10] * q.d[14] + p.d[14] * q.d[15],
		 p.d[3] * q.d[12] + p.d[7] * q.d[13] +
		 p.d[11] * q.d[14] + p.d[15] * q.d[15]}};
}

geom_vec4 mul_16x4(const geom_mat4 p, const geom_vec4 q)
{
   //Again, just the first row of mul_16x16.
   return {.d = {p.d[0] * q.d[0] + p.d[4] * q.d[1] +
		 p.d[8] * q.d[2] + p.d[12] * q.d[3],
		 p.d[1] * q.d[0] + p.d[5] * q.d[1] +
		 p.d[9] * q.d[2] + p.d[13] * q.d[3],
		 p.d[2] * q.d[0] + p.d[6] * q.d[1] +
		 p.d[10] * q.d[2] + p.d[14] * q.d[3],
		 p.d[3] * q.d[0] + p.d[7] * q.d[1] +
		 p.d[11] * q.d[2] + p.d[15] * q.d[3]}};
}

#ifdef GEOM_CPP
} //extern "C"
#endif

#ifdef GEOM_CPP
namespace geom
{
//TODO class vec2

class vec3;
class vec4;
class mat3;
class mat4;
class quaternion;
class axisAngle;

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
   friend vec3 operator* (const mat3 p, const vec3 q);
   vec3 operator/ (const float scalar) const { return vec3(div_3x1(vec, scalar)); }

   float len() const { return len_3(vec); }
   vec3 norm() const { return vec3(norm_3(vec)); }
   vec3 inverse() const { return vec3(inverse_3(vec)); }

   bool isUnit(float epsilon = 0.01) const { return is_unit_3(vec, epsilon); }

   friend float dot(const vec3 p, const vec3 q) { return dot_3x3(p.vec, q.vec); }
   friend vec3 cross(const vec3 p, const vec3 q) { return vec3(cross_3x3(p.vec, q.vec)); }
};

class vec4
//vec4s are not very useful except in formatting for uploads. I've
//left them barebones to discourage their use.
{
private:
   geom_vec4 vec;

   vec4 (geom_vec4 xyzw) : vec (xyzw) {}

public:

   vec4 (float x, float y, float z, float w)
      : vec ({.d = {x, y, z, w}}) {}

   const float& operator[] (const size_t ind) const
   {
      return vec.d[ind];
   }

   float& operator[] (const size_t ind)
   {
      return vec.d[ind];
   }
   
   friend vec4 operator* (const mat4 p, const vec4 q);
};

class mat3
{
private:
   geom_mat3 mat;

   mat3 (geom_mat3 abc) : mat (abc) {}

public:

   mat3 (float a, float b, float c,
	 float d, float e, float f,
	 float g, float h, float i)
      : mat ({.d = {a, b, c, d, e, f, g, h, i}}) {}

   const float& operator[] (const size_t ind) const
   {
      return mat.d[ind];
   }

   float& operator[] (const size_t ind)
   {
      return mat.d[ind];
   }
      
   mat3 operator* (const mat3 q) const { return mat3(mul_9x9(mat, q.mat)); }
   friend vec3 operator* (const mat3 p, const vec3 q);
};

vec3 operator* (const mat3 p, const vec3 q) { return vec3(mul_9x3(p.mat, q.vec)); }

class mat4
{
private:
   geom_mat4 mat;

   mat4 (geom_mat4 abcd) : mat(abcd) {}

public:
   mat4 (float a, float b, float c, float d,
	 float e, float f, float g, float h,
	 float i, float j, float k, float l,
	 float m, float n, float o, float p)
      : mat ({.d = {a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p}}) {}

   const float& operator[] (const size_t ind) const
   {
      return mat.d[ind];
   }

   float& operator[] (const size_t ind) { return mat.d[ind]; }

   mat4 operator* (const mat4 q) { return mat4(mul_16x16(mat, q.mat)); }
   friend vec4 operator* (const mat4 p, const vec4 q);
};

vec4 operator* (const mat4 p, const vec4 q) { return vec4(mul_16x4(p.mat, q.vec)); }

class axisAngle
{
private:
   geom_aa aa;
   
public:
   axisAngle(const vec3 ax, const float ang) : aa ({ax[0], ax[1], ax[2], ang}) {}

   vec3 axis() const { return vec3(aa.d[0], aa.d[1], aa.d[2]); }
   float angle() const { return aa.d[3]; }

   //TODO axisAngle(quaternion)? For debugging etc.
};

class quaternion
{
private:
   geom_quat qu;

   quaternion(const geom_quat q) : qu (q) {}
   
public:
   quaternion (const axisAngle aa) : qu (make_q({aa.axis()[0], aa.axis()[1], aa.axis()[2], aa.angle()})) {}

   quaternion operator* (const quaternion q) const { return quaternion(mul_qxq(qu, q.qu)); }
   
   //Assumes this quaternion is unit-sized.
   vec3 rotate(const vec3 q) const
   {
      geom_vec3 result = rotate_qx3(qu, {q[0], q[1], q[2]});
      
      return vec3(result.d[0], result.d[1], result.d[2]);
   }
      
   bool isUnit(float epsilon) const { return is_unit_q(qu, epsilon); }
   quaternion norm() const { return quaternion(norm_q(qu)); }
   float len() const { return len_q(qu); }
   quaternion inverse() const { return quaternion(inverse_q(qu)); }
   quaternion conjugate() const { return quaternion(conjugate_q(qu)); }

   //TODO getMatrix?
};

}
#endif
