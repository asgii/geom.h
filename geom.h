#pragma once

#include <cmath> //for fmod

#define GEOM_PI 3.14159265358979323846

#ifdef GEOM_CPP
extern "C" {
#endif

//Vectors
typedef struct geom_vec2 { float d[2]; } geom_vec2;
typedef struct geom_vec3 { float d[3]; } geom_vec3;
typedef struct geom_vec4 { float d[4]; } geom_vec4;

//3x3 matrix - column-major order
typedef struct geom_mat3 { float d[9]; } geom_mat3;

//4x4 matrix
typedef struct geom_mat4 { float d[16]; } geom_mat4;

//Quaternion (real part last)
typedef struct geom_quat { float d[4]; } geom_quat;

//Axis and angle (in that order)
typedef struct geom_aa { float d[4]; } geom_aa;

//Convert degrees to radians
float radians(float degr);

//Dot-product
float dot_3x3(const geom_vec3 p, const geom_vec3 q);
float dot_2x2(const geom_vec2 p, const geom_vec2 q);

//Cross-product
geom_vec3 cross_3x3(const geom_vec3 p, const geom_vec3 q);

//Vector and vector-scalar arithmetic
geom_vec3 add_3x3(const geom_vec3 p, const geom_vec3 q);
geom_vec3 sub_3x3(const geom_vec3 p, const geom_vec3 q);   
geom_vec3 mul_3x1(const geom_vec3 p, const float scalar);   
geom_vec3 div_3x1(const geom_vec3 p, const float divisor);
geom_vec2 div_2x1(const geom_vec2 p, const float divisor);

//Get magnitude of a vector
float len_2(const geom_vec2 p);   
float len_3(const geom_vec3 p);

//Get normalised form of vector (ie, divided by its length)
geom_vec3 norm_3(const geom_vec3 p);

//Get inverse of vector
geom_vec3 inverse_3(const geom_vec3 p);

//Get whether vector is of unit-length (within margin, 'epsilon')
int is_unit_2(const geom_vec2 p, float epsilon);
int is_unit_3(const geom_vec3 p, float epsilon);

//Quaternion constructor
geom_quat make_q(const geom_aa p);

//Vector operations as above, for quaternions
float len_q(const geom_quat p);
geom_quat norm_q(const geom_quat p);
int is_unit_q(const geom_quat p, float epsilon);
geom_quat inverse_q(const geom_quat p);

//Conjugate of quaternion
geom_quat conjugate_q(const geom_quat p);

//Quaternion-specific multiplication
geom_quat mul_qxq(const geom_quat p, const geom_quat q);

//Rotate a vector (of any size) by quaternion
geom_vec3 rotate_qx3(const geom_quat p, const geom_vec3 q);

//Matrix multiplication
geom_mat3 mul_9x9(const geom_mat3 p, const geom_mat3 q);
geom_vec3 mul_9x3(const geom_mat3 p, const geom_vec3 q);
geom_mat4 mul_16x16(const geom_mat4 p, const geom_mat4 q);
geom_vec4 mul_16x4(const geom_mat4 p, const geom_vec4 q);

#ifdef GEOM_CPP
} //extern "C"

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

   vec3 (geom_vec3 xyz);
      
public:

   vec3 (float x, float y, float z);

   const float& operator[] (const size_t ind) const;
   float& operator[] (const size_t ind);
      
   vec3 operator+ (const vec3 q) const;
   vec3 operator- (const vec3 q) const;
   vec3 operator* (const float scalar) const;
   friend vec3 operator* (const mat3 p, const vec3 q);
   vec3 operator/ (const float scalar) const;

   float len() const;
   vec3 norm() const;
   vec3 inverse() const;

   bool isUnit(float epsilon = 0.01) const;

   //Implemented outside the class (as friend) so dot(p, q) works
   //rather than p.dot(q)
   friend float dot(const vec3 p, const vec3 q);
   friend vec3 cross(const vec3 p, const vec3 q);
};

class vec4
//vec4s are not very useful for graphics except in formatting for
//uploads to the server. I've left them barebones to discourage their use.
{
private:
   geom_vec4 vec;

   vec4 (geom_vec4 xyzw);

public:

   vec4 (float x, float y, float z, float w);

   const float& operator[] (const size_t ind) const;
   float& operator[] (const size_t ind);
   
   friend vec4 operator* (const mat4 p, const vec4 q);
};

class mat3
{
private:
   geom_mat3 mat;

   mat3 (geom_mat3 abc);

public:

   mat3 (float a, float b, float c,
	 float d, float e, float f,
	 float g, float h, float i);

   const float& operator[] (const size_t ind) const;
   float& operator[] (const size_t ind);
      
   mat3 operator* (const mat3 q) const;
   friend vec3 operator* (const mat3 p, const vec3 q);
};

vec3 operator* (const mat3 p, const vec3 q);

//Technically vec * mat should be a different operation. But I'm
//trying to discourage its use, to reduce confusion.
vec3 operator* (const vec3 p, mat3 q) = delete;

class mat4
{
private:
   geom_mat4 mat;

   mat4 (geom_mat4 abcd);

public:
   mat4 (float a, float b, float c, float d,
	 float e, float f, float g, float h,
	 float i, float j, float k, float l,
	 float m, float n, float o, float p);

   const float& operator[] (const size_t ind) const;
   float& operator[] (const size_t ind);

   mat4 operator* (const mat4 q);
   friend vec4 operator* (const mat4 p, const vec4 q);
};

vec4 operator* (const mat4 p, const vec4 q);
vec4 operator* (const vec4 p, const mat4 q) = delete;

class axisAngle
//Just an axis and an angle, for constructing quaternions
{
private:
   geom_aa aa;
   
public:
   axisAngle(const vec3 ax, const float ang);

   vec3 axis() const;
   float angle() const;

   //TODO axisAngle(quaternion)? For debugging etc.
};

class quaternion
{
private:
   geom_quat qu;

   quaternion(const geom_quat q);
   
public:
   quaternion (const axisAngle aa);

   quaternion operator* (const quaternion q) const;
   
   //Assumes this quaternion is unit-sized.
   vec3 rotate(const vec3 q) const;
      
   bool isUnit(float epsilon) const;
   quaternion norm() const;
   float len() const;
   quaternion inverse() const;
   quaternion conjugate() const;

   //TODO getMatrix?
};

} //namespace
#endif //GEOM_CPP

//Implementation

#ifdef GEOM_IMPL

//C implementation

#ifdef GEOM_CPP
extern "C" {
#endif
   
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
   return {.d = {p.d[0] * q.d[0] + p.d[3] * q.d[1] + p.d[6] * q.d[2], //1st column
		 p.d[1] * q.d[0] + p.d[4] * q.d[1] + p.d[7] * q.d[2],
		 p.d[2] * q.d[0] + p.d[5] * q.d[1] + p.d[8] * q.d[2],
		 p.d[0] * q.d[3] + p.d[3] * q.d[4] + p.d[6] * q.d[5], //2nd column
		 p.d[1] * q.d[3] + p.d[4] * q.d[4] + p.d[7] * q.d[5],
		 p.d[2] * q.d[3] + p.d[5] * q.d[4] + p.d[8] * q.d[5],
		 p.d[0] * q.d[6] + p.d[3] * q.d[7] + p.d[6] * q.d[8], //3rd column
		 p.d[1] * q.d[6] + p.d[4] * q.d[7] + p.d[7] * q.d[8],
		 p.d[2] * q.d[6] + p.d[5] * q.d[7] + p.d[8] * q.d[8]}};
}

geom_vec3 mul_9x3(const geom_mat3 p, const geom_vec3 q)
{
   //Just the first row of mul_9x9.
   return {.d = {p.d[0] * q.d[0] + p.d[3] * q.d[1] + p.d[6] * q.d[2],
		 p.d[1] * q.d[0] + p.d[4] * q.d[1] + p.d[7] * q.d[2],
		 p.d[2] * q.d[0] + p.d[5] * q.d[1] + p.d[8] * q.d[2]}};
}

geom_mat4 mul_16x16(const geom_mat4 p, const geom_mat4 q)
{
   return {.d = {p.d[0] * q.d[0] + p.d[4] * q.d[1] + p.d[8] * q.d[2] + p.d[12] * q.d[3],
		 p.d[1] * q.d[0] + p.d[5] * q.d[1] + p.d[9] * q.d[2] + p.d[13] * q.d[3],
		 p.d[2] * q.d[0] + p.d[6] * q.d[1] + p.d[10] * q.d[2] + p.d[14] * q.d[3],
		 p.d[3] * q.d[0] + p.d[7] * q.d[1] + p.d[11] * q.d[2] + p.d[15] * q.d[3],
		 //2nd column
		 p.d[0] * q.d[4] +  p.d[4] * q.d[5] + p.d[8] * q.d[6] + p.d[12] * q.d[7],
		 p.d[1] * q.d[4] + p.d[5] * q.d[5] + p.d[9] * q.d[6] + p.d[13] * q.d[7],
		 p.d[2] * q.d[4] + p.d[6] * q.d[5] + p.d[10] * q.d[6] + p.d[14] * q.d[7],
		 p.d[3] * q.d[4] + p.d[7] * q.d[5] + p.d[11] * q.d[6] + p.d[15] * q.d[7],
		 //3rd column
		 p.d[0] * q.d[8] + p.d[4] * q.d[9] + p.d[8] * q.d[10] + p.d[12] * q.d[11],
		 p.d[1] * q.d[8] + p.d[5] * q.d[9] + p.d[9] * q.d[10] + p.d[13] * q.d[11],
		 p.d[2] * q.d[8] + p.d[6] * q.d[9] + p.d[10] * q.d[10] + p.d[14] * q.d[11],
		 p.d[3] * q.d[8] + p.d[7] * q.d[9] + p.d[11] * q.d[10] + p.d[15] * q.d[11],
		 //4th column
		 p.d[0] * q.d[12] + p.d[4] * q.d[13] + p.d[8] * q.d[14] + p.d[12] * q.d[15],
		 p.d[1] * q.d[12] + p.d[5] * q.d[13] + p.d[9] * q.d[14] + p.d[13] * q.d[15],
		 p.d[2] * q.d[12] + p.d[6] * q.d[13] + p.d[10] * q.d[14] + p.d[14] * q.d[15],
		 p.d[3] * q.d[12] + p.d[7] * q.d[13] + p.d[11] * q.d[14] + p.d[15] * q.d[15]}};
}

geom_vec4 mul_16x4(const geom_mat4 p, const geom_vec4 q)
{
   //Again, just the first row of mul_16x16.
   return {.d = {p.d[0] * q.d[0] + p.d[4] * q.d[1] + p.d[8] * q.d[2] + p.d[12] * q.d[3],
		 p.d[1] * q.d[0] + p.d[5] * q.d[1] + p.d[9] * q.d[2] + p.d[13] * q.d[3],
		 p.d[2] * q.d[0] + p.d[6] * q.d[1] + p.d[10] * q.d[2] + p.d[14] * q.d[3],
		 p.d[3] * q.d[0] + p.d[7] * q.d[1] + p.d[11] * q.d[2] + p.d[15] * q.d[3]}};
}

#ifdef GEOM_CPP
} //extern "C"

//C++ implementation

namespace geom {

//vec3

vec3::vec3 (geom_vec3 xyz) : vec (xyz) {}
vec3::vec3 (float x, float y, float z) : vec ({.d = {x, y, z}}) {}

const float& vec3::operator[] (const size_t ind) const
{
   return vec.d[ind];
}

float& vec3::operator[] (const size_t ind)
{
   return vec.d[ind];
}
      
vec3 vec3::operator+ (const vec3 q) const { return vec3(add_3x3(vec, q.vec)); }
vec3 vec3::operator- (const vec3 q) const { return vec3(sub_3x3(vec, q.vec)); }
vec3 vec3::operator* (const float scalar) const { return vec3(mul_3x1(vec, scalar)); }
vec3 vec3::operator/ (const float scalar) const { return vec3(div_3x1(vec, scalar)); }

float vec3::len() const { return len_3(vec); }
vec3 vec3::norm() const { return vec3(norm_3(vec)); }
vec3 vec3::inverse() const { return vec3(inverse_3(vec)); }

bool vec3::isUnit(float epsilon) const { return is_unit_3(vec, epsilon); }

float dot(const vec3 p, const vec3 q) { return dot_3x3(p.vec, q.vec); }
vec3 cross(const vec3 p, const vec3 q) { return vec3(cross_3x3(p.vec, q.vec)); }

//vec4

vec4::vec4 (geom_vec4 xyzw) : vec (xyzw) {}
vec4::vec4 (float x, float y, float z, float w)
   : vec ({.d = {x, y, z, w}}) {}

const float& vec4::operator[] (const size_t ind) const { return vec.d[ind]; }
float& vec4::operator[] (const size_t ind) { return vec.d[ind]; }

//mat3

mat3::mat3 (geom_mat3 abc) : mat (abc) {}
mat3::mat3 (float a, float b, float c,
	    float d, float e, float f,
	    float g, float h, float i)
   : mat ({.d = {a, b, c, d, e, f, g, h, i}}) {}

const float& mat3::operator[] (const size_t ind) const { return mat.d[ind]; }
float& mat3::operator[] (const size_t ind) { return mat.d[ind]; }

mat3 mat3::operator* (const mat3 q) const { return mat3(mul_9x9(mat, q.mat)); }

vec3 operator* (const mat3 p, const vec3 q) { return vec3(mul_9x3(p.mat, q.vec)); }

//mat4

mat4::mat4 (geom_mat4 abcd) : mat(abcd) {}
mat4::mat4 (float a, float b, float c, float d,
	    float e, float f, float g, float h,
	    float i, float j, float k, float l,
	    float m, float n, float o, float p)
   : mat ({.d = {a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p}}) {}

const float& mat4::operator[] (const size_t ind) const { return mat.d[ind]; }
float& mat4::operator[] (const size_t ind) { return mat.d[ind]; }

mat4 mat4::operator* (const mat4 q) { return mat4(mul_16x16(mat, q.mat)); }

vec4 operator* (const mat4 p, const vec4 q) { return vec4(mul_16x4(p.mat, q.vec)); }

//axisAngle

axisAngle::axisAngle(const vec3 ax, const float ang)
: aa ({ax[0], ax[1], ax[2], ang}) {}

vec3 axisAngle::axis() const { return vec3(aa.d[0], aa.d[1], aa.d[2]); }
float axisAngle::angle() const { return aa.d[3]; }

//quaternion

quaternion::quaternion(const geom_quat q) : qu (q) {}
   
quaternion::quaternion (const axisAngle aa)
   : qu (make_q({aa.axis()[0], aa.axis()[1], aa.axis()[2], aa.angle()})) {}

quaternion quaternion::operator* (const quaternion q) const { return quaternion(mul_qxq(qu, q.qu)); }
   
//Assumes this quaternion is unit-sized.
vec3 quaternion::rotate(const vec3 q) const
{
   geom_vec3 result = rotate_qx3(qu, {q[0], q[1], q[2]});
      
   return vec3(result.d[0], result.d[1], result.d[2]);
}
      
bool quaternion::isUnit(float epsilon) const { return is_unit_q(qu, epsilon); }
quaternion quaternion::norm() const { return quaternion(norm_q(qu)); }
float quaternion::len() const { return len_q(qu); }
quaternion quaternion::inverse() const { return quaternion(inverse_q(qu)); }
quaternion quaternion::conjugate() const { return quaternion(conjugate_q(qu)); }

} //namespace
#endif //GEOM_CPP
#endif //GEOM_IMPL
