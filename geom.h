#ifdef GEOM_CPP
#include <stdexcept>
#endif

typedef geom_vec2 float[2];
typedef geom_vec3 float[3];
typedef geom_vec4 float[4];
typedef geom_mat3 geom_vec3[3];
typedef geom_mat4 geom_vec4[4];
typedef geom_quat geom_vec4;
typedef geom_aa geom_vec4;

#ifdef GEOM_CPP
inline
#endif
float dot(const geom_vec3 p, const geom_vec3 q);

//...

float dot(const geom_vec3 p, const geom_vec3 q)
{
   return p[0] * q[0] +
      p[1] * q[1] +
      p[2] * q[2];
}

float dot(const geom_vec2 p, const geom_vec2 q)
{
   return p[0] * q[0] +
      p[1] * q[1];
}

geom_vec3 cross(const geom_vec3 p, const geom_vec3 q)
{
   return {p[1] * q[2] - p[2] * q[1],
	 p[2] * q[0] - p[0] * q[2],
	 p[0] * q[1] - p[1] * q[0]};
}

geom_vec3 add(const geom_vec3 p, const geom_vec3 q)
{
   return {p[0] + q[0],
	 p[1] + q[1],
	 p[2] + q[2]};
}

geom_vec3 sub(const geom_vec3 p, const geom_vec3 q)
{
   return {p[0] - q[0],
	 p[1] - q[1],
	 p[2] - q[2]};
}

geom_vec3 mul(const geom_vec3 p, const float scalar)
{
   return {p[0] * scalar,
	 p[1] * scalar,
	 p[2] * scalar};
}

geom_vec3 div(const geom_vec3 p, const float divisor)
{
   return {p[0] / divisor,
	 p[1] / divisor,
	 p[2] / divisor};
}

geom_vec2 div(const geom_vec2 p, const float divisor)
{
   return {p[0] / divisor,
	 p[1] / divisor};
}
	 
float len(const geom_vec2 p)
{
   return sqrtf(dot(p, p));
}

float len(const geom_vec3 p)
{
   return sqrtf(dot(p, p));
}

geom_vec3 norm(const geom_vec3 p)
{
   /*
     A unit vector has a length of 1.
     To normalise a vector (i.e., to get a unit vector version of it),
     divide it by its own length; its length will then be 1, in the
     same way that dividing a scalar by its own magnitude will return 1.
   */

   return div(p, len(p));
}

geom_vec3 inverse(const geom_vec3 p)
{
   return {-p[0],
	 -p[1],
	 -p[2]};
}

float len_quat(const geom_quat p)
{
   return sqrtf(p[3] * p[3] +
		p[0] * p[0] +
		p[1] * p[1] +
		p[2] * p[2]);
}

geom_quat norm_quat(const geom_quat p)
{
   float len = len_quat(p);

   return {p[0] / len,
	 p[1] / len,
	 p[2] / len,
	 p[3] / len};
}

bool is_unit_quat(const geom_quat p, float epsilon)
{
   //A quaternion is normalised iff it has a length of 1.
   //Its length is equal to sqrt(...).
   //sqrt(1) = 1.
   //So just check that ... == 1.

   float sum = p[3] * p[3] +
      p[0] * p[0] +
      p[1] * p[1] +
      p[2] * p[2];

   if ((sum > (1 + epsilon)) || (sum < (1 - epsilon)))
   {
      return false;
   }

   else { return true; }
}

geom_quat conjugate_quat(const geom_quat p)
{
   const geom_vec3& vec = p;

   geom_vec3 inv_vec = inverse(vec);

   return {inv_vec[0], inv_vec[1], inv_vec[2], p[3]};
}

geom_quat inverse_quat(const geom_quat p)
{
   const float epsilon = 0.01;

   geom_quat conj = conjugate_quat(p);

   if (is_unit_quat(p, epsilon))
   {
      return conj;
   }

   //Else divide conjugate's elements by sum of non-conjugate squares.
   else
   {
      float sum_sq = p[0] * p[0] +
	 p[1] * p[1] +
	 p[2] * p[2] +
	 p[3] * p[3];

      return {conj[0] / sum_sq,
	    conj[1] / sum_sq,
	    conj[2] / sum_sq,
	    conj[3] / sum_sq};
   }
}

geom_quat quat(const geom_aa p)
{
   const float& angle = p[3];
   const geom_vec3& axis = p;

   float sin_a = sin(radians(angle / 2.f));
   float cos_a = cos(radians(angle / 2.f));

   geom_vec3 vec = axis * sin_a;
   
   geom_quat quat = {vec[0], vec[1], vec[2], cos_a};

   const float epsilon = 0.01;
   
   if (!is_unit_quat(quat, epsilon))
   {
      quat = norm_quat(quat);
   }

   return quat;
}

geom_quat mul_quat(const geom_quat p, const geom_quat q)
{
   //If either involves an angle of 0, it doesn't involve any rotation.

   if (p[3] == 0.f) { return q; }

   else if (q[3] == 0.f) { return p; }

   const geom_vec3& pvec = p;
   const geom_vec3& qvec = q;

   geom_vec3 vec = cross(pvec, qvec) +
      mul(pvec, q[3]) +
      mul(qvec, p[3]);
   
   float w = p[3] * q[3] - dot(p, q);

   //NB unit quaternions' product should itself be a unit quaternion.
   return {vec[0], vec[1], vec[2], w};
}

geom_vec3 rotate(const geom_quat p, const geom_vec3 q)
{
   //p * q * conjugate(p)
   //conjugate(p) = -p[0], -p[1], -p[2], p[3]

   //For convenience in writing the below mul.
   geom_vec3 psq = {p[0] * p[0], p[1] * p[1], p[2] * p[2]};

   //This is just a simplified subset of a 3x3 matrix-vector mul.

   geom_vec3 res;

   res[0] = (q[0] * (0.5 - psq[1] - psq[2]) +
	     q[1] * (p[0] * p[1] - p[3] * p[2]) +
	     q[2] * (p[0] * p[2] + p[1] * p[2]));

   res[1] = (q[0] * (p[0] * p[1] + p[2] * p[3]) +
	     q[1] * (0.5 - psq[0] - psq[2]) + 
	     q[2] * (p[1] * p[2] - p[0] * p[3]));

   res[2] = (q[0] * (p[0] * p[2] - p[1] * p[3]) +
	     q[1] * (p[1] * p[2] + p[0] * p[4]) +
	     q[2] * (0.5 - psq[0] - psq[1]));

   res = mul(res, 2.f);

   return res;
}

#ifdef GEOM_CPP
namespace geom
{
   class vec3
   {
     private:
      geom_vec3 vec;
      
     public:
     vec3(float x, float y, float z) : vec ({.x = x, .y = y, .z = z}) {}

      const float& operator[] (const size_t ind) const
      {
	 return vec[ind];
      }
      
      vec3 operator+ (const vec3 q) const { return add(vec, q.vec); }
      vec3 operator- (const vec3 q) const { return sub(vec, q.vec); }
      vec3 operator* (const float scalar) const { return mul(vec, scalar); }
      vec3 operator/ (const float scalar) const { return mul(vec, scalar); }

      float getLength() const { return len(vec); }
      vec3 getNormal() const { return norm(vec); }
      vec3 getInverse() const { return inverse(vec); }
   };

   vec3 dot(const vec3 p, const vec3 q) { return dot(p.vec, q.vec); }
}
#endif
