/**
 ******************************************************************************
 **
 **  File          : math_utils.h
 **  Created on: Oct 8, 2014
 **
 **  Author        : 1. Panos Marantos (panosmarantos@gmail.com)
 **  			     2. Yannis Koveos (ykoveos@gmail.com)
 **
 **  Abstract      : Header File for MathUtils.c
 **
 **  Last Modified : Nov 9, 2015
 **
 ******************************************************************************
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

// INCLUDES ===================================================================
#include "arm_math.h"

//#define USE_DOUBLE

// MATH definition for double or single precision
#ifdef USE_DOUBLE

typedef double real;
#define EPS_TOL 1e-10
#define sqrtv sqrt
#define sinv sin
#define cosv cos
#define acosv acos
#define asinv asin
#define atan2v atan2
#define fabsv fabs
#define logv log
#else

typedef float real;
#define EPS_TOL 1e-6
#define sqrtv(x) arm_sqrt(x)
#define sinv sinf
#define cosv cosf
#define acosv acosf
#define asinv asinf
#define atan2v atan2f
#define fabsv fabsf
#define logv logf
#endif

#define EARTH_radius 6378137.0
#define EARTH_flattening 0.003352811
#define E_EARTH  (sqrt(2.0*EARTH_flattening - EARTH_flattening*EARTH_flattening))

// DEFINITIONS ================================================================
//General Maths
#define BIT(nr)                 (1UL << (nr))
#define bswap16(x) 				((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))
#define bswap16_u(x) 			((((x) >> 8)) | (((x)) << 8))

#define MIN(x,xmin)  			((x) < (xmin)?(x):(xmin))
#define MAX(x,xmax)  			((x) > (xmax)?(x):(xmax))
#define SIGN(x) 				(((x) > 0) - ((x) < 0))
#define SATUR(x,Lim) 			((x) > (Lim))?(Lim):(((x)<-(Lim))?(-(Lim)):(x))
#define SATUR2(x,Low,High) 		((x) > (High))?(High):(((x)<(Low))?(Low):(x))
#define ABS(x) 					(((x) > 0)?(x):-(x))
#define ROUND(x) 				((x) >= 0?(long)((x)+0.5):(long)((x)-0.5))


// Axis3 & Quaternion STRUCTURES ==============================================
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
//axis3 FIX Struct
typedef struct {
	real x;
	real y;
	real z;
	uint8_t age;
}Axis3; //13bytes or 25bytes

#pragma pack(1)
typedef struct {
	real x;
	real y;
	real z;
	real w;
}Quat4; //17bytes or 32bytes

#pragma pack(1)
typedef struct {
	real L[3];
	real Rbs[3][3];
} Pose;

#pragma pack(pop)   /* restore original alignment from stack */

#define Axis3_SIZE sizeof(Axis3);
#define Quat4_SIZE sizeof(Quat4);

// PROTOTYPES =================================================================
//a. For Quaternions
real quat_norm (const Quat4 *q);
void quat_normalize (Quat4 *q);
void quat_multiply (const Quat4 a, const Quat4 b, Quat4 *c);
void quat_inverse (const Quat4 q, Quat4 *q_inv);
void quat_divide (const Quat4 a, const Quat4 b, Quat4 *c);
void quat_rotation (const real* axis, Quat4 *qr);
void quat_vector3_rotate (Quat4 q, real* v, real* res);
void quat_normal_wcomplete (Quat4 *q);
void quat_equiv_wpos_get (Quat4 *q);
void quat_propagate (Quat4 quatp, real *wb, real dt,Quat4* quatn);

//b. Transformations between Euler - Quaternions - Rotation Matrix
void quat2Reb(const Quat4, real *rotmtx);
void quat2Rbe(const Quat4, real *rotmtx);
void Reb2quat(const real *rotmtx, Quat4 *q);
void Rbe2quat(const real *rotmtx, Quat4 *q);
void Rbe2quat2(const real *rotmtx, Quat4 *q);
void euler2Reb(const real*, real *rotmtx);
void euler2Rbe(const real*, real *rotmtx);
void Reb2euler(const real *rotmtx, real*);
void Rbe2euler(const real *rotmtx, real*);
void quat2euler(const Quat4 ,real*);
void euler2quat (const real*, Quat4 *);

//c. Handle Matrices and Vectors
void RxV(const real *mtx, real *vec, real *res);
void RTxV(const real *mtx, real *vec, real *res);
void vector3_cross (const real *a, const real *b, real *c);
void multi_mtx_mtx_3X3 (real *a, real *b, real *res);
void multi_mtxT_mtx_3X3 (real *a, real *b, real *res);

//d. Others
void MMfindDirection(Quat4 q, real *mag, real *magE);
void lla2ecef(double lat, double lon, double alt, double *ecef);
void lla2xyz(double lat, double lon, double alt, double *ecef0, double (*RNe)[3], double *ned);
void getRNe(double lat, double lon,double (*RNe)[3]);

//e. Inlines
/**
 * @brief  FPU square root of float value
 * @param  in, float value
 * @retval out, square root of in value
 */
static inline float arm_sqrt(float in)
{
	float out;
	arm_sqrt_f32(in,&out);
	return out;
}

/**
 * @brief  Matrix determinant
 * @param  A, matrix
 * @retval det(A)
 */
static inline real detMatr(real (*A)[3])
{
	return A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]-(A[0][2]*A[1][1]*A[2][0]+A[0][1]*A[1][0]*A[2][2]+A[0][0]*A[1][2]*A[2][1]);
}

/**
 * @brief  Matrix multiplication A=BxC
 * @param  A,B,C real 3x3 matrices, A will be overwritten with the result BxC
 * @retval None
 */
static inline void mulMatr(real (*A)[3],real (*B)[3],real (*C)[3]){
	int i;
	for (i=0;i<3;i++){
		A[0][i]=B[0][0]*C[0][i]+B[0][1]*C[1][i]+B[0][2]*C[2][i];
		A[1][i]=B[1][0]*C[0][i]+B[1][1]*C[1][i]+B[1][2]*C[2][i];
		A[2][i]=B[2][0]*C[0][i]+B[2][1]*C[1][i]+B[2][2]*C[2][i];
	}
}

/**
 * @brief  Matrix multiplication A=BxC^T
 * @param  A,B,C real 3x3 matrices, A will be overwritten with the result BxC^T
 * @retval None
 */
static inline void mulMatrMatrTr(real (*A)[3],real (*B)[3],real (*C)[3]){
	int i;
	for (i=0;i<3;i++){
		A[0][i]=B[0][0]*C[i][0]+B[0][1]*C[i][1]+B[0][2]*C[i][2];
		A[1][i]=B[1][0]*C[i][0]+B[1][1]*C[i][1]+B[1][2]*C[i][2];
		A[2][i]=B[2][0]*C[i][0]+B[2][1]*C[i][1]+B[2][2]*C[i][2];
	}
}

/**
 * @brief  Matrix multiplication A=B^TxC
 * @param  A,B,C real 3x3 matrices, A will be overwritten with the result BxC^T
 * @retval None
 */
static inline void mulMatrTrMatr(real (*A)[3],real (*B)[3],real (*C)[3]){
	int i;
	for (i=0;i<3;i++){
		A[i][0]=B[0][0]*C[i][0]+B[0][1]*C[i][1]+B[0][2]*C[i][2];
		A[i][1]=B[1][0]*C[i][0]+B[1][1]*C[i][1]+B[1][2]*C[i][2];
		A[i][2]=B[2][0]*C[i][0]+B[2][1]*C[i][1]+B[2][2]*C[i][2];
	}
}

/**
 * @brief  Matrix - Vector multiplication v=Ax
 * @param  v,x vectors of length 3, A real 3x3 matrix
 * @retval None
 */
static inline void mulMatrVec(real *v,real (*A)[3],real *x){
	v[0]=A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2];
	v[1]=A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2];
	v[2]=A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2];
}

/**
 * @brief  Matrix Transpose - Vector multiplication v=A'x
 * @param  v,x vectors of length 3, A real 3x3 matrix
 * @retval None
 */
static inline void mulMatrTrVec(real *v,real (*A)[3],real *x){
	int i;
	v[0]=A[0][0]*x[0]+A[1][0]*x[1]+A[2][0]*x[2];
	v[1]=A[0][1]*x[0]+A[1][1]*x[1]+A[2][1]*x[2];
	v[2]=A[0][2]*x[0]+A[1][2]*x[1]+A[2][2]*x[2];
}

static inline void mulVecVecT(real (*c)[3], real *a,real *b) {
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
		{
			c[i][j]=a[i]*b[j];
		}
}

/**
 * @brief  Vector Norm
 * @param  V 3x1 real vector
 * @retval real ||V||
 */
static inline real normV(real V[3]){
	return sqrtv(V[0]*V[0]+V[1]*V[1]+V[2]*V[2]);
}

/**
 * @brief  Vector Norm difference
 * @param  A,V 3x1 real vector
 * @retval real ||A-V||
 */
static inline real normVd(real A[3],real B[3]){
	real V[3];
	V[0]=A[0]-B[0];
	V[1]=A[1]-B[1];
	V[2]=A[2]-B[2];
	return sqrtv(V[0]*V[0]+V[1]*V[1]+V[2]*V[2]);
}

/**
 * @brief  Vector Dot product
 * @param  A,B 3x1 real vector
 * @retval real A*B
 */
static inline real dot(real A[3],real B[3]){
	return sqrtv(A[0]*B[0]+A[1]*B[1]+A[2]*B[2]);
}

static inline void eye3(real (*I)[3])
{
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
		{
			I[i][j]=0.0;
			if (i==j)
				I[i][j]=1.0;
		}
}

static inline void Skew(real (*S)[3], real *v){
	S[0][0]=0.0;
	S[0][1]=-v[2];
	S[0][2]=v[1];

	S[1][0]=v[2];
	S[1][1]=0.0;
	S[1][2]=-v[0];

	S[2][0]=-v[1];
	S[2][1]=v[0];
	S[2][2]=0.0;
}



#endif /* MATH_UTILS_H_ */
