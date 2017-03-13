/*
 * wahba_rot.h
 *
 *  Created on: Jul 20, 2014
 *      Author: mobintu
 */

#ifndef WAHBA_ROT_H_
#define WAHBA_ROT_H_

#include "math_utils.h"

#define TOL EPS_TOL


typedef struct {
	real w_a;
	real w_m;
	real a_r[3];
	real m_r[3];
	real g_va[3];
	real g_vm[3];
	real q[4];
	real RotM[3][3];
	real RotM_prev[3][3];
	real dR[3][3];
	real dth;
	real W[3];
	real Euler[3];
	real AccErr;
	real dt;
}wahba_rotStruct;

void wahba_StructInit(wahba_rotStruct *WStruct,real dt);
void wahba_rot(real acc[],real gyr[],real mag[],wahba_rotStruct *WStruct);
void WahbaJacobiSVDRotM(real (*A)[3], real (*RotM)[3]);

void jacobi2Sided(real (*A)[3],real (*U)[3],real (*V)[3]);
void apply_jacobi_R (real (*A)[3],int p,int q,real c,real s,real k);
void apply_jacobi_L (real (*A)[3],int p,int q,real c,real s,real k);


//Rot2Euler
static inline void rotmtx2euler(const real *rotmtx, real *euler)
{
	/*0 1 2  0 3 6
      3 4 5  1 4 7
	  6 7 8  2 5 8
	 */

	// Rbe --> euler
	const real tol = 0.99999f;
	const real test = -rotmtx[2];
	if (test > +tol)
	{
		euler[0] = atan2v(-rotmtx[7], rotmtx[4]);
		euler[1] = +0.5f * M_PI;
		euler[2] = 0.0f;
	}
	else if (test < -tol)
	{
		euler[0] = atan2v(-rotmtx[7], rotmtx[4]);
		euler[1] = -0.5f * M_PI;
		euler[2] = 0.0f;
	}
	else
	{
		euler[0] = atan2v (rotmtx[5], rotmtx[8]);
		euler[1] = asinv (-rotmtx[2]); //3-4us!!
		euler[2] = atan2v (rotmtx[1], rotmtx[0]);
	}
}


static inline void dRpropagat(real (*Rbe)[3],real (*R)[3],real *wb,real dt){
	real norm,th,c,s,u;
	real w[3];
	real dR[3][3];

	mulMatrTrVec(w,R,wb); // w body to earth;

	norm =normV(w);
	if (norm<1e-6){
		for(int i=0;i<3;i++){
			dR[0][i]=0;
			dR[1][i]=0;
			dR[2][i]=0;
		}
		dR[0][0]=1;
		dR[1][1]=1;
		dR[2][2]=1;
	}
	else{
		w[0] = w[0]/norm;
		w[1] = w[1]/norm;
		w[2] = w[2]/norm;

		th = norm*dt;
		c = cosv(th);
		s = sinv(th);
		u = 1 - c;

		dR[0][0] = w[0]*w[0]*u + c;
		dR[0][1] = w[0]*w[1]*u - w[2]*s;
		dR[0][2] = w[0]*w[2]*u + w[1]*s;
		dR[1][0] = w[1]*w[0]*u + w[2]*s;
		dR[1][1] = w[1]*w[1]*u + c;
		dR[1][2] = w[1]*w[2]*u - w[0]*s;
		dR[2][0] = w[2]*w[0]*u - w[1]*s;
		dR[2][1] = w[2]*w[1]*u + w[0]*s;
		dR[2][2] = w[2]*w[2]*u + c;

		mulMatrMatrTr(Rbe,R,dR);
	}
}


static inline void dRpropagatInfinite(real (*Rbe)[3],real (*R)[3],real *wb,real dt){
	for (int i=0;i<3;i++){
		Rbe[0][i]=      0*R[0][i] + wb[2]*R[1][i] - wb[1]*R[2][i];
		Rbe[1][i]= -wb[2]*R[0][i] +     0*R[1][i] + wb[0]*R[2][i];
		Rbe[2][i]=  wb[1]*R[0][i]  -wb[0]*R[1][i] +     0*R[2][i];
	}
}

//Rot2quat
static inline void rotmtx2quat (const real *rotmtx, Quat4 *q)
{
	q->z = 0.0f;
	real sqtrp1;
	real sqtrp1x2;
	real d[3];
	real sqdip1;


	const real tr = rotmtx[0] + rotmtx[4] + rotmtx[8];
	if (tr>1e-6f)
	{
		sqtrp1 = sqrtv(tr + 1.0f);
		sqtrp1x2 = 2.0*sqtrp1;

		q->w = 0.5f*sqtrp1;
		q->x = (rotmtx[7] - rotmtx[5])/sqtrp1x2;
		q->y = (rotmtx[2] - rotmtx[6])/sqtrp1x2;
		q->z = (rotmtx[3] - rotmtx[1])/sqtrp1x2;
	}
	else
	{
		d[0]=rotmtx[0];
		d[1]=rotmtx[4];
		d[2]=rotmtx[8];

		if ((d[1] > d[0]) && (d[1] > d[2]))
		{
			sqdip1 = sqrtv(d[1] - d[0] - d[2] + 1.0f );
			q->y = 0.5f*sqdip1;

			if ( fabsv(sqdip1) > 1e-6f)
				sqdip1 = 0.5f/sqdip1;

			q->w = (rotmtx[2] - rotmtx[6])*sqdip1;
			q->x = (rotmtx[3] + rotmtx[1])*sqdip1;
			q->z = (rotmtx[7] + rotmtx[5])*sqdip1;
		}
		else if (d[2] > d[0])
		{
			//max value at R(3,3)
			sqdip1 = sqrtv(d[2] - d[0] - d[1] + 1.0f);

			q->z = 0.5f*sqdip1;

			if ( fabsv(sqdip1) > 1e-6f )
				sqdip1 = 0.5f/sqdip1;

			q->w = (rotmtx[3] - rotmtx[1])*sqdip1;
			q->x = (rotmtx[2] + rotmtx[6])*sqdip1;
			q->y = (rotmtx[7] + rotmtx[5])*sqdip1;
		}
		else
		{
			// max value at R(1,1)
			sqdip1 = sqrtv(d[0] - d[1] - d[2] + 1.0f );

			q->x = 0.5f*sqdip1;

			if ( fabsv(sqdip1) > 1e-6f )
				sqdip1 = 0.5f/sqdip1;

			q->w = (rotmtx[7] - rotmtx[5])*sqdip1;
			q->y = (rotmtx[3] + rotmtx[1])*sqdip1;
			q->z = (rotmtx[2] + rotmtx[6])*sqdip1;
		}
	}

}

#endif /* WAHBA_ROT_H_ */
