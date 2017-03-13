/*
 * wahba_rot.c//
 *
 *  Created on: Jul 20, 2014
 *      Author: mobintu
 */
#include "hw_conf.h"
#include "stdio.h"
#include "../inc/wahba_rot.h"



/**
 * @brief  This function initializes wahba_rotStruct. Change accordingly..
 * @param  WStruct, wahba_rotStruct
 * @param  dt, control loop period in (s)
 * @retval None
 */
void wahba_StructInit(wahba_rotStruct *WStruct,real dt){


	// Acc, mag weights (see paper.., briefly 1.0 to use only measurement, 0.0 to use only gyro)
	WStruct->w_a = 0.05;
	WStruct->w_m = 0.01;

	// Initial Reference magnetometer vector, will be updated during filter execution
	WStruct->m_r[0] = 0.707;
	WStruct->m_r[1] = 0.0;
	WStruct->m_r[2] = 0.707;

	// Reference gravity vector
	WStruct->a_r[0] = 0.0;
	WStruct->a_r[1] = 0.0;
	WStruct->a_r[2] = -1.0;

	// Control loop period
	WStruct->dt=dt;

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){
			WStruct->RotM[i][j]=0;
			WStruct->RotM_prev[i][j]=0;
		}
	for(int i=0;i<3;i++){
		WStruct->RotM[i][i]=1;
		WStruct->RotM_prev[i][i]=1;
	}

	for(int i=0;i<3;i++){
		WStruct->g_va[i] = 0;
		WStruct->g_vm[i] = 0;
		WStruct->W[i] = 0;
	}
}


/**
 * @brief  This function updates the rotation matrix estimation, WStruct.RotM (also updates Euler and quaternion)
 * @param  acc[3], accelerometer readings, a.u
 * @param  gyr[3], gyro reading in rad/s
 * @param  mag[3], magnetometer reading, a.u
 * @retval None
 */
void wahba_rot(real acc[3],real gyr[3],real mag[3],wahba_rotStruct *WStruct){

	static real w_a,w_ga,w_gm,w_m, A[3][3],dt;
	real (*RotM)[3];
	static int32_t cnt = 0;
	static real a_bn[3],m_bn[3],An,Mn,accok,magok;
	static real AccErr,*a_r,*w_g,*m_r,*g_va,*g_vm;//,*g_vg;
	static real Rbe[3][3];
	static real We[3];

	a_r = WStruct->a_r;
	m_r = WStruct->m_r;
	RotM = WStruct->RotM;
	g_va = WStruct->g_va;
	g_vm = WStruct->g_vm;
	dt = WStruct->dt;


	// Normalize readings
	An = normV(acc);
	Mn = normV(mag);

	accok = 1;
	if(An < TOL)
	{
		An = 1;
		accok = 0;
	}

	magok = 1;
	if(Mn < TOL)
	{
		Mn = 1;
		magok = 0;
	}

	w_a  = WStruct->w_a*accok;
	w_m  = WStruct->w_m*magok;

	w_ga = 1-w_a;
	w_gm = 1-w_m;


	for(int i=0;i<3;i++){
		a_bn[i] = acc[i]/An;
		m_bn[i] = mag[i]/Mn;
	}


	for(int i=0;i<3;i++){
		A[i][0] = w_a*a_bn[0]*a_r[i] + w_m*m_bn[0]*m_r[i] + w_ga*g_va[0]*a_r[i] + w_gm*g_vm[0]*m_r[i];
		A[i][1] = w_a*a_bn[1]*a_r[i] + w_m*m_bn[1]*m_r[i] + w_ga*g_va[1]*a_r[i] + w_gm*g_vm[1]*m_r[i];
		A[i][2] = w_a*a_bn[2]*a_r[i] + w_m*m_bn[2]*m_r[i] + w_ga*g_va[2]*a_r[i] + w_gm*g_vm[2]*m_r[i];
	}

	// Main SVD algorithm
	WahbaJacobiSVDRotM(A,RotM);

	// Propagate estimated R with current gyroscope reading
	dRpropagat(Rbe,RotM,gyr,dt);

	// Update reference magnetometer vector, if properly calibrated and correct initial is given it should not be used
	if(magok && (cnt<100)){
		mulMatrTrVec(m_r,Rbe,m_bn);
		m_r[0]=sqrtv(m_r[0]*m_r[0]+m_r[1]*m_r[1]);
		m_r[1]=0;
		cnt++;
	}

	// Virtual vectors calculations
	mulMatrVec(g_va,Rbe,a_r);
	mulMatrVec(g_vm,Rbe,m_r);

	// Euler, Quaternion calculations just for output
	// Conversion to euler takes 7-8us...
	rotmtx2euler((const real *)RotM,(real *)WStruct->Euler);
	rotmtx2quat((const real *)RotM,(Quat4 *)WStruct->q);

	// Calculate angular velocity W from dR, mainly unbiased gyr but more noisy due to derivative
	static real tmp;
	static real tr;

	mulMatrMatrTr(WStruct->dR,RotM,WStruct->RotM_prev);


	tr =  ((WStruct->dR[0][0] + WStruct->dR[1][1] + WStruct->dR[2][2]) - 1.0)/2.0 ;
	tr = SATUR(tr,1);
	tmp = acosv(tr);
	WStruct->dth = tmp;
	if (fabsv(tmp) < TOL){
		tmp = 0.5/dt;  // x/sin(x)|x->0 ->1
	}
	else{
		tmp = 0.5/dt*tmp/sinv(tmp);
	}
	We[0] = -tmp*(WStruct->dR[2][1] - WStruct->dR[1][2]);
	We[1] = -tmp*(WStruct->dR[0][2] - WStruct->dR[2][0]);
	We[2] = -tmp*(WStruct->dR[1][0] - WStruct->dR[0][1]);

	WStruct->W[0] = We[0];
	WStruct->W[1] = We[1];
	WStruct->W[2] = We[2];
	//mulMatrVec(WStruct->W,Rbe,We);



	for(int i = 0;i < 3; i++){
		WStruct->RotM_prev[0][i] = RotM[0][i];
		WStruct->RotM_prev[1][i] = RotM[1][i];
		WStruct->RotM_prev[2][i] = RotM[2][i];
	}
}


