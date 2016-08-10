//-------------------------------------------------------------------
//
//	Quasistatic Time Domain Converter Model
//
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\series.h"
#include "M117.h"
//-------------------------------------------------------------------
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
//-------------------------------------------------------------------
BOOL M117::QSN_PrepareNodeIndexArrays()
{
	int i,i1,k,nhalf,nsys;

	nhalf		= 2 * devsta;
	devsta_tq	= 2 * nhalf;
	nsys		= pNetSolver->nTrSysSystem;

	devosd_tq.resize(devsta_tq);

	k = 0;
	for ( i=0 ; i<devsta ; i++ )
	{
		devosd_tq[k++] = devosd[i] * 2;
		devosd_tq[k++] = devosd[i] * 2 + 1;
	}

	for ( i=0 ; i<nhalf ; i++ )
	{
		i1 = i + nhalf;
		devosd_tq[i1] = devosd_tq[i] + nsys;
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M117::QSN_QuasiSteadyStateInit()
{
	int i,j, nt;

	//------initialize generalized model-----------------------------

	devsta_tq	= 4 * devsta;
	nd_feq_tq	= 44;

	QSN_AllocateArrays();

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		xi_real_tq[i]  = 0.0;
		beq_real_tq[i] = 0.0;
		for ( j=0 ; j<devsta_tq ; j++ )
		{
			yeq_real_tq[i][j] = 0.0;
		}
	}
	
	//------debugging reports----------------------------------------
	//------debug report / input data--------------------------------

	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugDeviceGuiData ();
		DebugDeviceInternalData ();
		DebugDeviceOptimalData ();
	}

	switch (ioperatingmode)
	{
	case 1:		// rectifier mode
		switch (icontrol)
		{
		case 1:		// firing angle 
//			if ( !QSN_11_ComputeYeqMatrixM117() ) return false;
//			if ( !QSN_11_ComputeBeqVectorM117() ) return false;
//			if ( !QSN_11_ComputeFeqVectorM117() ) return false;
			amsr1.Format("Model 117 - Unsupported model options: 11");
			AfxMessageBox(amsr1);
			break;
		case 2:		// real power
			if ( !QSN_12_ComputeYeqMatrixM117() ) return false;
			if ( !QSN_12_ComputeBeqVectorM117() ) return false;
			if ( !QSN_12_ComputeFeqVectorM117() ) return false;
			break;
		case 3:		// DC voltage
//			if ( !QSN_13_ComputeYeqMatrixM117() ) return false;
//			if ( !QSN_13_ComputeBeqVectorM117() ) return false;
//			if ( !QSN_13_ComputeFeqVectorM117() ) return false;
			amsr1.Format("Model 117 - Unsupported model options: 13");
			AfxMessageBox(amsr1);
			break;
		default:
			amsr1.Format("Model 117 - Unidentified Power Flow control option");
			AfxMessageBox(amsr1);
			break;
		}
	case 2:		// inverter mode
		switch (icontrol)
		{
		case 1:		// firing angle 
//			if ( !QSN_21_ComputeYeqMatrixM117() ) return false;
//			if ( !QSN_21_ComputeBeqVectorM117() ) return false;
//			if ( !QSN_21_ComputeFeqVectorM117() ) return false;
			amsr1.Format("Model 117 - Unsupported model options: 21");
			AfxMessageBox(amsr1);
			break;
		case 2:		// real power
//			if ( !QSN_22_ComputeYeqMatrixM117() ) return false;
//			if ( !QSN_22_ComputeBeqVectorM117() ) return false;
//			if ( !QSN_22_ComputeFeqVectorM117() ) return false;
			amsr1.Format("Model 117 - Unsupported model options: 22");
			AfxMessageBox(amsr1);
			break;
		case 3:		// DC voltage
//			if ( !QSN_23_ComputeYeqMatrixM117() ) return false;
//			if ( !QSN_23_ComputeBeqVectorM117() ) return false;
//			if ( !QSN_23_ComputeFeqVectorM117() ) return false;
			amsr1.Format("Model 117 - Unsupported model options: 23");
			AfxMessageBox(amsr1);
			break;
		default:
			amsr1.Format("Model 117 - Unidentified Power Flow control option");
			AfxMessageBox(amsr1);
			break;
		}
	default:
		amsr1.Format("Model 117 - Unidentified operating mode");
		AfxMessageBox(amsr1);
		break;
	}


	//------intitialize state----------------------------------------
		
	j = 0;
	for ( i=0 ; i<devsta ; i++ )
	{
		xa_real_tq[j++]  = xav_cmpx[i].real();
		xa_real_tq[j++]  = xav_cmpx[i].imag();
	}
	
	//------initialize internal states, etc.-------------------------

	nt = devsta_tq / 2;
	for ( i=0 ; i<nt ; i++ )
	{
		xa_real_tq[i+nt]  = xa_real_tq[i];
	}

	//------compute arrays	yeq_real---------------------------------

	if ( !QSN_ComputeYeqMatrixM117() ) return false;

	//------compute past history

	if ( !QSN_ComputeBeqVectorM117() ) return false;

	//------compute quadratic terms

	if ( !QSN_ComputeFeqVectorM117() ) return false;

	//------set intitialization flags

	initialized = true;

	//------debug report / model output------------------------------

	ndebg = 0;
	if(ndebg == 1)
	{
		DebugTDDeviceModel();
	}

	//------set intitialization flags

	initialized = true;
	return true;
}
//-------------------------------------------------------------------
BOOL M117::QSN_QuasiSteadyStateReInit()
{
	return true;
}
//-------------------------------------------------------------------
BOOL M117::QSN_QuasiSteadyStateTimeStep()
{
	int		i,j;
	int		k;
	int		n;
	double	run_time;
	double	v;

	//------Compute terminal currents

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		dx0 = - beq_real_tq[i];
		for ( j=0 ; j<devsta_tq ; j++ )
		{
			dx0 += yeq_real_tq[i][j] * xa_real_tq[j];
		}
		xi_real_tq[i] = dx0;
	}
	
	//------Compute terminal currents - Add contributions of nonlinear part

	for ( n=0 ; n<nd_feq_tq ; n++ )
	{
		k = feq_reals_tq[n].scubix_k;
		i = feq_reals_tq[n].scubix_i;
		j = feq_reals_tq[n].scubix_j;
		v = feq_reals_tq[n].scubix_v;
		xi_real_tq[k] += v * xa_real_tq[i] * xa_real_tq[j];
	}

	//------Compute beq_real_tq--------------------------------------
		
	beq_real_tq[24]		=   dev_oparm[10];
	beq_real_tq[25]		=  -dev_oparm[10];
	beq_real_tq[54]		=   dev_oparm[10];
	beq_real_tq[55]		=  -dev_oparm[10];
	
	//------Update Time----------------------------------------------

	run_time = pNetSolver->dtsecs * pNetSolver->iMajorIterCount;

	return true;
}
//-------------------------------------------------------------------
//
//	This procedure computes the constant terms in the model equation
//
//-------------------------------------------------------------------
BOOL M117::QSN_12_ComputeBeqVectorM117()
{
	int nhalf, i, ip;
		
	//------define constant terms------------------------------------
		
	beq_real_tq[24]		=   dev_oparm[10];
	beq_real_tq[25]		=  -dev_oparm[10];

	//------augment for time tm--------------------------------------

	nhalf = devsta_tq / 2;

	for ( i=0 ; i<nhalf ; i++ )
	{
		ip = i + nhalf;
		beq_real_tq[ip] = beq_real_tq[i];
	}
	
	return true;

}
//-------------------------------------------------------------------
//
//		this procedure determines the matrix 
//
//		----> yeq_real_tq
//
//		for the converter model 117
//
//-------------------------------------------------------------------
BOOL M117::QSN_12_ComputeYeqMatrixM117()
{

	int		i, j, k, ip, jp, nhalf;
	double	dXReactance;
	double	dBSusceptance;

	dCLInductance	= dev_oparm[6];
	dXReactance		= 2.0 * dCLInductance *  GetBaseOmega();
	dBSusceptance	= 1.0 / dXReactance;
	
	//------Compute Admittance Matrix / Linear part------------------

	yeq_real_tq[0][1]	=   dBSusceptance;
	yeq_real_tq[0][17]	= - dBSusceptance;

	yeq_real_tq[1][0]	= - dBSusceptance;
	yeq_real_tq[1][16]	=   dBSusceptance;

	yeq_real_tq[2][3]	=   dBSusceptance;
	yeq_real_tq[2][16]	=   0.5 * sqrt(3.0) * dBSusceptance;
	yeq_real_tq[2][17]	=   0.5 * dBSusceptance;

	yeq_real_tq[3][2]	= - dBSusceptance;
	yeq_real_tq[3][16]	= - 0.5 * dBSusceptance;
	yeq_real_tq[3][17]	=   0.5 * sqrt(3.0) * dBSusceptance;

	yeq_real_tq[4][5]	=   dBSusceptance;
	yeq_real_tq[4][16]	= - 0.5 * sqrt(3.0) * dBSusceptance;
	yeq_real_tq[4][17]	=   0.5 * dBSusceptance;

	yeq_real_tq[5][4]	= - dBSusceptance;
	yeq_real_tq[5][16]	= - 0.5 * dBSusceptance;
	yeq_real_tq[5][17]	= - 0.5 * sqrt(3.0) * dBSusceptance;

	yeq_real_tq[6][6]	=   0.001 * dBSusceptance;
	yeq_real_tq[6][8]	= - 0.001 * dBSusceptance;
	yeq_real_tq[6][26]	=  (1.02) * ( 2.0 * DPI / sqrt(6.0) ) * dBSusceptance;
		
	yeq_real_tq[7][7]	=  1.0;
	
	yeq_real_tq[8][6]	= - 0.001 * dBSusceptance;
	yeq_real_tq[8][8]	=   0.001 * dBSusceptance;
	yeq_real_tq[8][26]	= - (1.02) * ( 2.0 * DPI / sqrt(6.0) ) * dBSusceptance;
		
	yeq_real_tq[9][9]	=  1.0;

	yeq_real_tq[10][10]	=  1.0;	
	yeq_real_tq[11][11]	=  1.0;	
	yeq_real_tq[12][12]	=  1.0;	
	yeq_real_tq[13][13]	=  1.0;	
	yeq_real_tq[14][14]	=  1.0;	
	yeq_real_tq[15][15]	=  1.0;
	
	yeq_real_tq[16][1]	= - 1.0;
	yeq_real_tq[16][17]	=   1.0;
	yeq_real_tq[16][20]	=   1.0;
	
	yeq_real_tq[17][0]	=   1.0;
	yeq_real_tq[17][16]	= - 1.0;
	yeq_real_tq[17][21]	=   1.0;
	
	yeq_real_tq[18][3]	= - 1.0;
	yeq_real_tq[18][16]	= - 0.5 * sqrt(3.0);
	yeq_real_tq[18][17]	= - 0.5;
	yeq_real_tq[18][22]	=   1.0;

	yeq_real_tq[19][2]	=   1.0;
	yeq_real_tq[19][16]	=   0.5;
	yeq_real_tq[19][17]	= - 0.5 * sqrt(3.0);
	yeq_real_tq[19][23]	=   1.0;
		
	yeq_real_tq[20][5]	= - 1.0;
	yeq_real_tq[20][16]	=   0.5 * sqrt(3.0);
	yeq_real_tq[20][17]	= - 0.5;
	yeq_real_tq[20][24]	=   1.0;

	yeq_real_tq[21][4]	=   1.0;
	yeq_real_tq[21][16]	=   0.5;
	yeq_real_tq[21][17]	=   0.5 * sqrt(3.0);
	yeq_real_tq[21][25]	=   1.0;
		
	yeq_real_tq[22][6]	=   1.0;
	yeq_real_tq[22][8]	= - 1.0;
	
	yeq_real_tq[28][28]	=   1.0;

	yeq_real_tq[29][29]	=   1.0;

	//------augment for time tm--------------------------------------

	nhalf = devsta_tq / 2;

	for ( i=0 ; i<nhalf ; i++ )
	{
		ip = i + nhalf;
		for ( j=0 ; j<nhalf ; j++ )
		{
			jp = j + nhalf;
			yeq_real_tq[ip][jp] = yeq_real_tq[i][j];
		}
	}
	
	return true;

}
//-------------------------------------------------------------------
//
//		this procedure updates the nonlinear terms for model 149
//
//-------------------------------------------------------------------
BOOL M117::QSN_12_ComputeFeqVectorM117()
{

	int		k, n, nhalf, iOffset;
	double	dXReactance;
	double	dBSusceptance;

	dCLInductance	= dev_oparm[6];
	dXReactance		= 2.0 * dCLInductance *  GetBaseOmega();
	dBSusceptance	= 1.0 / dXReactance;

	//------define nonlinear terms-----------------------------------
		
	k = 0;

	//------fill feq_reals - equation 22 ----------------------------

	feq_reals_tq[k].scubix_k = 22;
	feq_reals_tq[k].scubix_i = 19;
	feq_reals_tq[k].scubix_j = 27;
	feq_reals_tq[k].scubix_v = - (1.02) * 1.5 * sqrt(6.0) / DPI;
	k++;

	//------fill feq_reals_tq - equation 23 ----------------------------

	feq_reals_tq[k].scubix_k = 23;
	feq_reals_tq[k].scubix_i = 16;
	feq_reals_tq[k].scubix_j = 20;
	feq_reals_tq[k].scubix_v = 1.0;
	k++;
		
	feq_reals_tq[k].scubix_k = 23;
	feq_reals_tq[k].scubix_i = 17;
	feq_reals_tq[k].scubix_j = 21;
	feq_reals_tq[k].scubix_v = 1.0;
	k++;
	
	feq_reals_tq[k].scubix_k = 23;
	feq_reals_tq[k].scubix_i = 18;
	feq_reals_tq[k].scubix_j = 28;
	feq_reals_tq[k].scubix_v = 1.0;
	k++;
	
	//------fill feq_reals_tq - equation 24 ----------------------------

	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 0;
	feq_reals_tq[k].scubix_j = 20;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 1;
	feq_reals_tq[k].scubix_j = 21;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 2;
	feq_reals_tq[k].scubix_j = 22;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 3;
	feq_reals_tq[k].scubix_j = 23;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 4;
	feq_reals_tq[k].scubix_j = 24;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals_tq[k].scubix_k = 24;
	feq_reals_tq[k].scubix_i = 5;
	feq_reals_tq[k].scubix_j = 25;
	feq_reals_tq[k].scubix_v = - dBSusceptance;
	k++;
	
	//------fill feq_reals_tq - equation 25 ----------------------------

	feq_reals_tq[k].scubix_k = 25;
	feq_reals_tq[k].scubix_i = 6;
	feq_reals_tq[k].scubix_j = 26;
	feq_reals_tq[k].scubix_v = - (2.0 * DPI / sqrt(6.0) )* dBSusceptance;
	k++;
		
	feq_reals_tq[k].scubix_k = 25;
	feq_reals_tq[k].scubix_i = 8;
	feq_reals_tq[k].scubix_j = 26;
	feq_reals_tq[k].scubix_v = (2.0 * DPI / sqrt(6.0) )* dBSusceptance;
	k++;
	
	feq_reals_tq[k].scubix_k = 25;
	feq_reals_tq[k].scubix_i = 6;
	feq_reals_tq[k].scubix_j = 6;
	feq_reals_tq[k].scubix_v = - 0.001 * dBSusceptance;
	k++;
	
	feq_reals_tq[k].scubix_k = 25;
	feq_reals_tq[k].scubix_i = 6;
	feq_reals_tq[k].scubix_j = 8;
	feq_reals_tq[k].scubix_v = 2.0 * 0.001 * dBSusceptance;
	k++;

	feq_reals_tq[k].scubix_k = 25;
	feq_reals_tq[k].scubix_i = 8;
	feq_reals_tq[k].scubix_j = 8;
	feq_reals_tq[k].scubix_v = - 0.001 * dBSusceptance;
	k++;
		
	//------fill feq_reals_tq - equation 26 ----------------------------

	feq_reals_tq[k].scubix_k = 26;
	feq_reals_tq[k].scubix_i = 20;
	feq_reals_tq[k].scubix_j = 20;
	feq_reals_tq[k].scubix_v = - 1.0;
	k++;
		
	feq_reals_tq[k].scubix_k = 26;
	feq_reals_tq[k].scubix_i = 21;
	feq_reals_tq[k].scubix_j = 21;
	feq_reals_tq[k].scubix_v = - 1.0;
	k++;
	
	feq_reals_tq[k].scubix_k = 26;
	feq_reals_tq[k].scubix_i = 26;
	feq_reals_tq[k].scubix_j = 26;
	feq_reals_tq[k].scubix_v = 1.0;
	k++;
		
	//------fill feq_reals_tq - equation 27 ----------------------------

	feq_reals_tq[k].scubix_k = 27;
	feq_reals_tq[k].scubix_i = 16;
	feq_reals_tq[k].scubix_j = 16;
	feq_reals_tq[k].scubix_v = - 1.0;
	k++;
		
	feq_reals_tq[k].scubix_k = 27;
	feq_reals_tq[k].scubix_i = 17;
	feq_reals_tq[k].scubix_j = 17;
	feq_reals_tq[k].scubix_v = - 1.0;
	k++;
	
	feq_reals_tq[k].scubix_k = 27;
	feq_reals_tq[k].scubix_i = 27;
	feq_reals_tq[k].scubix_j = 27;
	feq_reals_tq[k].scubix_v = 1.0;
	k++;
			
	//------fill feq_reals_tq - equation 28 ----------------------------

	feq_reals_tq[k].scubix_k = 28;
	feq_reals_tq[k].scubix_i = 26;
	feq_reals_tq[k].scubix_j = 27;
	feq_reals_tq[k].scubix_v = - 1.0;
	k++;

	nhalf = nd_feq_tq / 2;

	if ( k != nhalf )
	{
		amsr1.Format("Model 117 - (A) Incorrect number of nonlinear terms: expected %d, created %d",nhalf,k);
		AfxMessageBox(amsr1);
		return false;
	}

	//------Augment for time tm--------------------------------------------

	iOffset = devsta_tq / 2;

	for ( n=0 ; n<nhalf ; n++ )
	{
		feq_reals_tq[k].scubix_k = feq_reals_tq[n].scubix_k + iOffset;
		feq_reals_tq[k].scubix_i = feq_reals_tq[n].scubix_i + iOffset;
		feq_reals_tq[k].scubix_j = feq_reals_tq[n].scubix_j + iOffset;
		feq_reals_tq[k].scubix_v = feq_reals_tq[n].scubix_v;
		k++;
	}

	if ( k != nd_feq_tq )
	{
		amsr1.Format("Model 117 - (B) Incorrect number of nonlinear terms: expected %d, created %d",nd_feq_tq,k);
		AfxMessageBox(amsr1);
		return false;
	}

	return true;

}
//-------------------------------------------------------------------