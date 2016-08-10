
//-------------------------------------------------------------------
//		File: WMASTER\DEVICES\M117\m117fdm.cpp
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
//	Device:		Six-Pulse converter
//	Contents:	Frequency Domain Model Functions
//				averaging model
//
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\Multerm.h"
#include "\wmaster\solver\solvutil.h"
#include "M117.h"
//-------------------------------------------------------------------
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
//-------------------------------------------------------------------
BOOL M117::FQN_SingleFrequencyModel()
{
	//------debug report (optional)----------------------------------
	
	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugDeviceGuiData ();
		DebugDeviceInternalData ();
		DebugDeviceOptimalData ();
	}
	ndebg = 0;

	//------diagnostics----------------------------------------------

	if ( !InitSingleFrequencyModel_M117() ) return false;
	
	//------debug report (optional)----------------------------------

	ndebg = 0;
	if ( ndebg == 1 )
	{
		ReportFDDeviceModel ();
	}
	ndebg = 0;

	bValidDeviceCurrent = false;
	return true;
}
//-------------------------------------------------------------------
BOOL M117::FQN_SetDeviceInitialState()
{
	int	i,k;

	double dVac,dVdc,dPwr,dTemp;

	if ( !bNominalPhaseValid )
	{
		AfxMessageBox("Nominal Phase Not Defined for "+dev_title);
		return false;
	}

	//------define initial values and bases--------------------------

	dVac = dev_oparm[8] / sqrt(3.0);	// Phase Voltage (kV)
	dVdc = dev_oparm[11] / 2.0;			// DC Bus Nominal Voltage
	dPwr = dev_oparm[10] / 3.0;			// Rated Power per Phase (MW)

	//----terminal voltages - AC Side

	k = 0;
	for ( i=0 ; i<3 ; i++ )
	{
		dx0 = -TWODPI * float(i) / 3.0 + dNominalPhase[0] * DEG_TO_RAD;
		dStateInitQFD[k++] = dx1 * cos(dx0);
		dStateInitQFD[k++] = dx1 * sin(dx0);
	}

	//----terminal voltages - DC Side

	dStateInitQFD[k++] =  dVdc;
	dStateInitQFD[k++] =  0.0;

	dStateInitQFD[k++] = -dVdc;
	dStateInitQFD[k++] =  0.0;

	//----terminal voltages - Zero Crossing Control 

	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 0.0;

	//----terminal voltages - Positive Sequence Control

	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 0.0;

	//----terminal voltages - Power Control

	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 0.0;

	//----Internal AC Voltage Er, Ei

	dStateInitQFD[k++] = dStateInitQFD[0];
	dStateInitQFD[k++] = dStateInitQFD[1];

	//----Internal Cos(b),Cos(d)

	dStateInitQFD[k++] = cos(dFiringAngle);
	dStateInitQFD[k++] = cos(dFiringAngle);

	//----Internal X1 through X6

	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 1.0;
	dStateInitQFD[k++] = 1.0;

	//----Internal X7 through x10

	dStateInitQFD[k++] = 0.1 * dVac;
	dStateInitQFD[k++] = 0.1 * dVac;
	dStateInitQFD[k++] = 0.01 * dVac * dVac;
	dStateInitQFD[k++] = 1.0;

	if ( k != devsta_q )
	{
		AfxMessageBox("Internal Error 1001 in M117::FQN_SetDeviceInitialState()");
		return false;
	}

	//----Set Voltage base - AC Side

	k = 0;

	for ( i=0 ; i<3 ; i++ )
	{
		dStateBase[k++] = dVac;
		dStateBase[k++] = dVac;
	}

	//----Set Voltage base - DC Side

	for ( i=0 ; i<2 ; i++ )
	{
		dStateBase[k++] = dVdc;
		dStateBase[k++] = dVdc;
	}

	//----Set Voltage base - Controls

	for ( i=0 ; i<3 ; i++ )
	{
		dStateBase[k++] = 1.0;
		dStateBase[k++] = 1.0;
	}

	//----Set Voltage base - Er,Ei

	for ( i=0 ; i<1 ; i++ )
	{
		dStateBase[k++] = dVac;
		dStateBase[k++] = dVac;
	}

	//----Set Voltage base - cos(b),cos(d)

	for ( i=0 ; i<1 ; i++ )
	{
		dStateBase[k++] = 1.0;
		dStateBase[k++] = 1.0;
	}

	//----Set Voltage base - x1 through x10

	for ( i=0 ; i<5 ; i++ )
	{
		dStateBase[k++] = dVac;
		dStateBase[k++] = dVac;
	}

	if ( k != devsta_q )
	{
		AfxMessageBox("Internal Error 1002 in M117::FQN_SetDeviceInitialState()");
		return false;
	}

	//----Set Power bases - All Terminals (AC/DC/Conrtols)

	k = 0;
	for ( i=0 ; i<16 ; i++ )
	{
		dPowerBase[k++] = dPwr;
	}

	//----Set Power bases - next 8 equations

	dTemp = dVac * dVac;

	for ( i=0 ; i<8 ; i++ )
	{
		dPowerBase[k++] = dTemp;
	}

	//----Set Power bases - exception equation 22

	dPowerBase[22] = dVdc * dVdc;

	//----Set Power bases - next 2 equations

	dTemp = dPwr * dVac;

	for ( i=0 ; i<2 ; i++ )
	{
		dPowerBase[k++] = dTemp;
	}

	//----Set Power bases - next 2 equations

	dTemp = dVac * dVac * dVac;

	for ( i=0 ; i<2 ; i++ )
	{
		dPowerBase[k++] = dTemp;
	}

	//----Set Power bases - next 2 equations

	dTemp = dVac * dVac;

	for ( i=0 ; i<2 ; i++ )
	{
		dPowerBase[k++] = dTemp;
	}

	if ( k != devsta_q )
	{
		AfxMessageBox("Internal Error 1003 in M117::FQN_SetDeviceInitialState()");
		return false;
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M117::FQN_LinearizedSingleFrequencyModel()
{
		
	int		k;
	int		k1;
	int		i;
	int		j;
	int		i1;
	int		i2;
	int		j1;
	int		j2;
	double	v;

	for ( i=0 ; i<devsta_q ; i++ )
	{
		beq_real_q[i] = 0.0;
		for ( j=0 ; j<devsta_q ; j++ )
		{
			yeq_real_q[i][j] = 0.0;
		}
	}

	//------Fill yeq_real_q------------------------------------------

	if ( !InitSingleFrequencyModel_M117() ) return false;

	for ( k=0 ; k<nd_feq ; k++ )
	{
		k1	= feq_reals[k].scubix_k;
		i1	= feq_reals[k].scubix_i;
		j1	= feq_reals[k].scubix_j;
		v	= feq_reals[k].scubix_v;
		
		yeq_real_q[k1][j1]  += v * xa_real[i1] ;
		yeq_real_q[k1][i1]  += v * xa_real[j1] ;
		beq_real_q[k1]		+= v * xa_real[i1] * xa_real[j1];		
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M117::InitSingleFrequencyModel_M117()
{
	int		i, j, k;
	double	dXReactance;
	double	dBSusceptance;

	dCLInductance	= dev_oparm[6];
	dXReactance		= 2.0 * dCLInductance *  GetBaseOmega();
	dBSusceptance	= 1.0 / dXReactance;
	iDeviceFdmModel = 1;
	
	//------initialize generalized model-----------------------------

	nd_feq = 22;
	FQN_AllocateNonLinearModelArrays();

	for ( i=0 ; i<devsta_q ; i++ )
	{
		beq_real_q[i] = 0.0;
		for ( j=0 ; j<devsta_q ; j++ )
		{
			yeq_real_q[i][j] = 0.0;
		}
	}

	//------Compute Admittance Matrix / Linear part------------------

	yeq_real_q[0][1]	=   dBSusceptance;
	yeq_real_q[0][17]	= - dBSusceptance;

	yeq_real_q[1][0]	= - dBSusceptance;
	yeq_real_q[1][16]	=   dBSusceptance;

	yeq_real_q[2][3]	=   dBSusceptance;
	yeq_real_q[2][16]	=   0.5 * sqrt(3.0) * dBSusceptance;
	yeq_real_q[2][17]	=   0.5 * dBSusceptance;

	yeq_real_q[3][2]	= - dBSusceptance;
	yeq_real_q[3][16]	= - 0.5 * dBSusceptance;
	yeq_real_q[3][17]	=   0.5 * sqrt(3.0) * dBSusceptance;

	yeq_real_q[4][5]	=   dBSusceptance;
	yeq_real_q[4][16]	= - 0.5 * sqrt(3.0) * dBSusceptance;
	yeq_real_q[4][17]	=   0.5 * dBSusceptance;

	yeq_real_q[5][4]	= - dBSusceptance;
	yeq_real_q[5][16]	= - 0.5 * dBSusceptance;
	yeq_real_q[5][17]	= - 0.5 * sqrt(3.0) * dBSusceptance;

	yeq_real_q[6][6]	=   0.001 * dBSusceptance;
	yeq_real_q[6][8]	= - 0.001 * dBSusceptance;
	yeq_real_q[6][26]	=  (1.02) * ( 2.0 * DPI / sqrt(6.0) ) * dBSusceptance;
		
	yeq_real_q[7][7]	=  1.0;
	
	yeq_real_q[8][6]	= - 0.001 * dBSusceptance;
	yeq_real_q[8][8]	=   0.001 * dBSusceptance;
	yeq_real_q[8][26]	= - (1.02) * ( 2.0 * DPI / sqrt(6.0) ) * dBSusceptance;
		
	yeq_real_q[9][9]	=  1.0;

	yeq_real_q[10][10]	=  1.0;	
	yeq_real_q[11][11]	=  1.0;	
	yeq_real_q[12][12]	=  1.0;	
	yeq_real_q[13][13]	=  1.0;	
	yeq_real_q[14][14]	=  1.0;	
	yeq_real_q[15][15]	=  1.0;
	
	yeq_real_q[16][1]	= - 1.0;
	yeq_real_q[16][17]	=   1.0;
	yeq_real_q[16][20]	=   1.0;
	
	yeq_real_q[17][0]	=   1.0;
	yeq_real_q[17][16]	= - 1.0;
	yeq_real_q[17][21]	=   1.0;
	
	yeq_real_q[18][3]	= - 1.0;
	yeq_real_q[18][16]	= - 0.5 * sqrt(3.0);
	yeq_real_q[18][17]	= - 0.5;
	yeq_real_q[18][22]	=   1.0;

	yeq_real_q[19][2]	=   1.0;
	yeq_real_q[19][16]	=   0.5;
	yeq_real_q[19][17]	= - 0.5 * sqrt(3.0);
	yeq_real_q[19][23]	=   1.0;
		
	yeq_real_q[20][5]	= - 1.0;
	yeq_real_q[20][16]	=   0.5 * sqrt(3.0);
	yeq_real_q[20][17]	= - 0.5;
	yeq_real_q[20][24]	=   1.0;

	yeq_real_q[21][4]	=   1.0;
	yeq_real_q[21][16]	=   0.5;
	yeq_real_q[21][17]	=   0.5 * sqrt(3.0);
	yeq_real_q[21][25]	=   1.0;
		
	yeq_real_q[22][6]	=   1.0;
	yeq_real_q[22][8]	= - 1.0;
	
	yeq_real_q[28][28]	=   1.0;

	yeq_real_q[29][29]	=   1.0;
	
	//------define constant terms------------------------------------
		
	beq_real_q[25]		=   -dev_oparm[10];
	
	beq_real_q[24]		=   dev_oparm[10];
	
//	beq_real_q[25]		=   - dev_oparm[10] * 0.1;

	//------define nonlinear terms-----------------------------------
		
	k = 0;

	//------fill feq_reals - equation 22 ----------------------------

	feq_reals[k].scubix_k = 22;
	feq_reals[k].scubix_i = 19;
	feq_reals[k].scubix_j = 27;
	feq_reals[k].scubix_v = - (1.02) * 1.5 * sqrt(6.0) / DPI;
	k++;

	//------fill feq_reals - equation 23 ----------------------------

	feq_reals[k].scubix_k = 23;
	feq_reals[k].scubix_i = 16;
	feq_reals[k].scubix_j = 20;
	feq_reals[k].scubix_v = 1.0;
	k++;
		
	feq_reals[k].scubix_k = 23;
	feq_reals[k].scubix_i = 17;
	feq_reals[k].scubix_j = 21;
	feq_reals[k].scubix_v = 1.0;
	k++;
	
	feq_reals[k].scubix_k = 23;
	feq_reals[k].scubix_i = 18;
	feq_reals[k].scubix_j = 28;
	feq_reals[k].scubix_v = 1.0;
	k++;
	
	//------fill feq_reals - equation 24 ----------------------------

	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 0;
	feq_reals[k].scubix_j = 20;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 1;
	feq_reals[k].scubix_j = 21;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 2;
	feq_reals[k].scubix_j = 22;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 3;
	feq_reals[k].scubix_j = 23;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 4;
	feq_reals[k].scubix_j = 24;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 24;
	feq_reals[k].scubix_i = 5;
	feq_reals[k].scubix_j = 25;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	//------fill feq_reals - equation 25 ----------------------------
/*
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 1;
	feq_reals[k].scubix_j = 20;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 0;
	feq_reals[k].scubix_j = 21;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 3;
	feq_reals[k].scubix_j = 22;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 2;
	feq_reals[k].scubix_j = 23;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
		
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 5;
	feq_reals[k].scubix_j = 24;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 4;
	feq_reals[k].scubix_j = 25;
	feq_reals[k].scubix_v = - dBSusceptance;
	k++;
*/

	//-----new 25------------------------------

	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 6;
	feq_reals[k].scubix_j = 26;
	feq_reals[k].scubix_v = - (2.0 * DPI / sqrt(6.0) )* dBSusceptance;
	k++;
		
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 8;
	feq_reals[k].scubix_j = 26;
	feq_reals[k].scubix_v = (2.0 * DPI / sqrt(6.0) )* dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 6;
	feq_reals[k].scubix_j = 6;
	feq_reals[k].scubix_v = - 0.001 * dBSusceptance;
	k++;
	
	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 6;
	feq_reals[k].scubix_j = 8;
	feq_reals[k].scubix_v = 2.0 * 0.001 * dBSusceptance;
	k++;

	feq_reals[k].scubix_k = 25;
	feq_reals[k].scubix_i = 8;
	feq_reals[k].scubix_j = 8;
	feq_reals[k].scubix_v = - 0.001 * dBSusceptance;
	k++;
		
	//------fill feq_reals - equation 26 ----------------------------

	feq_reals[k].scubix_k = 26;
	feq_reals[k].scubix_i = 20;
	feq_reals[k].scubix_j = 20;
	feq_reals[k].scubix_v = - 1.0;
	k++;
		
	feq_reals[k].scubix_k = 26;
	feq_reals[k].scubix_i = 21;
	feq_reals[k].scubix_j = 21;
	feq_reals[k].scubix_v = - 1.0;
	k++;
	
	feq_reals[k].scubix_k = 26;
	feq_reals[k].scubix_i = 26;
	feq_reals[k].scubix_j = 26;
	feq_reals[k].scubix_v = 1.0;
	k++;
		
	//------fill feq_reals - equation 27 ----------------------------

	feq_reals[k].scubix_k = 27;
	feq_reals[k].scubix_i = 16;
	feq_reals[k].scubix_j = 16;
	feq_reals[k].scubix_v = - 1.0;
	k++;
		
	feq_reals[k].scubix_k = 27;
	feq_reals[k].scubix_i = 17;
	feq_reals[k].scubix_j = 17;
	feq_reals[k].scubix_v = - 1.0;
	k++;
	
	feq_reals[k].scubix_k = 27;
	feq_reals[k].scubix_i = 27;
	feq_reals[k].scubix_j = 27;
	feq_reals[k].scubix_v = 1.0;
	k++;
			
	//------fill feq_reals - equation 28 ----------------------------

	feq_reals[k].scubix_k = 28;
	feq_reals[k].scubix_i = 26;
	feq_reals[k].scubix_j = 27;
	feq_reals[k].scubix_v = - 1.0;
	k++;

	if ( nd_feq != k )
	{
		AfxMessageBox("Incorrect nd_feq in M117::InitSingleFrequencyModel_M117()");
		return false;
	}

	return true;
}
//-------------------------------------------------------------------
