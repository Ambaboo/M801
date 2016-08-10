//-------------------------------------------------------------------
//
//		File: WMASTER\DEVICES\M117\m117tdm.cpp
//
//		Six-Pulse Converter Model
//
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\agc\agc.h"
#include "\wmaster\util\DbgDump.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\Series.h"
#include "M117.h"
//-------------------------------------------------------------------
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
//-------------------------------------------------------------------
//
//	Documentation of State and Current variables
//	
//	xa_real[ 0] = va(t-h)	terminal voltages	EXTERNAL STATES
//	xa_real[ 1] = vb(t-h)
//	xa_real[ 2] = vc(t-h)
//	xa_real[ 3] = vkathode(t-h)
//	xa_real[ 4] = vanode(t-h)
//	xa_real[ 5] = zero crossing
//	xa_real[ 6] = Pos Seq Magnitude
//	xa_real[ 7] = Net real Power
//	
//	xa_real[ 8] = vs1(t-h)	valve 1				INTERNAL STATES
//	xa_real[ 9] = vp1(t-h)
//	xa_real[10] = il1(t-h)
//	xa_real[11] = vs2(t-h)	valve 2
//	xa_real[12] = vp2(t-h)
//	xa_real[13] = il2(t-h)
//	xa_real[14] = vs3(t-h)	valve 3
//	xa_real[15] = vp3(t-h)
//	xa_real[16] = il3(t-h)
//	xa_real[17] = vs4(t-h)	valve 4
//	xa_real[18] = vp4(t-h)
//	xa_real[19] = il4(t-h)
//	xa_real[20] = vs5(t-h)	valve 5
//	xa_real[21] = vp5(t-h)
//	xa_real[22] = il5(t-h)
//	xa_real[23] = vs6(t-h)	valve 6
//	xa_real[24] = vp6(t-h)
//	xa_real[25] = il6(t-h)
//	
//	xi_real[ 0] = ia(t-h)	line currents		THROUGH VARIABLES
//	xi_real[ 1] = ib(t-h)
//	xi_real[ 2] = ic(t-h)
//	xi_real[ 3] = ikathode(t-h)
//	xi_real[ 4] = ianode(t-h)
//	xi_real[ 5] = 0.0 (ZX)
//	xi_real[ 6] = 0.0 (VMag)
//	xi_real[ 7] = 0.0 (P)
//	
//	xi_real[ 8] = 0.0		identical zero
//	.................		identical zero
//	xi_real[24] = 0.0		identical zero
//	xi_real[25] = 0.0		identical zero
//
//-------------------------------------------------------------------
BOOL M117::TTN_TimeDomainModelInit()
{
	int i;
	int	j;
	int iValve;

	//------debug report / input data

	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugDeviceGuiData();
		DebugDeviceInternalData();
		DebugDeviceOptimalData();
	}
	ndebg = 0;

	//------diagnostics

	if ( dev_oparm[4] < 0.001 )
	{
		amsr1.Format("Invalid Snubber Resistor Value (%.3f) in %s",
		dev_oparm[4],dev_title);
		AfxMessageBox(amsr1);
		return false;
	}

	if ( dev_oparm[5] < 0.001 )
	{
		amsr1.Format("Invalid Current Limiting Resistor Value (%.3f) in %s",
		dev_oparm[5],dev_title);
		AfxMessageBox(amsr1);
		return false;
	}

	//------initialize constants

	dTimePeriod				= 1.0 / GetBaseFrequency();
	dRunTimeNorm			= 0.0;
	dRunTimeNormRef			= 0.0;
	dPosSeqZeroTime			= 0.0;
	nBlockedCycles			= 1;
	nSubSteps				= 1;
	dHdtsecsSubM117			= pNetSolver->hdtsecs / double(nSubSteps);

	//------retrieve user defined parameters

	dValveONConductance		= dev_oparm[0];
	dValveOffConductance	= dev_oparm[1];
	dParasiticCapacitance	= dev_oparm[2];
	dSnubberCapacitance		= dev_oparm[3];
	dSnubberConductance		= 1.0 / dev_oparm[4];
	dCLRConductance			= 1.0 / dev_oparm[5];
	dCLInductance			= dev_oparm[6];
	dSmoothingCapacitance	= dev_oparm[7];
	dTimeDelay				= dTimePeriod * dev_oparm[9] / 360.0;
	iCControlM117			= dev_oparm[12];

	//------compute derived parameters

	dValveOffConductanceL	= dValveOffConductance * dCLInductance;
	dValveONConductanceL	= dValveONConductance * dCLInductance;
	dCLRConductanceL		= dCLRConductance * dCLInductance;
	dGvcConductance			= dParasiticCapacitance / pNetSolver->hdtsecs;
	dGvcConductanceL		= dGvcConductance * dCLInductance;
	dCpYeqReal				= (1.0 + dBetaM117 ) * dParasiticCapacitance / pNetSolver->hdtsecs;
	dCpPeqReal				= (1.0 - dBetaM117 ) * dParasiticCapacitance / pNetSolver->hdtsecs;
	dBlockedTime			= dTimePeriod * double(nBlockedCycles);

	//------allocate arrays

	AllocateTdmArraysLinear();
	AllocateDataModel_117();

	//------initialize arrays

	for ( i=0 ; i<devsta ; i++ )
	{
		xi_real[i]    = 0.0;
		xa_real[i]    = 0.0;
		beq_real[i]   = 0.0; 
		dPastState[i] = 0.0;
		for ( j=0 ; j<devsta ; j++ )
		{
			yeq_real[i][j] = 0.0;
		}
	}

	//------initialize valve status

	for ( iValve = 0; iValve<nValves ; iValve++ )
	{
		iValveStatusM117[iValve]	= 0;
		nValveDelayM117[iValve]		= 0;
		dCpPHCurrent[iValve]		= 0.0;
		dCpCurrent[iValve]			= 0.0;
		dSnubberCurrent[iValve]		= 0.0;
		dValveCurrentM117[iValve]	= 0.0;
		for ( i=0 ; i<nValTerm ; i++ )
		{
			dSValveCurrentM117[i][iValve]	= 0.0;
			dSValvePHCurrentM117[i][iValve]	= 0.0;
		}
	}

		iValveStatusM117[0]	= 1;
		iValveStatusM117[1]	= 1;


	//------update converter normalized time and time reference

	if ( !UpdateConverterRunTimeNormRef_117() ) return false;

	//------define connectivity pointers

	if ( !ComputeModel117Pointers() ) return false;

	//------initialize valve and capacitor matrices M and N

	if ( !InitValveAndCap_MandN_117() ) return false;

	//------Initialize valve conductance according to valve status

	if ( !DetermineValveConductance_M117() ) return false;

	//------compute converter matrices M and N

	if ( !ComputeConverter_MandN_117() ) return false;

	//------compute converter matrices dYeqSubStep and dPeqSubStep

	if ( !ComputeConverterSubStepYandP_117() ) return false;

	//------compute overall converter ACF model for initialization, use fixed time step scheme

	if ( !ComputeConverterFirstStepACF_117() ) return false;

	if ( !PrepareConverterCurrents_M117() ) return false;

	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugTDDeviceModel ();
		DebugReportModel117();
	}
	ndebg = 0;

	return true;
}
//-------------------------------------------------------------------
//	
//------this routine performs the time loop
//		computations for the model:
//		six-pulse converter
//
//-------------------------------------------------------------------
BOOL M117::TTN_TimeDomainModelTimeStep()
{
	int i;
	int j;
	int	iSubStep;
	int iValve;
	double	dStep1;
	double	dStep2;

	//------compute terminal currents

	for ( i=0 ; i<devsta ; i++ )
	{
		dx0 = - beq_real[i];
		for ( j=0 ; j<devsta ; j++ )
		{
			dx0 += yeq_real[i][j] * xa_real[j];
		}
		xi_real[i] = dx0;
	}
	
	//------do diagnostic loop

	ndebg = 0;
	if ( ndebg == 1 )
	{
		dx0 = 0.0;
	  	for ( i=devnod-3 ; i<devsta ; i++ )
		{
			dx0 += fabs(xi_real[i]);
		}
	  	if ( dx0 > 1.0e-6 )
		{
			amsr1.Format("NonZero InternalCurrents-Model 117: %10.8f",dx0); 
			DumpString(amsr1);
			AfxMessageBox(amsr1);
			return false;
		}
	}
	ndebg = 0;

	//-----update converter currents

	if ( !UpdateConverterCurrents_M117() ) return false;

	//------retrieve dPosSeqZeroTime and dPosSeqVMagn

	dPosSeqZeroTime = xa_real[5];
	dPosSeqVMagn	= xa_real[6];
	dNetRealPower	= xa_real[7];

	//------Determine if run time is within the blocked cycles
	//		iBlockedOperation = 1, means it is still within the blocked cycles
	//		and no need to compute valve status

	iBlockedOperation = 1;
	if ( (pNetSolver->run_time - dBlockedTime) > 0.0 ) iBlockedOperation = 1;

	//------update converter control reference

	if ( !UpdateConverterRunTimeNormRef_117() ) return false;

	//------Compute switching times (for given control scheme)
	//		only when time is after blocked cycles

	if ( !ComputeValveScheduleTimes_117() ) return false;

	if ( !ComputeValveSwitchingTimes_117() ) return false;

	//------Detect if any switching (actual) activity in next time step
	//------if iAnySwitching = 1, it means switching(s) will occur
	//------iBlockedOperation = 0 is a necessary condition for iAnySwitching = 1

	iAnySwitching = 0;
	if ( iBlockedOperation == 0 )
	{
		for ( iValve=0 ; iValve<nValves ; iValve++ )
		{
			if ( dValveSwitchTimeM117[iValve] > 0.0 ) iAnySwitching = 1;
		}
	}

	//------compute overall converter ACF model

	if ( nSubSteps == 1 )
	{
		if ( iAnySwitching == 1 )
		{
			if ( !ComputeValveState_117() ) return false;
			if ( !DetermineValveConductance_M117() )return false;
			if ( !ComputeConverter_MandN_117() )return false;
			if ( !ComputeConverterSubStepYandP_117() )return false;
			pNetSolver->bUpdateLnSystemFlag = true;
		}
		if ( !ComputeConverterFirstStepACF_117() )return false;
	}
	else if ( nSubSteps > 1 )
	{
		if ( iAnySwitching == 1 )
		{
			iSubStep = 0;

			if ( !DetermineValveStateAtSubStep_M117(iSubStep) ) return false;
			if ( !DetermineValveConductance_M117() ) return false;
			if ( !ComputeConverter_MandN_117() ) return false;

			dHdtsecsSubM117  = pNetSolver->hdtsecs / double(nSubSteps);

			if ( !ComputeConverterSubStepYandP_117() ) return false;
			if ( !ComputeConverterFirstStepACF_117() ) return false;

			for ( iSubStep=1 ; iSubStep<nSubSteps ; iSubStep++ )
			{
				if ( !DetermineValveStateAtSubStep_M117(iSubStep) ) return false;
				if ( !DetermineValveConductance_M117() ) return false;
				if ( !ComputeConverter_MandN_117() ) return false;

				dHdtsecsSubM117  = pNetSolver->hdtsecs / double(nSubSteps);

				if ( !ComputeConverterSubStepYandP_117() ) return false;

				dStep1 = dHdtsecsSubM117 * double(iSubStep) * 2.0;
				dStep2 = dHdtsecsSubM117 * 2.0;

				if ( !ComputeConverterTwoStepACF_117(dStep1,dStep2) ) return false;

				pNetSolver->bUpdateLnSystemFlag = true; 
			}
		}
		else
		{
			dHdtsecsSubM117  = pNetSolver->hdtsecs;

			if ( !ComputeConverterSubStepYandP_117() ) return false;
			if ( !ComputeConverterFirstStepACF_117() ) return false;
		}
	}

	for ( i=0 ; i<devsta ; i++ )
	{
		dPastState[i] = xa_real[i];
	}

	for ( i=0 ; i<nValves ; i++ )
	{
		dValveCurrentPastM117[i] = dValveCurrentM117[i];
	}

	if ( !PrepareConverterCurrents_M117() ) return false;

	//--------------debug

	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugPartialReportModel117(); 
	}
	ndebg = 0;

	return true;
}
//-------------------------------------------------------------------
//
//	Updates the normalized run time (dRunTimeNorm) and run time
//	reference (dRunTimeNormRef)
//
//-------------------------------------------------------------------
BOOL M117::UpdateConverterRunTimeNormRef_117()
{
	dRunTimeNorm = pNetSolver->run_time - dRunTimeNormRef;

	if ( dRunTimeNorm > dTimePeriod )
	{
		dRunTimeNormRef	+= dTimePeriod;
		dRunTimeNorm	-= dTimePeriod;
	}

	return true;
}






//-------------------------------------------------------------------
//
//------this routine computes the valve currents 
//		the results are stored in array:
//			dValveCurrent_M117
//
//-------------------------------------------------------------------
BOOL M117::PrepareConverterCurrents_M117()
{
	int	iValve;
	int k1,k2;
	int	i;
	int	j;
	int	j1;

	//------initialize matrices

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dMmatrixSvalve[4][0] = - dValveConductanceM117[iValve] - dGvcConductance;			
		dMmatrixSvalve[4][1] =   dValveConductanceM117[iValve];	
		dNmatrixSvalve[4][4] =   dValveConductanceM117[iValve] * dCLInductance + dCLRConductanceL + dGvcConductanceL;			
		for ( i=0 ; i<nValTerm ; i++ )
		{
			dx0 = 0.0;
			for ( j=0 ; j<nValTerm ; j++ )
			{
				j1  = iValvePointerM117[j][iValve];
				dx1 = dMmatrixSvalve[i][j];
				dx2 = dNmatrixSvalve[i][j] / dHdtsecsSubM117;
				dx3 = dx2 * dBetaM117;
				dx0 += ( - dx1 + dx2 - dx3 ) * xa_real[j1];
			}
			dSValvePHCurrentM117[i][iValve] = dx0 + dSValveCurrentM117[i][iValve];
		}
		k1 = iValvePointerM117[1][iValve];
		k2 = iValvePointerM117[3][iValve];
		dCpPHCurrent[iValve] = dCpPeqReal * ( xa_real[k1] - xa_real[k2] ) + dCpCurrent[iValve];
	}

	return true;
}
//-------------------------------------------------------------------
//
//------this routine computes the valve currents 
//		the results are stored in array:
//			dValveCurrent_M117
//
//-------------------------------------------------------------------
BOOL M117::UpdateConverterCurrents_M117()
{
	int	iValve;
	int k1,k2;
	int	i;
	int	j;
	int	j1;

	//------initialize matrices

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dMmatrixSvalve[4][0] = - dValveConductanceM117[iValve] - dGvcConductance;			
		dMmatrixSvalve[4][1] =   dValveConductanceM117[iValve];	
		dNmatrixSvalve[4][4] =   dValveConductanceM117[iValve] * dCLInductance + dCLRConductanceL + dGvcConductanceL;
		for ( i=0 ; i<nValTerm ; i++ )
		{
			dx0 = - dSValvePHCurrentM117[i][iValve];
			for ( j=0 ; j<nValTerm ; j++ )
			{
				j1  = iValvePointerM117[j][iValve];
				dx1 = dMmatrixSvalve[i][j];
				dx2 = dNmatrixSvalve[i][j] / dHdtsecsSubM117;
				dx3 = dx2 * dBetaM117;
				dx0 += ( dx1 + dx2 + dx3 ) * xa_real[j1];
			}
			dSValveCurrentM117[i][iValve] = dx0;
		}
		k1 = iValvePointerM117[1][iValve];
		k2 = iValvePointerM117[3][iValve];
		dCpCurrent[iValve]		= dCpYeqReal * ( xa_real[k1] - xa_real[k2] ) - dCpPHCurrent[iValve];
		k1 = iValvePointerM117[1][iValve];
		k2 = iValvePointerM117[2][iValve];
		dSnubberCurrent[iValve]   = dSnubberConductance * ( xa_real[k1] - xa_real[k2] );
		dValveCurrentM117[iValve] = dSValveCurrentM117[0][iValve] + dCpCurrent[iValve] + dSnubberCurrent[iValve];
	}
	
	//------do diagnostic loop

	ndebg = 0;
	if ( ndebg == 1 )
	{
		for ( iValve=0 ; iValve<nValves ; iValve++ )
		{
			dx0 = 0.0;
	  		for ( i=2 ; i<nValTerm ; i++ )
			{
				dx0 += fabs(dSValveCurrentM117[i][iValve]);
			}
	  		if ( dx0 > 1.0e-6 )
			{
				amsr1.Format("NonZero InternalCurrents-Model 117: %10.8f, Valve= %d",dx0,iValve); 
				DumpString(amsr1);
				AfxMessageBox(amsr1);
			}
		}
	}
	ndebg = 0;

	return true;
}
//-------------------------------------------------------------------
//
//		This routine computes the scheduled ON time
//			dValve1TurnOnTime for the first valve.
//		in addition, it computes the scheduled ON time
//			dValveNextTurnON[] for all the valves
//
//-------------------------------------------------------------------
BOOL M117::ComputeValveScheduleTimes_117()
{
	int iValve;
	
	//------update firing time signals

	dValve1TurnOnTime = dPosSeqZeroTime + dTimeDelay;
	dx1 = dValve1TurnOnTime - dRunTimeNorm;
	if ( dx1 < 0.0 )
	{
		dValve1TurnOnTime += dTimePeriod;
	}
	if ( dx1 > dTimePeriod )
	{
		dValve1TurnOnTime -= dTimePeriod;
	}
	dx0 = dTimePeriod / 6.0;
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveNextTurnOn[iValve] = dValve1TurnOnTime + dx0 * double(iValve);
	}

	//------adjust to have only positive values

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx1 = dValveNextTurnOn[iValve] - dRunTimeNorm;
		if ( dx1 < 0.0 )
		{
			dValveNextTurnOn[iValve] += dTimePeriod;
		}
		if ( dx1 > dTimePeriod )
		{
			dValveNextTurnOn[iValve] -= dTimePeriod;
		}
	}

	return true;
}
//-------------------------------------------------------------------
//
//------this routine computes the switching time
//		for each valve of the converter model.
//		The switching time is defined with reference 
//		the time (t-h).
//		The result is stored in variable:
//			dValveSwitchTimeM117
//
//		this routine also sets a logic delay to block the
//		OFF switching immediately after a valve is turned ON.
//		the delay time is stored in variable:
//			dLogicDelay
//
//		this routine also implements a logic to improve
//		starting process, i.e. to turn ON valve i-1 when
//		truning ON valve i if valve i-1 is positively biased.
//
//-------------------------------------------------------------------
BOOL M117::ComputeValveSwitchingTimes_117()
{
	int iValve;
	int iValveM;

	iValveM = -1;

//------initialize dValveSwitchTimeM117 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveSwitchTimeM117[iValve] = -1.0;
	}

//------compute dValveSwitchTimeM117 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{

//------compute dValveSwitchTimeM117 for ON -> OFF switching in [t-h,t]

		if ( iValveStatusM117[iValve] == 1 )
		{
			if ( nValveDelayM117[iValve] > 0 ) nValveDelayM117[iValve]--;
			if ( nValveDelayM117[iValve] == 0 )
			{
				dx0 = dValveCurrentM117[iValve] / dValveConductanceM117[iValve];		
				dx1 = dValveCurrentPastM117[iValve] / dValveConductanceM117[iValve];	
				if ( dx0 <= 0.0 )
				{
					amsr1.Format("\nComputeValveSwitchingTimes: Valve# : %d, V_bias(t-2h) =%.6f, iBlockedOperation = %d\n",iValve, dx1,iBlockedOperation);
					DumpString(amsr1);
					amsr1.Format("ComputeValveSwitchingTimes: Valve# : %d, V_bias(t-h) =%.6f, iBlockedOperation = %d\n",iValve, dx0,iBlockedOperation);
					DumpString(amsr1);
					dValveSwitchTimeM117[iValve] = 0.001 * pNetSolver->dtsecs;
				}
				else if ( dx0 > 0.0 )
				{
					if  ( dx1 > 2 * dx0 )
					{
						amsr1.Format("\nComputeValveSwitchingTimes: Valve# : %d, V_bias(t-2h) =%.6f,iBlockedOperation =%d\n",iValve, dx1,iBlockedOperation);
						DumpString(amsr1);
						amsr1.Format("ComputeValveSwitchingTimes: Valve# : %d, V_bias(t-h) =%.6f,iBlockedOperation =%d\n",iValve, dx0,iBlockedOperation);
						DumpString(amsr1);
						dValveSwitchTimeM117[iValve] = dx0 * pNetSolver->dtsecs / ( dx1 - dx0 );
					}
				}
			}
		}

//------compute dValveSwitchTimeM117 for OFF -> ON switching in [t-h,t]

		if ( iValveStatusM117[iValve] == 0 )
		{
			dx0 = dValveNextTurnOn[iValve] - dRunTimeNorm;
			if ( dx0 > 0.0 )
			{
				if  ( dx0 < 1.0001 * pNetSolver->dtsecs )
				{
					dx1 = dValveCurrentM117[iValve] / dValveConductanceM117[iValve];		
					amsr1.Format("\nComputeValveSwitchingTimes: Valve# : %d, V_bias for (scheduled) ON =%.6f,iBlockedOperation=%d\n",iValve, dx1,iBlockedOperation);
					DumpString(amsr1);
					if ( dx1 > 0.0 )
					{
						dValveSwitchTimeM117[iValve]  = dx0;
						iValveM = iValve;
					}
					else
					{
//						amsr1.Format("negative polarization voltage %.4f during valve %d switch-on",dx1,iValve);
//						AfxMessageBox(amsr1);
						dValveSwitchTimeM117[iValve]  = dx0;
						iValveM = iValve;
					}
				}
			}
		}

	}

	return true;
}
//-------------------------------------------------------------------
//
//------This routine determines the state of the valve that will
//		be switched at the next time step. 
//		The valve status of all other valves does not change.
//		For the valve that will switch on, the status of the valve 
//		that is needed to close the circuit is checked and is accordingly
//		adjusted if not in proper status. This control must occur
//		during start-up only.
//		This routine also defines a valve delay operation for the valve
//		that has been switched on. 
//
//-------------------------------------------------------------------
BOOL M117::ComputeValveState_117()
{
	int iValve;
	int	mValve;
	int	iValve1;
	int	nDelay;

	nDelay = 80;

	mValve = -1;
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx0 = dValveSwitchTimeM117[iValve];
		if ( dx0 >= 0.0 )
		{
			if ( iValveStatusM117[iValve] == 0 )
			{
				iValveStatusM117[iValve] = 1;
				amsr1.Format("ComputeValveState: ***** ON Valve# : %d, runtime =%.6f\n",iValve, pNetSolver->run_time);
				DumpString(amsr1);
				nValveDelayM117[iValve] = nDelay;
				mValve = iValve;
			}
			else
			{
				iValveStatusM117[iValve] = 0;
				amsr1.Format("ComputeValveState: ***** OFF Valve# : %d, runtime =%.6f\n",iValve, pNetSolver->run_time);
				DumpString(amsr1);
			}
		}
	}

	//------make sure the right valves are on

	if ( mValve > -1 )
	{
		iValve1 = mValve - 1;
		if ( iValve1 < 0 ) iValve1 += 6;
		if ( iValveStatusM117[iValve1] == 0 )
		{
			iValveStatusM117[iValve1] = 1;
			nValveDelayM117[iValve1]  = nDelay;
		}
	}

	return true;
}
//-------------------------------------------------------------------
//
//------this routine determines the state of each valve
//		at a specified substep.
//
//-------------------------------------------------------------------
BOOL M117::DetermineValveStateAtSubStep_M117(int iSubStep)
{
	int iValve;

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx0 = double(nSubSteps) * dValveSwitchTimeM117[iValve] / pNetSolver->dtsecs;
		if ( int(dx0) == iSubStep )
		{
			if ( iValveStatusM117[iValve] == 0 )
			{
				iValveStatusM117[iValve] = 1;
			}
			else
			{
				iValveStatusM117[iValve] = 0;
			}
		}
	}

	return true;
}
//-------------------------------------------------------------------
//
//------this routine updates the valve conductances
//		based on the state of the valves.
//		the state of the valves is defined by the variable
//			iValveStatusM117
//		the updated valve conductances are in the variables: 
//			dValveConductanceM117
//
//-------------------------------------------------------------------
BOOL M117::DetermineValveConductance_M117()
{
	int iValve;

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		if ( iValveStatusM117[iValve] == 0 )
		{
			dValveConductanceM117[iValve] = dValveOffConductance;
		}
		else if ( iValveStatusM117[iValve] == 1 )
		{
			dValveConductanceM117[iValve] = dValveONConductance;
		}
		else
		{
			AfxMessageBox("undefined valve status in: DetermineValveConductance_M117");
		}
	}

	return true;
}

//-------------------------------------------------------------------
//
//------This procedure initializes the model matrices M & N 
//		for a single valve and for the capacitor model.
//		The results are stored in arrays:
//			dMmatrixSvalve, and
//			dNmatrixSvalve
//			dMmatrixCap, and
//			dNmatrixCap
//
//-------------------------------------------------------------------
BOOL M117::InitValveAndCap_MandN_117()
{

	int	i;
	int	j;

	//------initialize capacitor model

	dMmatrixCap[0][0] = 0.0;
	dMmatrixCap[0][1] = 0.0;
	dMmatrixCap[1][0] = 0.0;
	dMmatrixCap[1][1] = 0.0;

	dNmatrixCap[0][0] =   dSmoothingCapacitance;
	dNmatrixCap[0][1] = - dSmoothingCapacitance;
	dNmatrixCap[1][0] = - dSmoothingCapacitance;
	dNmatrixCap[1][1] =   dSmoothingCapacitance;

	//------initialize valve model

	for ( i=0 ; i<5 ; i++ )
	{
		for ( j=0 ; j<5 ; j++ )
		{
			dMmatrixSvalve[i][j] = 0.0;
			dNmatrixSvalve[i][j] = 0.0;
		}
	}

	dMmatrixSvalve[0][4] =  1.0;

	dMmatrixSvalve[1][4] = -1.0;

	dMmatrixSvalve[2][1] = -dSnubberConductance;
	dMmatrixSvalve[2][2] =  dSnubberConductance;

	dMmatrixSvalve[3][0] = -dGvcConductance;
	dMmatrixSvalve[3][3] =  dGvcConductance;

	dMmatrixSvalve[4][0] = -dValveOffConductance - dGvcConductance;
	dMmatrixSvalve[4][1] =  dValveOffConductance;
	dMmatrixSvalve[4][3] =  dGvcConductance;
	dMmatrixSvalve[4][4] =  1.0;


	dNmatrixSvalve[0][0] =  dSnubberCapacitance;
	dNmatrixSvalve[0][2] = -dSnubberCapacitance;
	dNmatrixSvalve[0][4] =  dCLRConductanceL;

	dNmatrixSvalve[1][0] = -dSnubberCapacitance;
	dNmatrixSvalve[1][2] =  dSnubberCapacitance;
	dNmatrixSvalve[1][4] = -dCLRConductanceL;

	dNmatrixSvalve[2][0] = -dSnubberCapacitance;
	dNmatrixSvalve[2][2] =  dSnubberCapacitance;

	dNmatrixSvalve[3][1] = -dParasiticCapacitance;
	dNmatrixSvalve[3][3] =  dParasiticCapacitance;
	dNmatrixSvalve[3][4] =  dGvcConductanceL;

	dNmatrixSvalve[4][4] =  dValveOffConductanceL + dCLRConductanceL + dGvcConductanceL;

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the model 117 matrices M & N 
//		(entire converter). The model is stored in arrays
//			dMmatrixSvalve, and
//			dNmatrixSvalve
//
//		It is assumed that the individual valve matrices
//		have been computed and stored. The entries that depend on
//		the valve's positions are computed here before they are
//		added to the whole converter M & N matrices
//
//-------------------------------------------------------------------
BOOL M117::ComputeConverter_MandN_117()
{

	int	i;
	int	i1;
	int	j;
	int	j1;
	int	iValve;

	//------initialize matrices

	for ( i=0 ; i<devsta ; i++ )
	{
		for ( j=0 ; j<devsta ; j++ )
		{
			dMmatrixConverter[i][j] = 0.0;
			dNmatrixConverter[i][j] = 0.0;
		}
	}

	//------form matrix by summing up valve contributions

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dMmatrixSvalve[4][0] = - dValveConductanceM117[iValve] - dGvcConductance;			
		dMmatrixSvalve[4][1] =   dValveConductanceM117[iValve];	
		dNmatrixSvalve[4][4] =   dValveConductanceM117[iValve] * dCLInductance + dCLRConductanceL + dGvcConductanceL;			
		for ( i=0 ; i<5 ; i++ )
		{
			i1 = iValvePointerM117[i][iValve];
			for ( j=0 ; j<5 ; j++ )
			{
				j1 = iValvePointerM117[j][iValve];
				dMmatrixConverter[i1][j1] += dMmatrixSvalve[i][j];
				dNmatrixConverter[i1][j1] += dNmatrixSvalve[i][j];
			}
		}
	}
	//------capacitor contributions
	for ( i=0 ; i<2 ; i++ )
	{
		i1 = iCapPointerM117[i];
		for ( j=0 ; j<2 ; j++ )
		{
			j1 = iCapPointerM117[j];
			dMmatrixConverter[i1][j1] += dMmatrixCap[i][j];
			dNmatrixConverter[i1][j1] += dNmatrixCap[i][j];
		}
	}
		
	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the Yeq and Peq matrices 
//		of entire converter for a time interval: dHdtsecsSubM117*2.0
//		The model is stored in arrays
//			dYeqSubStepM117, and
//			dPeqSubStepM117
//
//		It is assumed that the converter matrices M & N
//		have been computed and stored
//		dHdtsecsSubM117 value depends on whether fixed time step or
//		variable time step is used
//
//-------------------------------------------------------------------
BOOL M117::ComputeConverterSubStepYandP_117()
{

	int	i;
	int	j;

	//------initialize matrices

	for ( i=0 ; i<devsta ; i++ )
	{
		for ( j=0 ; j<devsta ; j++ )
		{
			dx1 = dMmatrixConverter[i][j];
			dx2 = dNmatrixConverter[i][j] / dHdtsecsSubM117;
			dx3 = dx2 * dBetaM117;
			dYeqSubStepM117[i][j] =   dx1 + dx2 + dx3;
			dPeqSubStepM117[i][j] = - dx1 + dx2 - dx3;
		}
	}

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the ACF model 
//		of entire converter for an initial time 
//		interval: dHdtsecsSubM117*2.0
//		It can be used in following three cases:
//		(1). Fixed time step, where nSubSteps = 1;
//		(2). Step 0 for multiple sub steps;
//		(3). Variable time step, but no switching occuring
//		The model is stored in arrays
//			yeq_real, and
//			beq_real
//
//		It is assumed that at this time Yeq and Peq matrices
//		have been computed already
//
//-------------------------------------------------------------------
BOOL M117::ComputeConverterFirstStepACF_117()
{
	int i;
	int j;

	for ( i=0 ; i<devsta ; i++ )
	{
		dx0 = xi_real[i];
		for ( j=0 ; j<devsta ; j++ )
		{
			yeq_real[i][j] = dYeqSubStepM117[i][j];
			dx0 += dPeqSubStepM117[i][j] * xa_real[j];
		}
		beq_real[i] = dx0;
	}

	return true;
}

//-------------------------------------------------------------------
//
//------This procedure computes the ACF model 
//		of entire converter for a time interval: dStep1+dStep2
//		Given the
//			ACF model for interval dStep1: yeq_real and beq_real
//			Yeq and Peq matrices for interval dStep2
//		The model is stored in arrays
//			yeq_real, and
//			beq_real
//
//-------------------------------------------------------------------
BOOL M117::ComputeConverterTwoStepACF_117(double dStep1,double dStep2)
{

/*
	int	i;
	int	j;
	int k;
	double dAlpha; 

	dAlpha = dStep2 / ( dStep1 + dStep2 );

//		amsr1.Format("     *********Inside routine: TwoStepACF****** \n");//Xi-1019
//		DumpString(amsr1);
//		amsr1.Format("dAlpha =%.6f;\n",dAlpha); //Xi-1019
//		DumpString(amsr1);	
	
	//------Compute dInvY22M117, -- inverse of dY22M117

		//------retrieve matrix dY22M117 from yeq_real

		for ( i=0 ; i<18 ; i++ )
		{
			for ( j=0 ; j<18 ; j++ )
			{
				dY22M117[i][j] = yeq_real[i+5][j+5];
			}
		}

//		DumpDoubleMatrix("Matrix: dY22M117",18,18,dY22M117); //Xi-1019

		//------triangulate matrix dY22M117

		FactorMatrix(dY22M117,18); 

		//------compute dInvY22M117 

		for ( i=0 ; i<18 ; i++ )
		{
			for ( j=0 ; j<18 ; j++ )
			{
				b_vector[j] = 0.0;
			}
			b_vector[i] = 1.0;
			ForBackSub(dY22M117,x_vector,b_vector,18);
			for ( j=0 ; j<18 ; j++ )
			{
//				dInvY22M117[i][j] = x_vector[j]; //Xi-1019
				dInvY22M117[j][i] = x_vector[j];
			}
		}

//		DumpDoubleMatrix("Matrix: dInvY22M117",18,18,dInvY22M117);//Xi-1019

	//------ retrieve dY21M117 from yeq_real

		for ( i=0 ; i<18 ; i++ )
		{
			for ( j=0 ; j<5 ; j++ )
			{
				dY21M117[i][j] = yeq_real[i+5][j];
			}
		}

//		DumpDoubleMatrix("Matrix: dY21M117",18,5,dY21M117);//Xi-1019

	//------compute dDM117 from dInvY22M117 and dY21M117

		//------initialize dDM117

		for ( i=0 ; i<devsta ; i++ )
		{
			for ( j=0 ; j<devsta ; j++ )
			{
				dDM117[i][j] = 0.0;
			}
		}
		//------initialize D(1,1) submatrix

		for ( i=0 ; i<5 ; i++ )
		{
			dDM117[i][i] = 1.0;
		}
		//------compute D(2,1) submatrix = -dInvY22M117 * dY21M117

		for ( i=0 ; i<18 ; i++ )
		{
			for ( j=0 ; j<5 ; j++ )
			{
				dx0 = 0.0;
				for ( k=0 ; k<18 ; k++ )
				{
					dx0 += dInvY22M117[i][k] * dY21M117[k][j];
				}
				dDM117[i+5][j] = - dx0;
			}
		}
	//------ compute dPeqSubStepM117 = -Y + M - 2N/hs. 
	//			Here dPeqSubStepM117 is used as scratch matrix

		for ( i=0 ; i<devsta ; i++ )
		{
			for ( j=0 ; j<devsta ; j++ )
			{
				dPeqSubStepM117[i][j] += - yeq_real[i][j];
			}
		}
	//------ compute Y' -- ACF admittance matrix

		for ( i=0 ; i<devsta ; i++ )
		{
			for ( j=0 ; j<devsta ; j++ )
			{
				dx0 = 0.0;
				for ( k=0 ; k<devsta ; k++ )
				{
					dx0 += dPeqSubStepM117[i][k] * dDM117[k][j];
				}
				yeq_real[i][j] = dYeqSubStepM117[i][j] + (1-dAlpha)*dx0;
			}
		}

//------ Added for debugging VTS// Xi-1019

//		DumpDoubleMatrix("Matrix: dDM117",23,23,dDM117);
//		DumpDoubleMatrix("Matrix: dPeqSubStepM117",23,23,dPeqSubStepM117);

	//------ compute b' -- ACF for this SubStep

		//------Compute x_vector = dInvY22M117 * b2(t-h)
		//				x_vector is scrtach array

		for ( i=0 ; i<18 ; i++ )
		{
			dx0 = 0.0;
			for ( k=0 ; k<18 ; k++ )
			{
				dx0 += dInvY22M117[i][k] * beq_real[k+5];
			}
			x_vector[i] = dx0;
		}

		//------Compute y_vector = {dAlpha*dDM117*xa_real+[0, x_vector]'}
		//				y_vector is scrtach array

		for ( i=0 ; i<devsta ; i++ )
		{
			dx0 = 0.0;
			for ( k=0 ; k<devsta ; k++ )
			{
				dx0 += dDM117[i][k] * xa_real[k];
			}
			y_vector[i] = dAlpha * dx0;
		}

		for ( i=5 ; i<devsta ; i++ )
		{
			y_vector[i] += x_vector[i-5];
		}
		//------Compute b'= - beq_real - dPeqSubStepM117 * y_vector

		for ( i=0 ; i<devsta ; i++ )
		{
			dx0 = 0.0;
			for ( k=0 ; k<devsta ; k++ )
			{
				dx0 += dPeqSubStepM117[i][k] * y_vector[k];
			}
			beq_real[i] = - beq_real[i]- dx0;
		}
*/
	return true;

}
//-------------------------------------------------------------------
//
//		this routine computes the one-step transition matrix
//		of the six-pulse converter model
//
//-------------------------------------------------------------------
BOOL M117::TTN_TimeDomainModelTransMat()
{

//	DumpDoubleMatrix("E1eq_real - Device: "+dev_title,devsta,devsta,E1eq_real);
//	DumpDoubleMatrix("E2eq_real - Device: "+dev_title,devsta,devsta,E2eq_real);

	return true;

}
//-------------------------------------------------------------------
BOOL M117::ComputeModel117Pointers()
{

	//------define valve pointers

	iValvePointerM117[0][0] =  0;
	iValvePointerM117[1][0] =  3;
	iValvePointerM117[2][0] =  8;
	iValvePointerM117[3][0] =  9;
	iValvePointerM117[4][0] = 10;

	iValvePointerM117[0][1] =  4;
	iValvePointerM117[1][1] =  2;
	iValvePointerM117[2][1] = 11;
	iValvePointerM117[3][1] = 12;
	iValvePointerM117[4][1] = 13;

	iValvePointerM117[0][2] =  1;
	iValvePointerM117[1][2] =  3;
	iValvePointerM117[2][2] = 14;
	iValvePointerM117[3][2] = 15;
	iValvePointerM117[4][2] = 16;

	iValvePointerM117[0][3] =  4;
	iValvePointerM117[1][3] =  0;
	iValvePointerM117[2][3] = 17;
	iValvePointerM117[3][3] = 18;
	iValvePointerM117[4][3] = 19;

	iValvePointerM117[0][4] =  2;
	iValvePointerM117[1][4] =  3;
	iValvePointerM117[2][4] = 20;
	iValvePointerM117[3][4] = 21;
	iValvePointerM117[4][4] = 22;

	iValvePointerM117[0][5] =  4;
	iValvePointerM117[1][5] =  1;
	iValvePointerM117[2][5] = 23;
	iValvePointerM117[3][5] = 24;
	iValvePointerM117[4][5] = 25;

	//------define capacitor pointers

	iCapPointerM117[0] = 3;
	iCapPointerM117[1] = 4;
	 
	return true;
}
//-------------------------------------------------------------------
BOOL M117::DebugReportModel117()
{
	DumpString("\nModel 117 Debug Report\n");
	DumpString("----------------------\n");

	DumpIntScalar("nValves",nValves);
	DumpIntScalar("iCControlM117",iCControlM117);
	DumpIntScalar("nSubSteps",nSubSteps);
	DumpIntScalar("nBlockedCycles",nBlockedCycles);

	DumpDoubleScalar("dHdtsecsSubM117",dHdtsecsSubM117);
	DumpDoubleScalar("dTimeDelay",dTimeDelay);
	DumpDoubleScalar("dTimePeriod",dTimePeriod);
	DumpDoubleScalar("dRunTimeNorm",dRunTimeNorm);
	DumpDoubleScalar("dRunTimeNormRef",dRunTimeNormRef);
	DumpDoubleScalar("dPosSeqZeroTime",dPosSeqZeroTime);
	DumpDoubleScalar("dValve1TurnOnTime",dValve1TurnOnTime);

	DumpDoubleScalar("dBetaM117",dBetaM117);
	DumpDoubleScalar("dBlockedTime",dBlockedTime);
	DumpDoubleScalar("dValveOffConductance",dValveOffConductance);
	DumpDoubleScalar("dValveONConductance",dValveONConductance);
	DumpDoubleScalar("dValveONConductanceL",dValveONConductanceL);
	DumpDoubleScalar("dCLRConductance",dCLRConductance);
	DumpDoubleScalar("dCLRConductanceL",dCLRConductanceL);
	DumpDoubleScalar("dSnubberConductance",dSnubberConductance);
	DumpDoubleScalar("dCpYeqReal",dCpYeqReal);
	DumpDoubleScalar("dCpPeqReal",dCpPeqReal);
	DumpDoubleScalar("dGvcConductance",dGvcConductance);
	DumpDoubleScalar("dGvcConductanceL",dGvcConductanceL);
	DumpDoubleScalar("dSmoothingCapacitance",dSmoothingCapacitance);

	DumpString("\n---------------------------------------------\n\n");
	return true;
}
//-------------------------------------------------------------------
BOOL M117::DebugPartialReportModel117()
{
	amsr1.Format("++++++++++++++AbsoluteRunTime: %.8f\n",pNetSolver->run_time);
	DumpString(amsr1);

	amsr1.Format("ReportVariable:		dRunTimeNorm: %.8f\n",dRunTimeNorm);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dRunTimeNormRef: %.8f\n",dRunTimeNormRef);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dPosSeqZeroTime: %.8f\n",dPosSeqZeroTime);
	DumpString(amsr1);

	amsr1.Format("ReportVariable:		dValve1TurnOnTime: %.8f\n",dValve1TurnOnTime);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		iValveStatusM117: %d %d %d %d %d %d\n",
		iValveStatusM117[0],iValveStatusM117[1],iValveStatusM117[2],
		iValveStatusM117[3],iValveStatusM117[4],iValveStatusM117[5]);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dValveConductanceM117: %.8f %.8f %.8f %.8f %.8f %.8f\n",
		dValveConductanceM117[0],dValveConductanceM117[1],dValveConductanceM117[2],
		dValveConductanceM117[3],dValveConductanceM117[4],dValveConductanceM117[5]);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dValveNextTurnOn: %.8f %.8f %.8f %.8f %.8f %.8f\n",
		dValveNextTurnOn[0],dValveNextTurnOn[1],dValveNextTurnOn[2],
		dValveNextTurnOn[3],dValveNextTurnOn[4],dValveNextTurnOn[5]);
	DumpString(amsr1);

	return true;
}
//-------------------------------------------------------------------