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

	//------initialize constants

	dRealPowerDC           = 0.0;
	dTimePeriod				= 1.0 / GetBaseFrequency();
	ddtsecsM117             = pNetSolver->dtsecs;        //----Time Interval 
	nBlockedCycles			= 1;
	dBlockedTime			= dTimePeriod * double(nBlockedCycles);
	
	dConverterRelativeTime		= 0.0;
	dConverterTimeRef			= 0.0;

	dVacZeroTime			= 0.0;
	dPosSeqVMagn			= 0.0;
	dRealPowerAC			= 0.0;


	dFiringAngle			= 0.0;
	dPre_FiringAngle		= 1.57;
	dTimeDelay				= 0.0;
	dDirectVoltage			= 0.0;
	dCommutationTime 		= 0.0;
	dCommutationCount		= 0.0;

	//------retrieve user defined parameters

	dValveONConductance		= dev_oparm[0];
	dValveOffConductance	= dev_oparm[1];
	dParasiticCapacitance	= dev_oparm[2];
	dSnubberCapacitance		= dev_oparm[3];
	dSnubberConductance		= 1.0 / dev_oparm[4];
	dCLConductance			= 1.0 / dev_oparm[5];
	dCLInductance			= dev_oparm[6];
	dSmoothingCapacitance	= dev_oparm[7];
	dRealPowerDC			= dev_oparm[10];
	iOperating_mode		= dev_oparm[13];

	//------allocate arrays

	TTN_AllocateLinearModelArrays();
	AllocateDataModel_117();

	//------initialize arrays

	for ( i=0 ; i<devsta ; i++ )
	{
		xi_real[i]    = 0.0;
		xa_real[i]    = 0.0;
		beq_real[i]   = 0.0; 
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
	}

	//------update converter normalized time and time reference

	if ( !UpdateConverterRunTimeNormRef_117() ) return false;

	//------define connectivity pointers

	if ( !ComputeModel117Pointers() ) return false;

	//------initialize valve and capacitor matrices M and N

	if ( !ComputeValveAndCap_Y_PandZ_117() ) return false;
	
	//------compute converter matrices Y P and Z
	
	if ( !ComputeConverter_Y_PandZ_117() ) return false;

	//------compute overall converter ACF model for initialization, use fixed time step scheme

	if ( !ComputeConverterFirstStepACF_117() ) return false;

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
	int iValve;

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


	//------retrieve dPosSeqZeroTime and dPosSeqVMagn

	dVacZeroTime = xa_real[5];
	dPosSeqVMagn	= xa_real[6];
	dRealPowerAC	= xa_real[7];

	//------prepare control action

	dNoLoadDirectVoltage = ((3.0*sqrt(2.0))/DPI)*dPosSeqVMagn;
	dDirectCurrent       = xi_real[4];
	dDirectVoltage		 = (xa_real[3]-xa_real[4]);

	//------update converter control reference

	if ( !UpdateConverterRunTimeNormRef_117() ) return false;

	//------Determine if run time is within the blocked cycles
	//		bConverterON = 1, means it is still within the blocked cycles
	//		and no need to compute valve status

	iAnySwitching = 0;
	bConverterON = 1;
	if ( (pNetSolver->run_time - dBlockedTime) > 0.0 ) bConverterON = 0;

	if ( bConverterON == 0 ) 
	{

		//---------- Operating mode is converter

		if( iOperating_mode == 1 )
		{
			//------Compute firing anlge

			if ( !CalculateFiringAngle_117() ) return false;

			//------Compute Valve ScheduleTime

			if ( !ComputeValveScheduleTimes_117() ) return false;

			//------Compute Valve Switching time for each valve

			if ( !ComputeValveSwitchingTimes_117() ) return false;
		}

		//----------- Operating mode is inverter

		else if ( iOperating_mode == 2 )
		{		
			//------Compute firing anlge

			if(!CalculateFiringAngleINV_117()) return false;

			//------Compute Valve ScheduleTime

			if ( !ComputeValveScheduleTimes_117() ) return false;

			//------Compute Valve Switching time for each valve

			if ( !ComputeValveSwitchingTimesINV_117() ) return false;

		}

		if ( !TimeUpdateValveStatus_117()) return false;

		//------compute converter matrices Y, P, and Z

		if ( iAnySwitching == 1)
		{
			if ( !ComputeConverter_Y_PandZ_117() ) return false;

			pNetSolver->bUpdateLnSystemFlag = true;
		}
	}
	
	//------compute overall converter ACF model for initialization, use fixed time step scheme

	if ( !ComputeConverterFirstStepACF_117() ) return false;
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
//	Updates the normalized run time (dConverterRelativeTime) and run time
//	reference (dConverterTimeRef)
//
//-------------------------------------------------------------------
BOOL M117::UpdateConverterRunTimeNormRef_117()
{
	dConverterRelativeTime = pNetSolver->run_time - dConverterTimeRef;

	if ( dConverterRelativeTime > dTimePeriod )
	{
		dConverterTimeRef	+= dTimePeriod;
		dConverterRelativeTime	-= dTimePeriod;
	}

	return true;
}
//----------------------------------------------------------------------------
//
//		This routine is to calculate firing angle for next step
//		firing angle = firing angle of present time +deviation of firing angle
//		the deviation can be calcuated from deviation between measured power 
//		and prosected output power.
//		the process of firing angle calculation is for retifier.
//
//-----------------------------------------------------------------------------
BOOL M117::CalculateFiringAngle_117()
{
	double dVoltageRef;
	double dDeviationOfDCVoltage;
	double dDeviationOfFiringangle;
	double dFilteredDeviationOfFiringangle;
	double dPropositionalConstant;
	double NoloadVoltageRef;

	dDeviationOfDCVoltage	= 0.0;
	dDeviationOfFiringangle = 0.0;
	dPropositionalConstant  = 0.01;

	//--------Calculation of reference voltage of DC side

	dVoltageRef = dRealPowerDC / dDirectCurrent;

	//--------Compare reference voltage with no load direct voltage
	
	if( dVoltageRef >= dNoLoadDirectVoltage || dDirectCurrent <= 0.0) dVoltageRef = dNoLoadDirectVoltage;
	//--------Calculation of deviation of output power

	dDeviationOfDCVoltage	= dVoltageRef - dDirectVoltage;

	dDeviationOfFiringangle = - dDeviationOfDCVoltage/ ( dNoLoadDirectVoltage *sin(	dPre_FiringAngle));

	//--------Calculation of firing angle deviation

	dFilteredDeviationOfFiringangle= dPropositionalConstant * dDeviationOfFiringangle;

	//--------Calculation of New firing angle

	dFiringAngle	= (	dPre_FiringAngle + dFilteredDeviationOfFiringangle);


	//------compute time delay from zero crosing of Line-Line voltage

	if( dFiringAngle <= 10.0 / 180.0 * DPI )
	{
		dFiringAngle = 10.0 / 180.0 * DPI;
	}
	else if( dFiringAngle >= 85.0 / 180.0 * DPI ) dFiringAngle = 85.0 / 180.0 * DPI;

	//DumpDoubleScalar("Conv_Firing angle: ",dPre_FiringAngle);
	//dFiringAngle=dev_oparm[ 9]/180.0*DPI;

	return true;
}

//----------------------------------------------------------------------------
//
//		This routine is to calculate firing angle for next step
//		firing angle = firing angle of present time +deviation of firing angle
//		the deviation can be calcuated from deviation between measured power 
//		and prosected output power.
//		the process of firing angle calculation has to offer two different 
//		ways for retifier and inverter.
//
//-----------------------------------------------------------------------------
BOOL M117::CalculateFiringAngleINV_117()
{	
	double dVoltageRef;
	double dDeviationOfDCVoltage;
	double dDeviationOfExtinct;
	double dFilteredDeviationOfFiringangle;
	double dPropositionalConstant;
	double NoloadVoltageRef;
	double dExtinctionAngleM117;
	double dComutationAngle;
	double dLimitationAngle;
	double dCalculationDCvolt;
	double dAngleofAdvance;

	dDeviationOfDCVoltage	= 0.0;
	dDeviationOfExtinct = 0.0;
	dPropositionalConstant  = 0.01;

	//--------Calculation of reference voltage of DC side
	dVoltageRef =  dRealPowerDC / dDirectCurrent;
		

	//--------Compare reference voltage with no load direct voltage

	if( dVoltageRef >= dNoLoadDirectVoltage || dDirectCurrent <= 0.0) dVoltageRef = dNoLoadDirectVoltage;

	//--------Calculation of deviation of output power

	dDeviationOfDCVoltage	= -dVoltageRef - dDirectVoltage;
	

	dComutationAngle = ( dCommutationTime / dTimePeriod ) * TWODPI;

	//--------Calculation of present extinction angle

	dExtinctionAngleM117	=	( DPI-dPre_FiringAngle -	dComutationAngle );
	dAngleofAdvance			=	( DPI-dPre_FiringAngle );
	dCalculationDCvolt		=	( dNoLoadDirectVoltage / 2 ) * ( cos( dExtinctionAngleM117 ) + cos( dAngleofAdvance ) ) ;
	
	dx1 = dDirectVoltage + dCalculationDCvolt;

	if(dDeviationOfDCVoltage>=-5.0)
	{
 			dDeviationOfDCVoltage	= -dCalculationDCvolt - dDirectVoltage;
	}

	dDeviationOfExtinct =   dDeviationOfDCVoltage/ ( dNoLoadDirectVoltage *sin(dExtinctionAngleM117));
	 
	//--------Calculation of firing angle deviation

	dFilteredDeviationOfFiringangle = dPropositionalConstant * dDeviationOfExtinct;

	//--------Calculation of New firing angle

	dFiringAngle		=  ( DPI - dExtinctionAngleM117 - dFilteredDeviationOfFiringangle -dComutationAngle );
	
	//if (abs( dfiringangle - dPre_FiringAngle <= 0.001) ) 
	//{
	//	dfiringangle = dPre_FiringAngle;
	//}
	//------compute time delay from zero crosing of Line-Line voltage
	
	dLimitationAngle = DPI - dComutationAngle - ( 20.0 * DPI ) / 180.0;
	if( dFiringAngle <= 95.0 / 180.0 * DPI ) dFiringAngle = 95.0 / 180.0 * DPI;
	if ( dFiringAngle >= dLimitationAngle ) 
	{
		dFiringAngle = dLimitationAngle ;
	}
		
//	DumpDoubleScalar("running time: ",pNetSolver->run_time);
//	DumpDoubleScalar("INV_Firing angle: ",dPre_FiringAngle);

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

	dTimeDelay	=( dFiringAngle / TWODPI ) * dTimePeriod;
	
	//------update firing time signals

	dValve1TurnOnTime = dVacZeroTime + dTimeDelay;

	dx0 = dTimePeriod / 6.0;
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveNextTurnOn[iValve] = dValve1TurnOnTime + dx0 * double(iValve+1);
	}

	//------adjust to have only positive values

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx1 = dValveNextTurnOn[iValve] - dConverterTimeRef;
		if ( dx1 <= 0.0 )
		{
			dValveNextTurnOn[iValve] += dTimePeriod;
		}
		if ( dx1 > dTimePeriod )
		{
			dValveNextTurnOn[iValve] -= dTimePeriod;
		}
	}
		
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx1=dValveNextTurnOn[iValve]- pNetSolver->run_time ;

		if( dx1 <= -0.000001 )
		{
			dValveNextTurnOn[iValve] += dTimePeriod;
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
	int k1;
	int k2;

//------initialize dValveSwitchTimeM117 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveSwitchTime[iValve] = -1.0;
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
				k1=iValvePointerM117[1][iValve];
				k2=iValvePointerM117[3][iValve];
				dx0 = xa_real[k2] - xa_real[k1];

				if ( dx0 < 0.0 )
				{
					dValveSwitchTime[iValve] = 0.001 * ddtsecsM117;
					if( iValve == 0 )
					{
						if( iValveStatusM117[2] ==1 ) 
						{
							dCommutationTime = pNetSolver->run_time - dCommutationCount;
						}
					}
				}
			}
		}

//------compute dValveSwitchTimeM117 for OFF -> ON switching in [t-h,t]

		if ( iValveStatusM117[iValve] == 0 )
		{
			dx0 = dValveNextTurnOn[iValve] - (pNetSolver->run_time + ddtsecsM117 ) ;

			if  ( dx0 < 0.0)
				{
					k1=iValvePointerM117[1][iValve];
					k2=iValvePointerM117[3][iValve];
					dx1 = xa_real[k2] - xa_real[k1];	


					if ( dx1 >= 0.0 )
					{
						dValveSwitchTime[iValve]  = 0.1*ddtsecsM117;
						dPre_FiringAngle    =  dFiringAngle;  
						if (dFiringAngle < dPre_FiringAngle) dPre_FiringAngle    =  dFiringAngle;
						else dPre_FiringAngle    =  dFiringAngle;
						if( iValve == 2) dCommutationCount		=	 pNetSolver->run_time + ddtsecsM117;
					}
					else
					{
						dx1=dx1;
					}

				}
		}

	}


	return true;
}
//-------------------------------------------------------------------
//
//		This routine computes the switching time
//		for each valve of the inverter mode.
//		The switching time is defined with reference 
//		the time (t-h).
//		The result is stored in variable:
//			dValveSwitchTimeM117
//
//-------------------------------------------------------------------
BOOL M117::ComputeValveSwitchingTimesINV_117()
{
	int iValve;
	int k1;
	int k2;

//------initialize dValveSwitchTimeM117 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveSwitchTime[iValve] = -1.0;
	}

//------compute dValveSwitchTimeM117 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{

//------compute dValveSwitchTimeM117 for ON -> OFF switching in [t-h,t]
		if ( iValveStatusM117[iValve] == 1 )
		{
			k1=iValvePointerM117[1][iValve];
			k2=iValvePointerM117[3][iValve];
			dx0 = xa_real[k2] - xa_real[k1];

			if ( dx0 < 0.0 )
			{
				dValveSwitchTime[iValve] = 0.001 * ddtsecsM117;
				if( iValve == 0 )
				{
					if( iValveStatusM117[2] ==1 ) 
					{
						dCommutationTime = pNetSolver->run_time - dCommutationCount;
					}
				}
			}
		}

//------compute dValveSwitchTimeM117 for OFF -> ON switching in [t-h,t]

		if ( iValveStatusM117[iValve] == 0 )
		{
			dx0 = dValveNextTurnOn[iValve] - (pNetSolver->run_time + ddtsecsM117 ) ;

			if  ( dx0 < 0.0)
				{
					k1=iValvePointerM117[1][iValve];
					k2=iValvePointerM117[3][iValve];
					dx1 = xa_real[k2] - xa_real[k1];	


					if ( dx1 >= 0.0 )
					{
						dValveSwitchTime[iValve]  = 0.1*ddtsecsM117;
						dPre_FiringAngle    =  dFiringAngle;  
						if (dFiringAngle < dPre_FiringAngle) dPre_FiringAngle    =  dFiringAngle;
						else dPre_FiringAngle    =  dFiringAngle;
						if( iValve == 2) dCommutationCount		=	 pNetSolver->run_time + ddtsecsM117;
					}
					else
					{
						dx1=dx1;
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
BOOL M117::TimeUpdateValveStatus_117()
{
	int iValve;
	int	iValve1;
	int nDelay;

	nDelay=6140;

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx0 = dValveSwitchTime[iValve];
		if ( dx0 >= 0.0 )
		{
			if ( iValveStatusM117[iValve] == 0 )
			{
				iValveStatusM117[iValve] = 1;
				iAnySwitching = 1;
				nValveDelayM117[iValve] = nDelay;
			}
			else if ( iValveStatusM117[iValve] == 1 )
			{
				iValveStatusM117[iValve] = 0;
				iAnySwitching = 1;

			}
		}
	}

//	DumpIntVector("iValveStatusM117",6,iValveStatusM117);
	
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
		dx0 = double(nSubSteps) * dValveSwitchTime[iValve] / pNetSolver->dtsecs;
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
//------This procedure initializes the model matrices M & N 
//		for a single valve and for the capacitor model.
//		The results are stored in arrays:
//			dMmatrixSvalve, and
//			dNmatrixSvalve
//			dMmatrixCap, and
//			dNmatrixCap
//
//-------------------------------------------------------------------
BOOL M117::ComputeValveAndCap_Y_PandZ_117()
{

	int	i;
	int	j;

	//------ capacitor model for Algebraic Companion Form

	//------ initialize for Yeq and Peq of single valve

	for( i=0 ; i<nCapTerm ; i++ )
	{ 
		for( j=0 ; j<nCapTerm ; j++ )
		{	
			dYmatrixCap[i][j] = 0.0;
		}
	}
	for(i=0 ; i<nCapTerm ; i++ )
	{
		for(j=0 ; j<nCapTerm ; j++)
		{
			dPmatrixCap[i][j] = 0.0;
			dZmatrixCap[i][j] = 0.0;
		}
	}

	//-------------------- set up dYmatrixCap----------------------------------------

	dYmatrixCap[0][3] =   dSmoothingCapacitance;
	dYmatrixCap[1][3] =  -dSmoothingCapacitance;
	dYmatrixCap[2][2] =	  1.0;
	dYmatrixCap[2][3] =  -ddtsecsM117 / 2.0;
	dYmatrixCap[3][0] =  -1.0;
	dYmatrixCap[3][1] =   1.0;
	dYmatrixCap[3][2] =   1.0;

	//-------------------- set up dPmatrixCap----------------------------------------

	dPmatrixCap[2][2] =	 1.0;
	dPmatrixCap[2][3] =  ddtsecsM117 / 2.0;

	//------ single valve model for Algebraic Companion Form -------------------------

	for(i = 0 ; i<nValTerm ; i++ )
	{
		for(j = 0 ; j<nValTerm ; j++ )
		{
			dYmatrixSvalveON[i][j]=0.0;
			dYmatrixSvalveOFF[i][j]=0.0;

		}
	}
	for(i = 0 ; i<nValTerm ; i++ )
	{
		for(j = 0 ; j<nValTerm ; j++ )
		{
			dPmatrixSvalveON[i][j]=0.0;
			dPmatrixSvalveOFF[i][j]=0.0;
			dZmatrixSvalve[i][j]=0.0;
		}
	}
	//-------------dYmatrixSvalve during valve ON---------------------------

	dYmatrixSvalveON[0][0] =  dCLConductance;
	dYmatrixSvalveON[0][1] = -dSnubberConductance;
	dYmatrixSvalveON[0][2] =  dSnubberConductance;
	dYmatrixSvalveON[0][3] = -dCLConductance;
	dYmatrixSvalveON[0][4] =  1.0;
	
	dYmatrixSvalveON[1][0] = -dCLConductance;
	dYmatrixSvalveON[1][1] =  dSnubberConductance;
	dYmatrixSvalveON[1][2] = -dSnubberConductance;
	dYmatrixSvalveON[1][3] =  dCLConductance;
	dYmatrixSvalveON[1][4] = -1.0;

	dYmatrixSvalveON[2][0] = -dSnubberCapacitance;
	dYmatrixSvalveON[2][1] = -( ddtsecsM117 / 2.0 ) * dSnubberConductance;
	dYmatrixSvalveON[2][2] =  ( ddtsecsM117 / 2.0 ) * dSnubberConductance + dSnubberCapacitance;

	dYmatrixSvalveON[3][0] = -( ddtsecsM117 / 2.0 ) * dCLConductance;
	dYmatrixSvalveON[3][1] = -( ddtsecsM117 / 2.0 ) * dValveONConductance - dParasiticCapacitance;
	dYmatrixSvalveON[3][3] =  ( ddtsecsM117 / 2.0 ) * dValveONConductance + ( ddtsecsM117 / 2.0 ) * dCLConductance + dParasiticCapacitance;
	dYmatrixSvalveON[3][4] = -( ddtsecsM117 / 2.0 );

	dYmatrixSvalveON[4][0] = -( ddtsecsM117 / 2.0 );
	dYmatrixSvalveON[4][3] =  ( ddtsecsM117 / 2.0 );
	dYmatrixSvalveON[4][4] =  dCLInductance;

//-------------dYmatrixSvalve during valve off---------------------------

	dYmatrixSvalveOFF[0][0] =  dCLConductance;
	dYmatrixSvalveOFF[0][1] = -dSnubberConductance;
	dYmatrixSvalveOFF[0][2] =  dSnubberConductance;
	dYmatrixSvalveOFF[0][3] = -dCLConductance;
	dYmatrixSvalveOFF[0][4] =  1.0;
	
	dYmatrixSvalveOFF[1][0] = -dCLConductance;
	dYmatrixSvalveOFF[1][1] =  dSnubberConductance;
	dYmatrixSvalveOFF[1][2] = -dSnubberConductance;
	dYmatrixSvalveOFF[1][3] =  dCLConductance;
	dYmatrixSvalveOFF[1][4] = -1.0;

	dYmatrixSvalveOFF[2][0] = -dSnubberCapacitance;
	dYmatrixSvalveOFF[2][1] = -( ddtsecsM117 / 2.0 ) * dSnubberConductance;
	dYmatrixSvalveOFF[2][2] =  ( ddtsecsM117 / 2.0 ) * dSnubberConductance + dSnubberCapacitance;

	dYmatrixSvalveOFF[3][0] = -( ddtsecsM117 / 2.0 ) * dCLConductance;
	dYmatrixSvalveOFF[3][1] = -( ddtsecsM117 / 2.0 ) * dValveOffConductance - dParasiticCapacitance;
	dYmatrixSvalveOFF[3][3] =  ( ddtsecsM117 / 2.0 ) * dValveOffConductance + ( ddtsecsM117 / 2.0 ) * dCLConductance + dParasiticCapacitance;
	dYmatrixSvalveOFF[3][4] = -( ddtsecsM117 / 2.0 );

	dYmatrixSvalveOFF[4][0] = -( ddtsecsM117 / 2.0 );
	dYmatrixSvalveOFF[4][3] =  ( ddtsecsM117 / 2.0 );
	dYmatrixSvalveOFF[4][4] =  dCLInductance;

	//-------------dPmatrixSvalve during valve on----------------------------

	dPmatrixSvalveON[2][0] = -dSnubberCapacitance;
	dPmatrixSvalveON[2][1] =  ( ddtsecsM117 / 2.0 ) * dSnubberConductance;
	dPmatrixSvalveON[2][2] = -( ddtsecsM117 / 2.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveON[3][0] =  ( ddtsecsM117 / 2.0 ) * dCLConductance;
	dPmatrixSvalveON[3][1] =  ( ddtsecsM117 / 2.0 ) * dValveONConductance - dParasiticCapacitance;	
	dPmatrixSvalveON[3][3] = -( ddtsecsM117 / 2.0 ) * dCLConductance - ( ddtsecsM117 / 2.0 ) * dValveONConductance + dParasiticCapacitance;
	dPmatrixSvalveON[3][4] =  ( ddtsecsM117 / 2.0 );

	dPmatrixSvalveON[4][0] =  ( ddtsecsM117 / 2.0 );
	dPmatrixSvalveON[4][3] = -( ddtsecsM117 / 2.0 );
	dPmatrixSvalveON[4][4] =  dCLInductance;

	//-------------dPmatrixSvalve during valve off----------------------------

	dPmatrixSvalveOFF[2][0] = -dSnubberCapacitance;
	dPmatrixSvalveOFF[2][1] =  ( ddtsecsM117 / 2.0 ) * dSnubberConductance;
	dPmatrixSvalveOFF[2][2] = -( ddtsecsM117 / 2.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveOFF[3][0] =  ( ddtsecsM117 / 2.0 ) * dCLConductance;
	dPmatrixSvalveOFF[3][1] =  ( ddtsecsM117 / 2.0 ) * dValveOffConductance - dParasiticCapacitance;	
	dPmatrixSvalveOFF[3][3] = -( ddtsecsM117 / 2.0 ) * dCLConductance - ( ddtsecsM117 / 2.0 ) * dValveOffConductance + dParasiticCapacitance;
	dPmatrixSvalveOFF[3][4] =  ( ddtsecsM117 / 2.0 );

	dPmatrixSvalveOFF[4][0] =  ( ddtsecsM117 / 2.0 );
	dPmatrixSvalveOFF[4][3] = -( ddtsecsM117 / 2.0 );
	dPmatrixSvalveOFF[4][4] =  dCLInductance;

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the model 117 matrices Y,P & Z 
//		(entire converter). The model is stored in arrays
//			dYmatrixConverter,
//			dPmatrixConverter, and							
//			dZmatrixConverter
//
//-------------------------------------------------------------------
BOOL M117::ComputeConverter_Y_PandZ_117()
{
	int k;
	int m;
	int	i;
	int	i1;
	int	j;
	int	j1;
	int	iValve;
	
//---------initialize Y,P, and Z for converter

	for ( i=0 ; i<nValTerm ; i++ )
	{
		for ( j=0 ; j<nValTerm; j++ )
		{
			dYmatrixSvalve[i][j] = 0.0;
			dPmatrixSvalve[i][j] = 0.0;
		}
	}
	for ( i=0 ; i<devsta ; i++ )
	{
		for ( j=0 ; j<devsta ; j++ )
		{
			dYmatrixConverter[i][j] = 0.0;
			dPmatrixConverter[i][j] = 0.0;
			dZmatrixConverter[i][j] = 0.0;
		}
	}
		


	//------Form matrix by summing up valve contributions

	for ( iValve = 0 ; iValve < nValves ; iValve++ )
	{
		if(iValveStatusM117[iValve]==1)
		{
			for ( i = 0 ; i < nValTerm; i++ )
			{
				for ( j = 0 ; j < nValTerm; j++ )
				{
					dYmatrixSvalve[i][j] = dYmatrixSvalveON[i][j]; 
					dPmatrixSvalve[i][j] = dPmatrixSvalveON[i][j];
				}
			}
		}
		else if ( iValveStatusM117[iValve]==0 )
		{
			for ( i = 0 ; i < nValTerm; i++ )
			{
				for ( j = 0 ; j < nValTerm; j++ )
				{
					dYmatrixSvalve[i][j] = dYmatrixSvalveOFF[i][j];
					dPmatrixSvalve[i][j] = dPmatrixSvalveOFF[i][j];
				}
			}
		}

//--------Compute Y_P_and Z for a converter by using pointer

		for ( k = 0 ; k < nValTerm ; k++ )
		{
			i1 = iValvePointerM117[k][iValve];
			for ( m = 0 ; m < nValTerm ; m++ )
			{
				j1 = iValvePointerM117[m][iValve];
				dYmatrixConverter[i1][j1] += dYmatrixSvalve[k][m];
				dPmatrixConverter[i1][j1] += dPmatrixSvalve[k][m];
			}
		}
	}
	
	//------capacitor contributions

	for ( i=0 ; i<nCapTerm ; i++ )
	{
		i1 = iCapPointerM117[i];
		for ( j=0 ; j<nCapTerm ; j++ )
		{
			j1 = iCapPointerM117[j];
			dYmatrixConverter[i1][j1] += dYmatrixCap[i][j];
			dPmatrixConverter[i1][j1] += dPmatrixCap[i][j];
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
		dx1=0.0;
		for ( j=0 ; j<devsta ; j++ )
		{	
			yeq_real[i][j] = dYmatrixConverter[i][j];
			dx1 += dPmatrixConverter[i][j] * xa_real[j];
		}
		beq_real[i]= dx1;
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
	iCapPointerM117[2] = 26;
	iCapPointerM117[3] = 27;	 
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
	DumpDoubleScalar("dConverterRelativeTime",dConverterRelativeTime);
	DumpDoubleScalar("dConverterTimeRef",dConverterTimeRef);
	DumpDoubleScalar("dPosSeqZeroTime",dVacZeroTime);
	DumpDoubleScalar("dValve1TurnOnTime",dValve1TurnOnTime);

	DumpDoubleScalar("dBetaM117",dBetaM117);
	DumpDoubleScalar("dBlockedTime",dBlockedTime);
	DumpDoubleScalar("dValveOffConductance",dValveOffConductance);
	DumpDoubleScalar("dValveONConductance",dValveONConductance);
	DumpDoubleScalar("dValveONConductanceL",dValveONConductanceL);
	DumpDoubleScalar("dCLRConductance",dCLConductance);
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

	amsr1.Format("ReportVariable:		dConverterRelativeTime: %.8f\n",dConverterRelativeTime);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dConverterTimeRef: %.8f\n",dConverterTimeRef);
	DumpString(amsr1);
	amsr1.Format("ReportVariable:		dPosSeqZeroTime: %.8f\n",dVacZeroTime);
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