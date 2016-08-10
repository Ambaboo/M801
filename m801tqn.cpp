//-------------------------------------------------------------------
//
//		File: WMASTER\DEVICES\M801\m801tdm.cpp
//
//		Six-Pulse Converter Model
//
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\agc\agc.h"
#include "\wmaster\util\DbgDump.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\Multerm.h"
#include "M801.h"
#include <fstream>

//#include "\wmaster\devices\M176\M176.h"
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
//	xa_real_tq[ 0] = va(t-h)	terminal voltages	EXTERNAL STATES
//	xa_real_tq[ 1] = vb(t-h)
//	xa_real_tq[ 2] = vc(t-h)
//	xa_real_tq[ 3] = vkathode(t-h)
//	xa_real_tq[ 4] = vanode(t-h)
//	xa_real_tq[ 5] = Zero crossing
//	xa_real_tq[ 6] = Mag.pos.voltage
//	xa_real_tq[ 7] = Power
//	
//	xa_real_tq[ 8] = vs1(t-h)	valve 1				INTERNAL STATES
//	xa_real_tq[ 9] = vp1(t-h)
//	xa_real_tq[ 10] = il1(t-h)
//	xa_real_tq[ 11] = vs2(t-h)	valve 2
//	xa_real_tq[ 12] = vp2(t-h)
//	xa_real_tq[13] = il2(t-h)
//	xa_real_tq[14] = vs3(t-h)	valve 3
//	xa_real_tq[15] = vp3(t-h)
//	xa_real_tq[16] = il3(t-h)
//	xa_real_tq[17] = vs4(t-h)	valve 4
//	xa_real_tq[18] = vp4(t-h)
//	xa_real_tq[19] = il4(t-h)
//	xa_real_tq[20] = vs5(t-h)	valve 5
//	xa_real_tq[21] = vp5(t-h)
//	xa_real_tq[22] = il5(t-h)
//	xa_real_tq[23] = vs6(t-h)	valve 6
//	xa_real_tq[24] = vp6(t-h)
//	xa_real_tq[25] = il6(t-h)
	

//	xi_real_tq[ 0] = ia(t-h)	line currents		THROUGH VARIABLES
//	xi_real_tq[ 1] = ib(t-h)
//	xi_real_tq[ 2] = ic(t-h)
//	xi_real_tq[ 3] = ikathode(t-h)
//	xi_real_tq[ 4] = ianode(t-h)
//	xi_real_tq[ 5] = ZX			identical
//	xi_real_tq[ 6] = MagVL
//	xi_real_tq[ 7] = Power

//	xi_real_tq[8] = 0.0			identical zero
//	.................			identical zero
//	xi_real_tq[21] = 0.0		identical zero
//	xi_real_tq[25] = 0.0		identical zero
//

//-------------------------------------------------------------------
#ifdef _SCAQCF_MODEL_SUPPORT
BOOL M801::TQN_TimeDomainModelInit()
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

//	if ( dev_oparm[4] < 0.001 )
//	{
//		amsr1.Format("Invalid Snubber Resistor Value (%.3f) in %s",
//		dev_oparm[4],dev_title);
//		AfxMessageBox(amsr1);
//		return false;
//	}
//
//	if ( dev_oparm[5] < 0.001 )
//	{
//		amsr1.Format("Invalid Current Limiting Resistor Value (%.3f) in %s",
//		dev_oparm[5],dev_title);
//		AfxMessageBox(amsr1);
//		return false;
//	}

	//------initialize constants

	iInverterMode			= 12;
	iNextCommutationValve	= -1;
	iNextTurnONValve		= 0;

	bConverterON			= false;
	bCommutationON			= false;
	bLogicStart				= false;

	dTimePeriod				= 1.0 / GetBaseFrequency();
	dCommutationTime 		= 5.0 / 360.0 * dTimePeriod;
	dCommutationStartTime	= 0.0;
	dConverterRelativeTime	= 0.0;
	dConverterTimeRef		= 0.0;
	dLastDebugTime			= 0.0;

	//------retrieve user defined parameters

	dValveONConductance		= dev_oparm[0];
	dValveOffConductance	= dev_oparm[1];
	dParasiticCapacitance	= dev_oparm[2];
	dSnubberCapacitance		= dev_oparm[3];
	dSnubberConductance		= dev_oparm[4];
	dCLInductance			= dev_oparm[5];
	dCLConductance			= dev_oparm[6];
	dSmoothingCapacitance	= dev_oparm[7];
	dRealPowerDC			= dev_oparm[10];
	iOperating_mode			= dev_oparm[13];			// Operating mode 1=rectifier, 2=inverter
	dLogicStartTime			= dev_oparm[16];			// seconds after logic starts
	dConverterStartTime		= dev_oparm[17];			// seconds after simulation initiation

	//------allocate arrays

	TQN_AllocateLinearModelArrays();
	AllocateDataModel_801();

	//------define connectivity pointers and ON/OFF state sequence

	if ( !TQN_ComputeModelPointers() ) return false;

	//------setup single valve QDM and SCAQCF model with status ON

	if ( !TQN_SingleValveModelStatusOnModel() ) return false;

	//------setup single valve QDM and SCAQCF model with status OFF

	if ( !TQN_SingleValveModelStatusOffModel() ) return false;

	//------setup single capacitor QDM and SCAQCF model

	if ( !TQN_SingleCapacitorModel() ) return false;

	//------prepare AQCF converter model for all modes
	
	if ( !TQN_PrepareAQCFConverterModel() ) return false;

	if ( !TQN_CopyAQCFModeltoObject(12) ) return false;

//	//------ Converter QDM model
//
//	if ( !TQN_TimeDomainQDMModelInit() ) return true;
//
//	//------ SCAQCF creation from quadratized model automatically
//
//	if (!TQN_TimeDomainQM_SCAQCFModelCreation1()) return false;


	return true;
}
/*NEW
BOOL M801::TQN_TimeDomainModelInit()
{
	int i;
	int	j;
	int iValve;

	//------debug report / input data

	ndebg = 0;
	if (ndebg == 1)
	{
		DebugDeviceGuiData();
		DebugDeviceInternalData();
		DebugDeviceOptimalData();
	}
	ndebg = 0;

	//------diagnostics

	//	if ( dev_oparm[4] < 0.001 )
	//	{
	//		amsr1.Format("Invalid Snubber Resistor Value (%.3f) in %s",
	//		dev_oparm[4],dev_title);
	//		AfxMessageBox(amsr1);
	//		return false;
	//	}
	//
	//	if ( dev_oparm[5] < 0.001 )
	//	{
	//		amsr1.Format("Invalid Current Limiting Resistor Value (%.3f) in %s",
	//		dev_oparm[5],dev_title);
	//		AfxMessageBox(amsr1);
	//		return false;
	//	}

	//------initialize constants

	iInverterMode = 3;
	//what's this
	iNextCommutationValve = -1;
	iNextTurnONValve = 0;

	bConverterON = false;
	bCommutationON = false;
	bLogicStart = false;

	dTimePeriod = 1.0 / GetBaseFrequency();
	dCommutationTime = 5.0 / 360.0 * dTimePeriod;
	dCommutationStartTime = 0.0;
	dConverterRelativeTime = 0.0;
	dConverterTimeRef = 0.0;
	dLastDebugTime = 0.0;

	//------retrieve user defined parameters

	dValveONConductance = dev_oparm[0];
	dValveOffConductance = dev_oparm[1];
	dParasiticCapacitance = dev_oparm[2];
	dSnubberCapacitance = dev_oparm[3];
	dSnubberConductance = dev_oparm[4];
	dCLInductance = dev_oparm[5];
	dCLConductance = dev_oparm[6];
	dSmoothingCapacitance = dev_oparm[7];
	dRatedDC = dev_oparm[9];
	dLogicStartTime = dev_oparm[16];			// seconds after logic starts
	dConverterStartTime = dev_oparm[17];			// seconds after simulation initiation

	//------allocate arrays

	TQN_AllocateLinearModelArrays();
	AllocateDataModel_801();

	//------define connectivity pointers and ON/OFF state sequence
 
	if (!TQN_ComputeModelPointers()) return false;

	//------setup single valve QDM and SCAQCF model with status ON

	if (!TQN_SingleValveModelStatusOnModel()) return false;

	//------setup single valve QDM and SCAQCF model with status OFF

	if (!TQN_SingleValveModelStatusOffModel()) return false;

	//------setup single capacitor QDM and SCAQCF model

	if (!TQN_SingleCapacitorModel()) return false;

	//------prepare AQCF converter model for all modes

	if (!TQN_PrepareAQCFConverterModel()) return false;

	if (!TQN_CopyAQCFModeltoObject(12)) return false;

	//	//------ Converter QDM model
	//
	//	if ( !TQN_TimeDomainQDMModelInit() ) return true;
	//
	//	//------ SCAQCF creation from quadratized model automatically
	//
	//	if (!TQN_TimeDomainQM_SCAQCFModelCreation1()) return false;


	return true;
}
*/
//-------------------------------------------------------------------
//	
//------this routine performs the time loop
//		computations for the model:
//		six-pulse converter
//
//-------------------------------------------------------------------
BOOL M801::TQN_TimeDomainModelTimeStep()
{

	//	this routine performs the time step computations
	//	for the converter model

	int i, j, k1, k2;
	int iValve;
	double dAcutalRunTime;

	// ----- update converter time reference (every second) 
	// ----- and the converter relative time

	dConverterTimeRef		= int(pNetSolver->run_time) * 1.0;
	dConverterRelativeTime	= pNetSolver->run_time - dConverterTimeRef;
	dAcutalRunTime			= pNetSolver->run_time;

	// ----- update information from DSP
	
	dFrequency				= xa_real_tq[8];
	dVacZeroTime			= xa_real_tq[5] + 60 / 360 / dFrequency; // Calculate Vac zero crossing from Vab zero crossing 
	dPosSeqVMagn			= xa_real_tq[6];
	dRealPowerAC			= xa_real_tq[7];
	
	//------compute through variable currents

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		dx0 =  -beq_real_tq[i];
		for ( j=0 ; j<devsta_tq  ; j++ )
		{
			dx0 += yeq_real_tq[i][j] * xa_real_tq[j];
		} 
		xi_real_tq[i] = dx0;
	}

	//------perform diagnostic loop

	ndebg = 0;
	if ( ndebg == 1 )
	{
		dx0 = 0.0;
	  	for ( i=devnod-3 ; i<devsta_tq/2 ; i++ )
		{
			dx0 += fabs(xi_real_tq[i]);
		}
	  	if ( dx0 > 1.0e-6 )
		{
			amsr1.Format("NonZero InternalCurrents-Model 801: %10.8f",dx0); 
			DumpString(amsr1);
			AfxMessageBox(amsr1);
			return false;
		}
	}
	ndebg = 0;

	// ---- Check whether converter is ON

	if ( bConverterON )
	{
		// ---- Converter is ON
		// ---- Check commutation is ON

		if ( bCommutationON ) 
		{
			// ---- Commutation is ON
			// ---- Compute the current through valve under commutation

			if ( iValveStatusPerMode[iNextCommutationValve][iInverterMode] == 1)
			{
				k1 = iValvePointer[1][iNextCommutationValve];
				k2 = iValvePointer[3][iNextCommutationValve];

				dCurrentNextCommuationValve = (xa_real_tq[k2] - xa_real_tq[k1]) * dValveONConductance;

				if ( dCurrentNextCommuationValve > 0.0 )
				{
					// Update past history

					if (!TQN_UpdatePastHistory()) return false;

					if ( dConverterRelativeTime>dValveTurnONTimes[iNextTurnONValve] )
					{
						// ---- Update Valve ScheduleTime
	
						iNextTurnONValve		= (iNextTurnONValve+1) % 6;
						if ( !TQN_UpdateValveScheduleTime() ) return false;
					}
				}
				else
				{
					// Commutation ends

					dCommutationTime	= dAcutalRunTime - dCommutationStartTime;
					iInverterMode++;
					bCommutationON		= false;

					// Return the AQCF converter model given iInverterMode
					// Copy it to simulation object 

					if ( !TQN_CopyAQCFModeltoObject(iInverterMode) ) return false;
					pNetSolver->bUpdateLnSystemFlag = true;

					// Update past history

					if (!TQN_UpdatePastHistory()) return false;
				}
			}
		}
		else
		{
			// ---- Commutation is NOT ON
			// ---- Check Turn ON time

			dx0 = dConverterRelativeTime - dValveTurnONTimes[iNextTurnONValve];

			if  ( dx0 > 0.0)
			{
				// ---- Valve turn on

				iInverterMode			= iNextTurnONValve * 2;
				iNextCommutationValve	= ( iNextTurnONValve+4 ) % 6;
				iNextTurnONValve		= (iNextTurnONValve+1) % 6;

				bCommutationON			= true;
				dCommutationStartTime	= dAcutalRunTime;			

				// ---- Update Valve ScheduleTime
	
				if ( !TQN_UpdateValveScheduleTime() ) return false;

				// Return the AQCF converter model given iInverterMode
				// Copy it to simulation object 

				if ( !TQN_CopyAQCFModeltoObject(iInverterMode) ) return false;
				pNetSolver->bUpdateLnSystemFlag = true;

				// Update past history

				if (!TQN_UpdatePastHistory()) return false;
			}
			else
			{
				// Update past history

				if (!TQN_UpdatePastHistory()) return false;
			}
		}
	}
	else
	{
		// ----- Converter is NOT ON
		// ----- Check Logic Start Time

		if ( dLogicStartTime < dAcutalRunTime ) 
		{
			if ( bLogicStart )
			{
				// ---- Logic part has started
				// ---- Compute firing anlge
		
				if ( !TQN_CalculateFiringAngleINV() ) return false;

				// ---- Computer Valve ScheduleTime
	
				if ( !TQN_ComputeValveScheduleTime() ) return false;

				// ---- Check Converter Start Time

				if ( dConverterStartTime < dAcutalRunTime )
				{
					// ---- Converter just Started
					// Intialize the converter model
					
					// Step 1: define iInverterMode 
					//         calculate valve under commutation 
					//         start commutation time count
					
					for ( iValve=0 ; iValve<nValves ; iValve++ )
					{
						dx0 = dConverterRelativeTime - dValveTurnONTimes[iValve] + dTimePeriod / 6.0;
					
						if  ( dx0 >= 0.0)
						{
							iInverterMode			= ((iValve+5) % 6) * 2 + 1;
							iNextTurnONValve		= iValve;
						}
					}
					
					// Step 2: Return the AQCF converter model given iInverterMode
					//         Copy it to simulation object 

					if ( iInverterMode==12 ) 
					{
						AfxMessageBox("Wrong iInverterMode in M801");
						return false;
					}

					if ( !TQN_CopyAQCFModeltoObject(iInverterMode) ) return false;
					pNetSolver->bUpdateLnSystemFlag = true;

					// Set bConverterON to be true

					bConverterON = true;
					
					// Update past history

					if (!TQN_UpdatePastHistory()) return false;
				}
				else
				{
					// ---- Converter has not Started yet
					// Update past history

					if (!TQN_UpdatePastHistory()) return false;
				}
			}
			else
			{
				// ---- Logic just started
				// Initialize the Valve Turn ON Schedule Time
				// Step 1: Compute firing anlge
				
				if ( !TQN_CalculateFiringAngleINV() ) return false;
				
				// Step 2: Set the valve turn on times

				dTimePeriod = 1 / dFrequency;
				dTimeDelay	=( dFiringAngle / TWODPI ) * dTimePeriod;
				dx0 = dVacZeroTime - dConverterTimeRef + dTimeDelay;
				dx1 = dTimePeriod / 6.0;

				for ( iValve=0 ; iValve<nValves ; iValve++ )
				{
					dValveTurnONTimes[iValve] = dx0 + dx1 * (iValve);
				}

				// Set bLogicStart to true

				bLogicStart = true;

				// ---- Update past history

				if (!TQN_UpdatePastHistory()) return false;
			}
		}
		else
		{
			// Logic has not started
			// ---- Update past history

			if (!TQN_UpdatePastHistory()) return false;
		}
	}

	// Debug Report

	if ( bLogicStart && (dAcutalRunTime < dLogicStartTime+1/dFrequency*0.5) )
	{
			TQN_DebugReportModel801();
	}

	if ( bLogicStart && (dAcutalRunTime > dConverterStartTime-1/dFrequency*0.5) && !bConverterON )
	{
			TQN_DebugReportModel801();
	}

	if ( bConverterON && (dAcutalRunTime < dConverterStartTime+1/dFrequency*0.5) )
	{
			TQN_DebugReportModel801();
	}
	

	return true;

}

//-------------------------------------------------------------------
BOOL M801::TQN_UpdateConverterRunTimeRef()
{
//	int iValve;

//	dConverterRelativeTime = pNetSolver->run_time - dConverterTimeRef;
//
//	if ( dConverterRelativeTime > dTimePeriod )
//	{
//		dConverterTimeRef	+= dTimePeriod;
//		dConverterRelativeTime	-= dTimePeriod;
//	}

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeModelPointers()
{
	//------define valve pointers

	iValvePointer[0][0] =  0;
	iValvePointer[1][0] =  3;
	iValvePointer[2][0] =  9;
	iValvePointer[3][0] = 10;
	iValvePointer[4][0] = 11;
	iValvePointer[5][0] = 27;
	iValvePointer[6][0] = 30;
	iValvePointer[7][0] = 36;
	iValvePointer[8][0] = 37;
	iValvePointer[9][0] = 38;

	iValvePointer[0][1] =  4;
	iValvePointer[1][1] =  2;
	iValvePointer[2][1] = 12;
	iValvePointer[3][1] = 13;
	iValvePointer[4][1] = 14;
	iValvePointer[5][1] = 31;
	iValvePointer[6][1] = 29;
	iValvePointer[7][1] = 39;
	iValvePointer[8][1] = 40;
	iValvePointer[9][1] = 41;

	iValvePointer[0][2] =  1;
	iValvePointer[1][2] =  3;
	iValvePointer[2][2] = 15;
	iValvePointer[3][2] = 16;
	iValvePointer[4][2] = 17;
	iValvePointer[5][2] = 28;
	iValvePointer[6][2] = 30;
	iValvePointer[7][2] = 42;
	iValvePointer[8][2] = 43;
	iValvePointer[9][2] = 44;

	iValvePointer[0][3] =  4;
	iValvePointer[1][3] =  0;
	iValvePointer[2][3] = 18;
	iValvePointer[3][3] = 19;
	iValvePointer[4][3] = 20;
	iValvePointer[5][3] = 31;
	iValvePointer[6][3] = 27;
	iValvePointer[7][3] = 45;
	iValvePointer[8][3] = 46;
	iValvePointer[9][3] = 47;

	iValvePointer[0][4] =  2;
	iValvePointer[1][4] =  3;
	iValvePointer[2][4] = 21;
	iValvePointer[3][4] = 22;
	iValvePointer[4][4] = 23;
	iValvePointer[5][4] = 29;
	iValvePointer[6][4] = 30;
	iValvePointer[7][4] = 48;
	iValvePointer[8][4] = 49;
	iValvePointer[9][4] = 50;

	iValvePointer[0][5] =  4;
	iValvePointer[1][5] =  1;
	iValvePointer[2][5] = 24;
	iValvePointer[3][5] = 25;
	iValvePointer[4][5] = 26;
	iValvePointer[5][5] = 31;
	iValvePointer[6][5] = 28;
	iValvePointer[7][5] = 51;
	iValvePointer[8][5] = 52;
	iValvePointer[9][5] = 53;

	//------define capacitor pointers

	iCapPointerM801_tq[0] = 3;
	iCapPointerM801_tq[1] = 4;
	iCapPointerM801_tq[2] = 30;
	iCapPointerM801_tq[3] = 31;

	//------define ON/OFF state sequence for each valve
	//------iValveStatusPerMode[i][j]=k; i: ith valve; j: jth mode; k: ON (1) or OFF (0)

	//------Mode 0
	iValveStatusPerMode[0][0]	= 1;
	iValveStatusPerMode[1][0]	= 0;
	iValveStatusPerMode[2][0]	= 0;
	iValveStatusPerMode[3][0]	= 0;
	iValveStatusPerMode[4][0]	= 1;
	iValveStatusPerMode[5][0]	= 1;

	//------Mode 1
	iValveStatusPerMode[0][1]	= 1;
	iValveStatusPerMode[1][1]	= 0;
	iValveStatusPerMode[2][1]	= 0;
	iValveStatusPerMode[3][1]	= 0;
	iValveStatusPerMode[4][1]	= 0;
	iValveStatusPerMode[5][1]	= 1;

	//------Mode 2
	iValveStatusPerMode[0][2]	= 1;
	iValveStatusPerMode[1][2]	= 1;
	iValveStatusPerMode[2][2]	= 0;
	iValveStatusPerMode[3][2]	= 0;
	iValveStatusPerMode[4][2]	= 0;
	iValveStatusPerMode[5][2]	= 1;

	//------Mode 3
	iValveStatusPerMode[0][3]	= 1;
	iValveStatusPerMode[1][3]	= 1;
	iValveStatusPerMode[2][3]	= 0;
	iValveStatusPerMode[3][3]	= 0;
	iValveStatusPerMode[4][3]	= 0;
	iValveStatusPerMode[5][3]	= 0;

	//------Mode 4
	iValveStatusPerMode[0][4]	= 1;
	iValveStatusPerMode[1][4]	= 1;
	iValveStatusPerMode[2][4]	= 1;
	iValveStatusPerMode[3][4]	= 0;
	iValveStatusPerMode[4][4]	= 0;
	iValveStatusPerMode[5][4]	= 0;

	//------Mode 5
	iValveStatusPerMode[0][5]	= 0;
	iValveStatusPerMode[1][5]	= 1;
	iValveStatusPerMode[2][5]	= 1;
	iValveStatusPerMode[3][5]	= 0;
	iValveStatusPerMode[4][5]	= 0;
	iValveStatusPerMode[5][5]	= 0;

	//------Mode 6
	iValveStatusPerMode[0][6]	= 0;
	iValveStatusPerMode[1][6]	= 1;
	iValveStatusPerMode[2][6]	= 1;
	iValveStatusPerMode[3][6]	= 1;
	iValveStatusPerMode[4][6]	= 0;
	iValveStatusPerMode[5][6]	= 0;

	//------Mode 7
	iValveStatusPerMode[0][7]	= 0;
	iValveStatusPerMode[1][7]	= 0;
	iValveStatusPerMode[2][7]	= 1;
	iValveStatusPerMode[3][7]	= 1;
	iValveStatusPerMode[4][7]	= 0;
	iValveStatusPerMode[5][7]	= 0;

	//------Mode 8
	iValveStatusPerMode[0][8]	= 0;
	iValveStatusPerMode[1][8]	= 0;
	iValveStatusPerMode[2][8]	= 1;
	iValveStatusPerMode[3][8]	= 1;
	iValveStatusPerMode[4][8]	= 1;
	iValveStatusPerMode[5][8]	= 0;

	//------Mode 9
	iValveStatusPerMode[0][9]	= 0;
	iValveStatusPerMode[1][9]	= 0;
	iValveStatusPerMode[2][9]	= 0;
	iValveStatusPerMode[3][9]	= 1;
	iValveStatusPerMode[4][9]	= 1;
	iValveStatusPerMode[5][9]	= 0;

	//------Mode 10
	iValveStatusPerMode[0][10]	= 0;
	iValveStatusPerMode[1][10]	= 0;
	iValveStatusPerMode[2][10]	= 0;
	iValveStatusPerMode[3][10]	= 1;
	iValveStatusPerMode[4][10]	= 1;
	iValveStatusPerMode[5][10]	= 1;

	//------Mode 11
	iValveStatusPerMode[0][11]	= 0;
	iValveStatusPerMode[1][11]	= 0;
	iValveStatusPerMode[2][11]	= 0;
	iValveStatusPerMode[3][11]	= 0;
	iValveStatusPerMode[4][11]	= 1;
	iValveStatusPerMode[5][11]	= 1;

	//------Mode 12
	iValveStatusPerMode[0][12]	= 0;
	iValveStatusPerMode[1][12]	= 0;
	iValveStatusPerMode[2][12]	= 0;
	iValveStatusPerMode[3][12]	= 0;
	iValveStatusPerMode[4][12]	= 0;
	iValveStatusPerMode[5][12]	= 0;

	return true;

}
//-------------------------------------------------------------------
BOOL M801::TQN_PrepareAQCFConverterModel()
{
	int iMode, nMode, iValve;
	int i, j, i1, j1;
	
	// this routine prepares the converter AQCF model for all modes

	// ---- define the converter AQCF model size: 12 modes

	nMode = 13;
	vConverterAQCFModel.resize(nMode);

	for ( iMode=0; iMode<nMode; iMode++ )
	{
		vConverterAQCFModel[iMode] = new M801ConverterAQCF();

		vConverterAQCFModel[iMode]->nAQCFConverterModel_Equ			= nConverterEqu;
		vConverterAQCFModel[iMode]->nAQCFConverterModel_EquOver2	= nConverterEqu / 2;
		vConverterAQCFModel[iMode]->nAQCFConverterModel_State		= nConverterState;
		
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Yeqx		= NewMatrix(nConverterEqu, nConverterState);
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Neqx		= NewMatrix(nConverterEqu, nConverterEqu / 2);
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq			= NewMatrix(nConverterEqu, nConverterEqu / 2);

		// ---- Form matrix by summing up valve contributions

		for ( iValve = 0 ; iValve < nValves ; iValve++ )
		{
			if( iValveStatusPerMode[iValve][iMode]==1 )
			{
				for ( i=0 ; i<nValTerm_tq ; i++ )
				{
					i1 = iValvePointer[i][iValve];
					for ( j=0 ; j<nValTerm_tq ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Yeqx[i1][j1] += pTDSCAQCFModel_ValveON_Yeqx[i][j];
					}

					for ( j=0 ; j<nValTerm ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Neqx[i1][j1] += pTDSCAQCFModel_ValveON_Neqx[i][j];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[i1][j1] += pTDSCAQCFModel_ValveON_Meq[i][j];
					}
				}
			}
			else if ( iValveStatusPerMode[iValve][iMode]==0 )
			{
				for ( i=0 ; i<nValTerm_tq ; i++ )
				{
					i1 = iValvePointer[i][iValve];
					for ( j=0 ; j<nValTerm_tq ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Yeqx[i1][j1] += pTDSCAQCFModel_ValveOFF_Yeqx[i][j];
					}

					for ( j=0 ; j<nValTerm ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Neqx[i1][j1] += pTDSCAQCFModel_ValveOFF_Neqx[i][j];
						vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[i1][j1] += pTDSCAQCFModel_ValveOFF_Meq[i][j];
					}
				}
			}
		}

		// ---- Capacitor contributions

		for ( i=0 ; i<nCapTerm_tq ; i++ )
		{
			i1 = iCapPointerM801_tq[i];
			for ( j=0 ; j<nCapTerm_tq ; j++ )
			{
				j1 = iCapPointerM801_tq[j];
				vConverterAQCFModel[iMode]->pAQCFConverterModel_Yeqx[i1][j1] += pTDSCAQCFModel_CAP_Yeqx[i][j];
			}

			for ( j=0 ; j<nCapTerm ; j++ )
			{
				j1 = iCapPointerM801_tq[j];
				vConverterAQCFModel[iMode]->pAQCFConverterModel_Neqx[i1][j1] += pTDSCAQCFModel_CAP_Neqx[i][j];
				vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[i1][j1] += pTDSCAQCFModel_CAP_Meq[i][j];
			}
		}

		// ---- Adjust Meq

		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[0][0] = 1.0;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[1][1] = 1.0;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[2][2] = 1.0;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[3][3] = 1.0;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[4][4] = 1.0;

		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[0+nConverterEqu / 2][0] = -0.5;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[1+nConverterEqu / 2][1] = -0.5;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[2+nConverterEqu / 2][2] = -0.5;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[3+nConverterEqu / 2][3] = -0.5;
		vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[4+nConverterEqu / 2][4] = -0.5;
	}
	
	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_CopyAQCFModeltoObject(int iMode)
{
	int i, j;


	// ---- check whether iMode is between 0 and 12
	
	// Copy pTDSAQCFModel_Yeqx to Yeq_real_tq

	for (i = 0; i < nConverterEqu; i++)
	{
		for (j = 0; j < nConverterState; j++)
		{
			yeq_real_tq[i][j] = vConverterAQCFModel[iMode]->pAQCFConverterModel_Yeqx[i][j];
		}
	}

	
	// Copy -pTDSAQCFModel_Neqx to dNeq
	for (i = 0; i < nConverterEqu; i++)
	{
		for (j = 0; j < nConverterState/2; j++)
		{
			dNeq_real_tq[i][j] = -(vConverterAQCFModel[iMode]->pAQCFConverterModel_Neqx[i][j]);
		}
	}

	// Copy -pTDSAQCFModel_Meq to dMeq
	for (i = 0; i < nConverterEqu; i++)
	{
		for (j = 0; j < nConverterEqu/2; j++)
		{
			dMeq_real_tq[i][j] = -(vConverterAQCFModel[iMode]->pAQCFConverterModel_Meq[i][j]);
		}
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_SingleValveModelStatusOnModel()
{
	int i;
	int j;

	Init_SCQDM_ValveON();

	nTDSCQDM_ValveON_Equ1 = 2;		
	nTDSCQDM_ValveON_Equ2 = 3;		
	nTDSCQDM_ValveON_Equ3 = 0;		
	nTDSCQDM_ValveON_State = 5;		
	nTDSCQDM_ValveON_Control = 0;	
	nTDSCQDM_ValveON_Feqxx = 0;		
	nTDSCQDM_ValveON_Fequu = 0;		
	nTDSCQDM_ValveON_Fequx = 0;		

	pTDSCQDM_ValveON_Yeqx1					= NewMatrix(nTDSCQDM_ValveON_Equ1, nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_Yequ1					= NewMatrix(nTDSCQDM_ValveON_Equ1, nTDSCQDM_ValveON_Control);
	pTDSCQDM_ValveON_Deqxd1					= NewMatrix(nTDSCQDM_ValveON_Equ1, nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_Ceqc1					= NewVector(nTDSCQDM_ValveON_Equ1);

	pTDSCQDM_ValveON_Yeqx2					= NewMatrix(nTDSCQDM_ValveON_Equ2, nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_Yequ2					= NewMatrix(nTDSCQDM_ValveON_Equ2, nTDSCQDM_ValveON_Control);
	pTDSCQDM_ValveON_Deqxd2					= NewMatrix(nTDSCQDM_ValveON_Equ2, nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_Ceqc2					= NewVector(nTDSCQDM_ValveON_Equ2);

	pTDSCQDM_ValveON_Yeqx3					= NewMatrix(nTDSCQDM_ValveON_Equ3, nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_Yequ3					= NewMatrix(nTDSCQDM_ValveON_Equ3, nTDSCQDM_ValveON_Control);
	pTDSCQDM_ValveON_Ceqc3					= NewVector(nTDSCQDM_ValveON_Equ3);

	pTDSCQDM_ValveON_Feqxx3					= new SP_CUBIX[nTDSCQDM_ValveON_Feqxx];
	pTDSCQDM_ValveON_Fequu3					= new SP_CUBIX[nTDSCQDM_ValveON_Fequu];
	pTDSCQDM_ValveON_Fequx3					= new SP_CUBIX[nTDSCQDM_ValveON_Fequx];

	pTDSCQDM_ValveON_StateNormFactor		= NewVector(nTDSCQDM_ValveON_State);
	pTDSCQDM_ValveON_ThroughNormFactor1		= NewVector(nTDSCQDM_ValveON_Equ1);
	pTDSCQDM_ValveON_ThroughNormFactor2		= NewVector(nTDSCQDM_ValveON_Equ2);
	pTDSCQDM_ValveON_ThroughNormFactor3		= NewVector(nTDSCQDM_ValveON_Equ3);
	pTDSCQDM_ValveON_ControlNormFactor		= NewVector(nTDSCQDM_ValveON_Control);

	pTDSCQDM_ValveON_NodeName				= new CString[nTDSCQDM_ValveON_Equ1];

	//------ Quadratized Model
	//----- Equation Set 1

	pTDSCQDM_ValveON_Yeqx1[0][0]	=  dCLConductance;
	pTDSCQDM_ValveON_Yeqx1[0][1]	= -dSnubberConductance;
	pTDSCQDM_ValveON_Yeqx1[0][2]	=  dSnubberConductance;
	pTDSCQDM_ValveON_Yeqx1[0][3]	= -dCLConductance;
	pTDSCQDM_ValveON_Yeqx1[0][4]	=  1.0;

	pTDSCQDM_ValveON_Yeqx1[1][0]	= -dCLConductance;
	pTDSCQDM_ValveON_Yeqx1[1][1]	=  dSnubberConductance;
	pTDSCQDM_ValveON_Yeqx1[1][2]	= -dSnubberConductance;
	pTDSCQDM_ValveON_Yeqx1[1][3]	=  dCLConductance;
	pTDSCQDM_ValveON_Yeqx1[1][4]	= -1.0;

	//----- Equation Set 2

	pTDSCQDM_ValveON_Yeqx2[0][1]	= -dSnubberConductance;
	pTDSCQDM_ValveON_Yeqx2[0][2]	= dSnubberConductance;
	pTDSCQDM_ValveON_Deqxd2[0][0]	= -dSnubberCapacitance;
	pTDSCQDM_ValveON_Deqxd2[0][2]	= dSnubberCapacitance;

	pTDSCQDM_ValveON_Yeqx2[1][1]	= -dValveONConductance;
	pTDSCQDM_ValveON_Yeqx2[1][3]	= dValveONConductance + dCLConductance;
	pTDSCQDM_ValveON_Yeqx2[1][0]	= -dCLConductance;
	pTDSCQDM_ValveON_Yeqx2[1][4]	= -1.0;
	pTDSCQDM_ValveON_Deqxd2[1][1]	= -dParasiticCapacitance;
	pTDSCQDM_ValveON_Deqxd2[1][3]	= dParasiticCapacitance;

	pTDSCQDM_ValveON_Yeqx2[2][0]	= -1.0;
	pTDSCQDM_ValveON_Yeqx2[2][3]	= 1.0;
	pTDSCQDM_ValveON_Deqxd2[2][4]	= dCLInductance;

	//------ SCAQCF creation from quadratized model automatically

	if (!TQN_TimeDomainQM_SCAQCFModel_M801(0)) return false;

	return true;
}
//----------------------------------------------------------------------------
BOOL M801::TQN_SingleValveModelStatusOffModel()
{
	int i;
	int j;

	Init_SCQDM_ValveOFF();

	nTDSCQDM_ValveOFF_Equ1 = 2;		
	nTDSCQDM_ValveOFF_Equ2 = 3;		
	nTDSCQDM_ValveOFF_Equ3 = 0;		
	nTDSCQDM_ValveOFF_State = 5;		
	nTDSCQDM_ValveOFF_Control = 0;	
	nTDSCQDM_ValveOFF_Feqxx = 0;		
	nTDSCQDM_ValveOFF_Fequu = 0;		
	nTDSCQDM_ValveOFF_Fequx = 0;		

	pTDSCQDM_ValveOFF_Yeqx1					= NewMatrix(nTDSCQDM_ValveOFF_Equ1, nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_Yequ1					= NewMatrix(nTDSCQDM_ValveOFF_Equ1, nTDSCQDM_ValveOFF_Control);
	pTDSCQDM_ValveOFF_Deqxd1				= NewMatrix(nTDSCQDM_ValveOFF_Equ1, nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_Ceqc1					= NewVector(nTDSCQDM_ValveOFF_Equ1);

	pTDSCQDM_ValveOFF_Yeqx2					= NewMatrix(nTDSCQDM_ValveOFF_Equ2, nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_Yequ2					= NewMatrix(nTDSCQDM_ValveOFF_Equ2, nTDSCQDM_ValveOFF_Control);
	pTDSCQDM_ValveOFF_Deqxd2				= NewMatrix(nTDSCQDM_ValveOFF_Equ2, nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_Ceqc2					= NewVector(nTDSCQDM_ValveOFF_Equ2);

	pTDSCQDM_ValveOFF_Yeqx3					= NewMatrix(nTDSCQDM_ValveOFF_Equ3, nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_Yequ3					= NewMatrix(nTDSCQDM_ValveOFF_Equ3, nTDSCQDM_ValveOFF_Control);
	pTDSCQDM_ValveOFF_Ceqc3					= NewVector(nTDSCQDM_ValveOFF_Equ3);

	pTDSCQDM_ValveOFF_Feqxx3				= new SP_CUBIX[nTDSCQDM_ValveOFF_Feqxx];
	pTDSCQDM_ValveOFF_Fequu3				= new SP_CUBIX[nTDSCQDM_ValveOFF_Fequu];
	pTDSCQDM_ValveOFF_Fequx3				= new SP_CUBIX[nTDSCQDM_ValveOFF_Fequx];

	pTDSCQDM_ValveOFF_StateNormFactor		= NewVector(nTDSCQDM_ValveOFF_State);
	pTDSCQDM_ValveOFF_ThroughNormFactor1	= NewVector(nTDSCQDM_ValveOFF_Equ1);
	pTDSCQDM_ValveOFF_ThroughNormFactor2	= NewVector(nTDSCQDM_ValveOFF_Equ2);
	pTDSCQDM_ValveOFF_ThroughNormFactor3	= NewVector(nTDSCQDM_ValveOFF_Equ3);
	pTDSCQDM_ValveOFF_ControlNormFactor		= NewVector(nTDSCQDM_ValveOFF_Control);

	pTDSCQDM_ValveOFF_NodeName				= new CString[nTDSCQDM_ValveOFF_Equ1];

	//------ Quadratized Model
	//----- Equation Set 1

	pTDSCQDM_ValveOFF_Yeqx1[0][0]	=  dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx1[0][1]	= -dSnubberConductance;
	pTDSCQDM_ValveOFF_Yeqx1[0][2]	=  dSnubberConductance;
	pTDSCQDM_ValveOFF_Yeqx1[0][3]	= -dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx1[0][4]	=  1.0;

	pTDSCQDM_ValveOFF_Yeqx1[1][0]	= -dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx1[1][1]	=  dSnubberConductance;
	pTDSCQDM_ValveOFF_Yeqx1[1][2]	= -dSnubberConductance;
	pTDSCQDM_ValveOFF_Yeqx1[1][3]	=  dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx1[1][4]	= -1.0;

	//----- Equation Set 2

	pTDSCQDM_ValveOFF_Yeqx2[0][1]	= -dSnubberConductance;
	pTDSCQDM_ValveOFF_Yeqx2[0][2]	= dSnubberConductance;
	pTDSCQDM_ValveOFF_Deqxd2[0][0]	= -dSnubberCapacitance;
	pTDSCQDM_ValveOFF_Deqxd2[0][2]	= dSnubberCapacitance;

	pTDSCQDM_ValveOFF_Yeqx2[1][0]	= -dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx2[1][1]	= -dValveOffConductance;
	pTDSCQDM_ValveOFF_Yeqx2[1][3]	= dValveOffConductance + dCLConductance;
	pTDSCQDM_ValveOFF_Yeqx2[1][4]	= -1.0;
	pTDSCQDM_ValveOFF_Deqxd2[1][1]	= -dParasiticCapacitance;
	pTDSCQDM_ValveOFF_Deqxd2[1][3]	= dParasiticCapacitance;

	pTDSCQDM_ValveOFF_Yeqx2[2][0]	= -1.0;
	pTDSCQDM_ValveOFF_Yeqx2[2][3]	= 1.0;
	pTDSCQDM_ValveOFF_Deqxd2[2][4]	= dCLInductance;

	//------ SCAQCF creation from quadratized model automatically

	if (!TQN_TimeDomainQM_SCAQCFModel_M801(1)) return false;

	return true;
}
//----------------------------------------------------------------------------
BOOL M801::TQN_SingleCapacitorModel()
{
	int i;
	int j;

	Init_SCQDM_CAP();

	nTDSCQDM_CAP_Equ1 = 2;		
	nTDSCQDM_CAP_Equ2 = 0;		
	nTDSCQDM_CAP_Equ3 = 0;		
	nTDSCQDM_CAP_State = 2;		
	nTDSCQDM_CAP_Control = 0;	
	nTDSCQDM_CAP_Feqxx = 0;		
	nTDSCQDM_CAP_Fequu = 0;		
	nTDSCQDM_CAP_Fequx = 0;		

	pTDSCQDM_CAP_Yeqx1					= NewMatrix(nTDSCQDM_CAP_Equ1, nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_Yequ1					= NewMatrix(nTDSCQDM_CAP_Equ1, nTDSCQDM_CAP_Control);
	pTDSCQDM_CAP_Deqxd1					= NewMatrix(nTDSCQDM_CAP_Equ1, nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_Ceqc1					= NewVector(nTDSCQDM_CAP_Equ1);

	pTDSCQDM_CAP_Yeqx2					= NewMatrix(nTDSCQDM_CAP_Equ2, nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_Yequ2					= NewMatrix(nTDSCQDM_CAP_Equ2, nTDSCQDM_CAP_Control);
	pTDSCQDM_CAP_Deqxd2					= NewMatrix(nTDSCQDM_CAP_Equ2, nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_Ceqc2					= NewVector(nTDSCQDM_CAP_Equ2);

	pTDSCQDM_CAP_Yeqx3					= NewMatrix(nTDSCQDM_CAP_Equ3, nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_Yequ3					= NewMatrix(nTDSCQDM_CAP_Equ3, nTDSCQDM_CAP_Control);
	pTDSCQDM_CAP_Ceqc3					= NewVector(nTDSCQDM_CAP_Equ3);

	pTDSCQDM_CAP_Feqxx3					= new SP_CUBIX[nTDSCQDM_CAP_Feqxx];
	pTDSCQDM_CAP_Fequu3					= new SP_CUBIX[nTDSCQDM_CAP_Fequu];
	pTDSCQDM_CAP_Fequx3					= new SP_CUBIX[nTDSCQDM_CAP_Fequx];

	pTDSCQDM_CAP_StateNormFactor		= NewVector(nTDSCQDM_CAP_State);
	pTDSCQDM_CAP_ThroughNormFactor1		= NewVector(nTDSCQDM_CAP_Equ1);
	pTDSCQDM_CAP_ThroughNormFactor2		= NewVector(nTDSCQDM_CAP_Equ2);
	pTDSCQDM_CAP_ThroughNormFactor3		= NewVector(nTDSCQDM_CAP_Equ3);
	pTDSCQDM_CAP_ControlNormFactor		= NewVector(nTDSCQDM_CAP_Control);

	pTDSCQDM_CAP_NodeName				= new CString[nTDSCQDM_CAP_Equ1];

	//----- Quadratized Model
	//----- Equation Set 1

	pTDSCQDM_CAP_Deqxd1[0][0]			=  dSmoothingCapacitance;
	pTDSCQDM_CAP_Deqxd1[0][1]			=  -dSmoothingCapacitance;
	pTDSCQDM_CAP_Deqxd1[1][0]			=  -dSmoothingCapacitance;
	pTDSCQDM_CAP_Deqxd1[1][1]			=  dSmoothingCapacitance;

	//------ SCAQCF creation from quadratized model automatically

	if (!TQN_TimeDomainQM_SCAQCFModel_M801(2)) return false;

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
BOOL M801::TQN_CalculateFiringAngle_801()
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

	//DumpDoubleScalar("Conv_Firing angle: ",Pre_FiringAngle);
	//dFiringAngle=dev_oparm[ 9]/180.0*DPI;
	return true;
}

//----------------------------------------------------------------------------
//
//		This routine is to calculate firing angle for next step
//		firing angle = firing angle of present time +deviation of firing angle
//		the deviation can be calcuated from deviation between measured power 
//		and prospected output power.
//		the process of firing angle calculation has to offer two different 
//		ways for retifier and inverter.
//
//-----------------------------------------------------------------------------
BOOL M801::TQN_CalculateFiringAngleINV()
{	
	double dVoltageRef;
	double dDeviationOfDCVoltage;
	double dDeviationOfExtinct;
	double dFilteredDeviationOfExtinctionAngle;
	double dPropositionalConstant;
	double NoloadVoltageRef;
	double dExtinctionAngle;
	double dCommutationAngle;
	double dLimitationAngle;
	double dCalculationDCvolt;
	double dAngleofAdvance;

	dPropositionalConstant  = 0.01;

	//------prepare control action
	
	dAvgDirectVoltage	 = ((3.0*sqrt(2.0)*sqrt(3.0))/DPI)*dPosSeqVMagn;
	dDirectCurrent       = xi_real_tq[3];
	dDirectVoltage		 = (xa_real_tq[4]-xa_real_tq[3]);

	//--------Calculation of reference voltage of DC side

	dVoltageRef =  dRealPowerDC / dDirectCurrent;
		
	if( dVoltageRef >= dAvgDirectVoltage || dDirectCurrent <= 0.0) dVoltageRef = dAvgDirectVoltage;

	//--------Calculation of deviation of voltage Delta_Vi = Vdi_ref - Vdi

	dDeviationOfDCVoltage = dVoltageRef - dDirectVoltage;
	
	//--------Calculation of present extinction angle

	dx0 = dDirectVoltage/dAvgDirectVoltage;

	if (dx0 > 1.0) dx0 = 1.0;

	dExtinctionAngle =	acos( dx0 );

	//--------Calculation of deviation extinction angle

	if ( dExtinctionAngle== 0.0 ) 
	{
		dDeviationOfExtinct = 0.0;
	}
	else
	{
		dDeviationOfExtinct = dDeviationOfDCVoltage/ ( -dAvgDirectVoltage *sin(dExtinctionAngle));
	}

	//--------Calculation of firing angle deviation

	dFilteredDeviationOfExtinctionAngle = dPropositionalConstant * dDeviationOfExtinct;

	//--------Calculation of commutation angle

	dCommutationAngle = ( dCommutationTime / dTimePeriod ) * TWODPI;

	//--------Calculation of firing angle

	dFiringAngle = ( DPI - dExtinctionAngle - dFilteredDeviationOfExtinctionAngle -dCommutationAngle );

	dLimitationAngle = DPI - dCommutationAngle - ( 20.0 * DPI ) / 180.0;
	if( dFiringAngle <= 90.0 / 180.0 * DPI )	dFiringAngle = 90.0 / 180.0 * DPI;
	if ( dFiringAngle >= dLimitationAngle )		dFiringAngle = dLimitationAngle ;
		
	return true;
}

//-------------------------------------------------------------------
//
//		This routine computes the scheduled ON time
//			dValve1TurnOnTime for the first valve.
//		in addition, it computes the scheduled ON time
//			dValveTurnONTimes[] for all the valves
//
//-------------------------------------------------------------------
BOOL M801::TQN_UpdateValveScheduleTime()
{
	//	this routine computes the valve turn on times
	//	the valve turn on times are in Converter Relative time

	int iValve;

	dTimePeriod = 1 / dFrequency;

	dTimeDelay	=( dFiringAngle / TWODPI ) * dTimePeriod;
	
	//------ Set the valve turn on times for a period of one cycle after run_time

	dx0 = dVacZeroTime - dConverterTimeRef + dTimeDelay;
	dx1 = dTimePeriod / 6.0;

	for ( iValve=iNextTurnONValve ; iValve < nValves ; iValve++ )
	{
		dValveTurnONTimesOld[iValve] = dValveTurnONTimes[iValve];
		dValveTurnONTimes[iValve] = dx0 + dx1 * (iValve);
	}

	for ( iValve=0 ; iValve < iNextTurnONValve ; iValve++ )
	{
		dValveTurnONTimesOld[iValve] = dValveTurnONTimes[iValve];
		dValveTurnONTimes[iValve] = dx0 + dx1 * (iValve) + dTimePeriod;
	}

	//	adjust turn on times so that the next valve turn on time remains same

	if ( dValveTurnONTimes[iNextTurnONValve] - dConverterRelativeTime < 2*(pNetSolver->dtsecs) )
	{
		dx0 = dValveTurnONTimesOld[iNextTurnONValve] - dValveTurnONTimes[iNextTurnONValve];
		
		for ( iValve=0 ; iValve < nValves ; iValve++ )
		{
			dValveTurnONTimes[iValve] += dx0;
		}
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeValveScheduleTime()
{
	//	this routine computes the valve turn on times
	//	the valve turn on times are in Converter Relative time

	int iValve;

	dTimePeriod = 1 / dFrequency;

	dTimeDelay	=( dFiringAngle / TWODPI ) * dTimePeriod;
	
	//------ Set the valve turn on times for a period of one cycle after run_time

	dx0 = dVacZeroTime - dConverterTimeRef + dTimeDelay;
	dx1 = dTimePeriod / 6.0;

	for ( iValve=0 ; iValve < nValves ; iValve++ )
	{
		dValveTurnONTimes[iValve] = dx0 + dx1 * (iValve);
	}

	for ( iValve=0 ; iValve < nValves ; iValve++ )
	{
		dx0 = dValveTurnONTimes[iValve] - dConverterRelativeTime;
		if ( dx0 < 0.0 ) dValveTurnONTimes[iValve] += dTimePeriod;
	}

	return true;
}
//-------------------------------------------------------------------
//
//		This routine updates the Valve Status.
//		if Valve state is ON,  iValveStatusPerModeM801 = 1
//		if Valve state is OFF, iValveStatusPerModeM801 = 0
//
//		A valve is turned off if the voltage bias goes negative
//		A valve is turned on with a equi-distant control
//
//-------------------------------------------------------------------
BOOL M801::TQN_TimeUpdateValveStatus_801()
{
	int iValve;
	int	iValve1;

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx0 = dValveSwitchTime[iValve];
		if ( dx0 >= 0.0 )
		{
			if ( iValveStatusM801[iValve] == 0 )
			{
				iValveStatusM801[iValve] = 1;
				iAnySwitching = 1;
			}
			else if ( iValveStatusM801[iValve] == 1 )
			{
				iValveStatusM801[iValve] = 0;
				iAnySwitching = 1;

			}
		}
	}

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the model 801 matrices Y,P & Z 
//		(entire converter). The model is stored in arrays
//			dYmatrixConverter,
//			dPmatrixConverter, and							
//			dZmatrixConverter
//
//-------------------------------------------------------------------
BOOL M801::TQN_TimeDomainQM_SCAQCFModel_M801(int iModel)
{
	int i, j, k;
	int iEqu, iEqu1, irow, icolumn, irow1, icolumn1;
	double dValue, dh;

	dh = pNetSolver->dtsecs;

	double	d4Overh		= 4.0 / dh;
	double	d8Overh		= 8.0 / dh;
	double	d0p5Overh	= 0.5 / dh;
	double	d2Overh		= 2.0 / dh;
	double	d2p5Overh	= 2.5 / dh;

	switch (iModel)
	{
	case 0:
		nTDSCAQCFModel_ValveON_Equ					= (nTDSCQDM_ValveON_Equ1 + nTDSCQDM_ValveON_Equ2 + nTDSCQDM_ValveON_Equ3) * 2;
		nTDSCAQCFModel_ValveON_EquOver2				= nTDSCQDM_ValveON_Equ1 + nTDSCQDM_ValveON_Equ2 + nTDSCQDM_ValveON_Equ3;
		nTDSCAQCFModel_ValveON_State				= nTDSCQDM_ValveON_State * 2;
		nTDSCAQCFModel_ValveON_Control				= nTDSCQDM_ValveON_Control * 2;
		nTDSCAQCFModel_ValveON_Feqxx				= nTDSCQDM_ValveON_Feqxx * 2;
		nTDSCAQCFModel_ValveON_Fequu				= nTDSCQDM_ValveON_Fequu * 2;
		nTDSCAQCFModel_ValveON_Fequx				= nTDSCQDM_ValveON_Fequx * 2;

		// ---- Allocate arrays and vectors of SCAQCF object

		pTDSCAQCFModel_ValveON_Yeqx					= NewMatrix(nTDSCAQCFModel_ValveON_Equ, nTDSCAQCFModel_ValveON_State);
		pTDSCAQCFModel_ValveON_Yequ					= NewMatrix(nTDSCAQCFModel_ValveON_Equ, nTDSCAQCFModel_ValveON_Control);
		pTDSCAQCFModel_ValveON_Feqxx				= new SP_CUBIX[nTDSCAQCFModel_ValveON_Feqxx];
		pTDSCAQCFModel_ValveON_Fequu				= new SP_CUBIX[nTDSCAQCFModel_ValveON_Fequu];
		pTDSCAQCFModel_ValveON_Fequx				= new SP_CUBIX[nTDSCAQCFModel_ValveON_Fequx];
		pTDSCAQCFModel_ValveON_Neqx					= NewMatrix(nTDSCAQCFModel_ValveON_Equ, nTDSCAQCFModel_ValveON_State/2);
		pTDSCAQCFModel_ValveON_Nequ					= NewMatrix(nTDSCAQCFModel_ValveON_Equ, nTDSCAQCFModel_ValveON_Control/2);
		pTDSCAQCFModel_ValveON_Meq					= NewMatrix(nTDSCAQCFModel_ValveON_Equ, nTDSCAQCFModel_ValveON_State / 2);
		pTDSCAQCFModel_ValveON_Keq					= NewVector(nTDSCAQCFModel_ValveON_Equ);

		pTDSCAQCFModel_ValveON_StateNormFactor		= NewVector(nTDSCAQCFModel_ValveON_State);
		pTDSCAQCFModel_ValveON_ThroughNormFactor	= NewVector(nTDSCAQCFModel_ValveON_Equ);
		pTDSCAQCFModel_ValveON_ControlNormFactor	= NewVector(nTDSCAQCFModel_ValveON_Control);
		pTDSCAQCFModel_ValveON_NodeName				= new CString[nTDSCQDM_ValveON_Equ1 * 2];

		// ----- Create SCAQCF device model from quadratized device model

		// ----- SCAQCF State Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveON_State; i++)
		{
			pTDSCAQCFModel_ValveON_StateNormFactor[i] = pTDSCQDM_ValveON_StateNormFactor[i];
			pTDSCAQCFModel_ValveON_StateNormFactor[i + nTDSCQDM_ValveON_State] = pTDSCQDM_ValveON_StateNormFactor[i];
		}

		// ----- SCAQCF Control Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveON_Control; i++)
		{
			pTDSCAQCFModel_ValveON_ControlNormFactor[i] = pTDSCQDM_ValveON_ControlNormFactor[i];
			pTDSCAQCFModel_ValveON_ControlNormFactor[i + nTDSCQDM_ValveON_Control] = pTDSCQDM_ValveON_ControlNormFactor[i];
		}

		// ----- SCAQCF Through Variables Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i] = pTDSCQDM_ValveON_ThroughNormFactor1[i];
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i + nTDSCQDM_ValveON_State] = pTDSCQDM_ValveON_ThroughNormFactor1[i];
		}

		for (i = 0; i < nTDSCQDM_ValveON_Equ2; i++)
		{
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i+nTDSCQDM_ValveON_Equ1] = pTDSCQDM_ValveON_ThroughNormFactor2[i];
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i+nTDSCQDM_ValveON_Equ1 + nTDSCQDM_ValveON_State] = pTDSCQDM_ValveON_ThroughNormFactor2[i];
		}

		for (i = 0; i < nTDSCQDM_ValveON_Equ3; i++)
		{
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i+nTDSCQDM_ValveON_Equ1+nTDSCQDM_ValveON_Equ2] = pTDSCQDM_ValveON_ThroughNormFactor3[i];
			pTDSCAQCFModel_ValveON_ThroughNormFactor[i+nTDSCQDM_ValveON_Equ1+nTDSCQDM_ValveON_Equ2+nTDSCQDM_ValveON_State] = pTDSCQDM_ValveON_ThroughNormFactor3[i];
		}


		//----contributions from Yeqx1 to Yeqx
		//----contributions from Yeqx1 to Neqx

		for ( i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			for ( j = 0; j < nTDSCQDM_ValveON_State; j++)
			{
				dValue = pTDSCQDM_ValveON_Yeqx1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] -= dValue;

					// Contributions to Neqx time tm part
					irow = i + nTDSCAQCFModel_ValveON_EquOver2;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yeqx time tm part
					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Yequ1 to Yequ
		//----contributions from Yequ1 to Nequ

		for (i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveON_Control; j++)
			{
				dValue = pTDSCQDM_ValveON_Yequ1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yequ and Nequ time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yequ[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveON_Nequ[irow][icolumn] -= dValue;

					// Contributions to Nequ time tm part
					irow = i + nTDSCAQCFModel_ValveON_EquOver2;
					pTDSCAQCFModel_ValveON_Nequ[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yequ time tm part
					icolumn = j + nTDSCQDM_ValveON_Control;
					pTDSCAQCFModel_ValveON_Yequ[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Deqxd1 to Yeqx
		//----contributions from Deqxd1 to Neqx

		for (i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveON_State; j++)
			{
				dValue = pTDSCQDM_ValveON_Deqxd1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d4Overh * dValue;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] += d4Overh * dValue;

					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] -= d8Overh * dValue;

					// Contributions to Yeqx time tm part
					irow = i + nTDSCAQCFModel_ValveON_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d0p5Overh * dValue;

					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d2Overh * dValue;

					// Contributions to Neqx time tm part
					icolumn = j;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] -= d2p5Overh * dValue;
				}
			}
		}

		//----contributions from Ceq1 to Keq

		for (i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			dValue = pTDSCQDM_ValveON_Ceqc1[i];
			if (dValue != 0.0)
			{
				irow = i + nTDSCAQCFModel_ValveON_EquOver2;
				pTDSCAQCFModel_ValveON_Keq[irow] = 1.5 * dValue;
			}
		}

		//----contributions from Yeqx2 to Yeqx
		//----contributions from Yeqx2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveON_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveON_State; j++)
			{
				dValue = pTDSCQDM_ValveON_Yeqx2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i + nTDSCQDM_ValveON_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] -= dValue;

					// Contributions to Neqx time tm part
					irow = i + nTDSCQDM_ValveON_Equ1 + nTDSCAQCFModel_ValveON_EquOver2;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yeqx time tm part
					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Yequ2 to Yeqx
		//----contributions from Yequ2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveON_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveON_Control; j++)
			{
				dValue = pTDSCQDM_ValveON_Yequ2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yequ and Nequ time t part
					irow = i + nTDSCQDM_ValveON_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yequ[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveON_Nequ[irow][icolumn] -= dValue;

					// Contributions to Nequ time tm part
					irow = i + nTDSCQDM_ValveON_Equ1 + nTDSCAQCFModel_ValveON_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Nequ[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yequ time tm part
					icolumn = j + nTDSCQDM_ValveON_Control;
					pTDSCAQCFModel_ValveON_Yequ[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Deqxd2 to Yeqx
		//----contributions from Deqxd2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveON_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveON_State; j++)
			{
				dValue = pTDSCQDM_ValveON_Deqxd2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i + nTDSCQDM_ValveON_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d4Overh * dValue;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] += d4Overh * dValue;

					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] -= d8Overh * dValue;

					// Contributions to Yeqx time tm part
					irow = i + nTDSCQDM_ValveON_Equ1 + nTDSCAQCFModel_ValveON_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d0p5Overh * dValue;

					icolumn = j + nTDSCQDM_ValveON_State;
					pTDSCAQCFModel_ValveON_Yeqx[irow][icolumn] += d2Overh * dValue;

					// Contributions to Neqx time tm part
					icolumn = j;
					pTDSCAQCFModel_ValveON_Neqx[irow][icolumn] -= d2p5Overh * dValue;
				}
			}
		}

		//----contributions from Ceq2 to Keq

		for (i = 0; i < nTDSCQDM_ValveON_Equ2; i++)
		{
			dValue = pTDSCQDM_ValveON_Ceqc2[i];
			if (dValue != 0.0)
			{
				irow = i + nTDSCQDM_ValveON_Equ1 + nTDSCAQCFModel_ValveON_EquOver2;
				pTDSCAQCFModel_ValveON_Keq[irow] = 1.5 * dValue;
			}
		}

		//----update of Meqx

		for (i = 0; i < nTDSCQDM_ValveON_Equ1; i++)
		{
			pTDSCAQCFModel_ValveON_Meq[i][i] = 1.0;
			pTDSCAQCFModel_ValveON_Meq[i + nTDSCAQCFModel_ValveON_EquOver2][i] = - 0.5;
		}

		break;
	case 1:
		nTDSCAQCFModel_ValveOFF_Equ					= (nTDSCQDM_ValveOFF_Equ1 + nTDSCQDM_ValveOFF_Equ2 + nTDSCQDM_ValveOFF_Equ3) * 2;
		nTDSCAQCFModel_ValveOFF_EquOver2			= nTDSCQDM_ValveOFF_Equ1 + nTDSCQDM_ValveOFF_Equ2 + nTDSCQDM_ValveOFF_Equ3;
		nTDSCAQCFModel_ValveOFF_State				= nTDSCQDM_ValveOFF_State * 2;
		nTDSCAQCFModel_ValveOFF_Control				= nTDSCQDM_ValveOFF_Control * 2;
		nTDSCAQCFModel_ValveOFF_Feqxx				= nTDSCQDM_ValveOFF_Feqxx * 2;
		nTDSCAQCFModel_ValveOFF_Fequu				= nTDSCQDM_ValveOFF_Fequu * 2;
		nTDSCAQCFModel_ValveOFF_Fequx				= nTDSCQDM_ValveOFF_Fequx * 2;

		// ---- Allocate arrays and vectors of SCAQCF object

		pTDSCAQCFModel_ValveOFF_Yeqx					= NewMatrix(nTDSCAQCFModel_ValveOFF_Equ, nTDSCAQCFModel_ValveOFF_State);
		pTDSCAQCFModel_ValveOFF_Yequ					= NewMatrix(nTDSCAQCFModel_ValveOFF_Equ, nTDSCAQCFModel_ValveOFF_Control);
		pTDSCAQCFModel_ValveOFF_Feqxx				= new SP_CUBIX[nTDSCAQCFModel_ValveOFF_Feqxx];
		pTDSCAQCFModel_ValveOFF_Fequu				= new SP_CUBIX[nTDSCAQCFModel_ValveOFF_Fequu];
		pTDSCAQCFModel_ValveOFF_Fequx				= new SP_CUBIX[nTDSCAQCFModel_ValveOFF_Fequx];
		pTDSCAQCFModel_ValveOFF_Neqx					= NewMatrix(nTDSCAQCFModel_ValveOFF_Equ, nTDSCAQCFModel_ValveOFF_State/2);
		pTDSCAQCFModel_ValveOFF_Nequ					= NewMatrix(nTDSCAQCFModel_ValveOFF_Equ, nTDSCAQCFModel_ValveOFF_Control/2);
		pTDSCAQCFModel_ValveOFF_Meq					= NewMatrix(nTDSCAQCFModel_ValveOFF_Equ, nTDSCAQCFModel_ValveOFF_State / 2);
		pTDSCAQCFModel_ValveOFF_Keq					= NewVector(nTDSCAQCFModel_ValveOFF_Equ);

		pTDSCAQCFModel_ValveOFF_StateNormFactor		= NewVector(nTDSCAQCFModel_ValveOFF_State);
		pTDSCAQCFModel_ValveOFF_ThroughNormFactor	= NewVector(nTDSCAQCFModel_ValveOFF_Equ);
		pTDSCAQCFModel_ValveOFF_ControlNormFactor	= NewVector(nTDSCAQCFModel_ValveOFF_Control);
		pTDSCAQCFModel_ValveOFF_NodeName				= new CString[nTDSCQDM_ValveOFF_Equ1 * 2];

		// ----- Create SCAQCF device model from quadratized device model

		// ----- SCAQCF State Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveOFF_State; i++)
		{
			pTDSCAQCFModel_ValveOFF_StateNormFactor[i] = pTDSCQDM_ValveOFF_StateNormFactor[i];
			pTDSCAQCFModel_ValveOFF_StateNormFactor[i + nTDSCQDM_ValveOFF_State] = pTDSCQDM_ValveOFF_StateNormFactor[i];
		}

		// ----- SCAQCF Control Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveOFF_Control; i++)
		{
			pTDSCAQCFModel_ValveOFF_ControlNormFactor[i] = pTDSCQDM_ValveOFF_ControlNormFactor[i];
			pTDSCAQCFModel_ValveOFF_ControlNormFactor[i + nTDSCQDM_ValveOFF_Control] = pTDSCQDM_ValveOFF_ControlNormFactor[i];
		}

		// ----- SCAQCF Through Variables Normalization Factors

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i] = pTDSCQDM_ValveOFF_ThroughNormFactor1[i];
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i + nTDSCQDM_ValveOFF_State] = pTDSCQDM_ValveOFF_ThroughNormFactor1[i];
		}

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ2; i++)
		{
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i+nTDSCQDM_ValveOFF_Equ1] = pTDSCQDM_ValveOFF_ThroughNormFactor2[i];
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i+nTDSCQDM_ValveOFF_Equ1 + nTDSCQDM_ValveOFF_State] = pTDSCQDM_ValveOFF_ThroughNormFactor2[i];
		}

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ3; i++)
		{
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i+nTDSCQDM_ValveOFF_Equ1+nTDSCQDM_ValveOFF_Equ2] = pTDSCQDM_ValveOFF_ThroughNormFactor3[i];
			pTDSCAQCFModel_ValveOFF_ThroughNormFactor[i+nTDSCQDM_ValveOFF_Equ1+nTDSCQDM_ValveOFF_Equ2+nTDSCQDM_ValveOFF_State] = pTDSCQDM_ValveOFF_ThroughNormFactor3[i];
		}


		//----contributions from Yeqx1 to Yeqx
		//----contributions from Yeqx1 to Neqx

		for ( i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			for ( j = 0; j < nTDSCQDM_ValveOFF_State; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Yeqx1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] -= dValue;

					// Contributions to Neqx time tm part
					irow = i + nTDSCAQCFModel_ValveOFF_EquOver2;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yeqx time tm part
					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Yequ1 to Yequ
		//----contributions from Yequ1 to Nequ

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveOFF_Control; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Yequ1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yequ and Nequ time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yequ[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveOFF_Nequ[irow][icolumn] -= dValue;

					// Contributions to Nequ time tm part
					irow = i + nTDSCAQCFModel_ValveOFF_EquOver2;
					pTDSCAQCFModel_ValveOFF_Nequ[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yequ time tm part
					icolumn = j + nTDSCQDM_ValveOFF_Control;
					pTDSCAQCFModel_ValveOFF_Yequ[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Deqxd1 to Yeqx
		//----contributions from Deqxd1 to Neqx

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveOFF_State; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Deqxd1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d4Overh * dValue;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] += d4Overh * dValue;

					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] -= d8Overh * dValue;

					// Contributions to Yeqx time tm part
					irow = i + nTDSCAQCFModel_ValveOFF_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d0p5Overh * dValue;

					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d2Overh * dValue;

					// Contributions to Neqx time tm part
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] -= d2p5Overh * dValue;
				}
			}
		}

		//----contributions from Ceq1 to Keq

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			dValue = pTDSCQDM_ValveOFF_Ceqc1[i];
			if (dValue != 0.0)
			{
				irow = i + nTDSCAQCFModel_ValveOFF_EquOver2;
				pTDSCAQCFModel_ValveOFF_Keq[irow] = 1.5 * dValue;
			}
		}

		//----contributions from Yeqx2 to Yeqx
		//----contributions from Yeqx2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveOFF_State; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Yeqx2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i + nTDSCQDM_ValveOFF_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] -= dValue;

					// Contributions to Neqx time tm part
					irow = i + nTDSCQDM_ValveOFF_Equ1 + nTDSCAQCFModel_ValveOFF_EquOver2;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yeqx time tm part
					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Yequ2 to Yeqx
		//----contributions from Yequ2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveOFF_Control; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Yequ2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yequ and Nequ time t part
					irow = i + nTDSCQDM_ValveOFF_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yequ[irow][icolumn] += dValue;
					pTDSCAQCFModel_ValveOFF_Nequ[irow][icolumn] -= dValue;

					// Contributions to Nequ time tm part
					irow = i + nTDSCQDM_ValveOFF_Equ1 + nTDSCAQCFModel_ValveOFF_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Nequ[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yequ time tm part
					icolumn = j + nTDSCQDM_ValveOFF_Control;
					pTDSCAQCFModel_ValveOFF_Yequ[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Deqxd2 to Yeqx
		//----contributions from Deqxd2 to Neqx

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ2; i++)
		{
			for (j = 0; j < nTDSCQDM_ValveOFF_State; j++)
			{
				dValue = pTDSCQDM_ValveOFF_Deqxd2[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i + nTDSCQDM_ValveOFF_Equ1;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d4Overh * dValue;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] += d4Overh * dValue;

					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] -= d8Overh * dValue;

					// Contributions to Yeqx time tm part
					irow = i + nTDSCQDM_ValveOFF_Equ1 + nTDSCAQCFModel_ValveOFF_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d0p5Overh * dValue;

					icolumn = j + nTDSCQDM_ValveOFF_State;
					pTDSCAQCFModel_ValveOFF_Yeqx[irow][icolumn] += d2Overh * dValue;

					// Contributions to Neqx time tm part
					icolumn = j;
					pTDSCAQCFModel_ValveOFF_Neqx[irow][icolumn] -= d2p5Overh * dValue;
				}
			}
		}

		//----contributions from Ceq2 to Keq

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ2; i++)
		{
			dValue = pTDSCQDM_ValveOFF_Ceqc2[i];
			if (dValue != 0.0)
			{
				irow = i + nTDSCQDM_ValveOFF_Equ1 + nTDSCAQCFModel_ValveOFF_EquOver2;
				pTDSCAQCFModel_ValveOFF_Keq[irow] = 1.5 * dValue;
			}
		}

		//----update of Meqx

		for (i = 0; i < nTDSCQDM_ValveOFF_Equ1; i++)
		{
			pTDSCAQCFModel_ValveOFF_Meq[i][i] = 1.0;
			pTDSCAQCFModel_ValveOFF_Meq[i + nTDSCAQCFModel_ValveOFF_EquOver2][i] = - 0.5;
		}

		break;
	case 2:
		nTDSCAQCFModel_CAP_Equ					= (nTDSCQDM_CAP_Equ1 + nTDSCQDM_CAP_Equ2 + nTDSCQDM_CAP_Equ3) * 2;
		nTDSCAQCFModel_CAP_EquOver2				= nTDSCQDM_CAP_Equ1 + nTDSCQDM_CAP_Equ2 + nTDSCQDM_CAP_Equ3;
		nTDSCAQCFModel_CAP_State				= nTDSCQDM_CAP_State * 2;
		nTDSCAQCFModel_CAP_Control				= nTDSCQDM_CAP_Control * 2;
		nTDSCAQCFModel_CAP_Feqxx				= nTDSCQDM_CAP_Feqxx * 2;
		nTDSCAQCFModel_CAP_Fequu				= nTDSCQDM_CAP_Fequu * 2;
		nTDSCAQCFModel_CAP_Fequx				= nTDSCQDM_CAP_Fequx * 2;

		// ---- Allocate arrays and vectors of SCAQCF object

		pTDSCAQCFModel_CAP_Yeqx					= NewMatrix(nTDSCAQCFModel_CAP_Equ, nTDSCAQCFModel_CAP_State);
		pTDSCAQCFModel_CAP_Yequ					= NewMatrix(nTDSCAQCFModel_CAP_Equ, nTDSCAQCFModel_CAP_Control);
		pTDSCAQCFModel_CAP_Feqxx				= new SP_CUBIX[nTDSCAQCFModel_CAP_Feqxx];
		pTDSCAQCFModel_CAP_Fequu				= new SP_CUBIX[nTDSCAQCFModel_CAP_Fequu];
		pTDSCAQCFModel_CAP_Fequx				= new SP_CUBIX[nTDSCAQCFModel_CAP_Fequx];
		pTDSCAQCFModel_CAP_Neqx					= NewMatrix(nTDSCAQCFModel_CAP_Equ, nTDSCAQCFModel_CAP_State/2);
		pTDSCAQCFModel_CAP_Nequ					= NewMatrix(nTDSCAQCFModel_CAP_Equ, nTDSCAQCFModel_CAP_Control/2);
		pTDSCAQCFModel_CAP_Meq					= NewMatrix(nTDSCAQCFModel_CAP_Equ, nTDSCAQCFModel_CAP_State / 2);
		pTDSCAQCFModel_CAP_Keq					= NewVector(nTDSCAQCFModel_CAP_Equ);

		pTDSCAQCFModel_CAP_StateNormFactor		= NewVector(nTDSCAQCFModel_CAP_State);
		pTDSCAQCFModel_CAP_ThroughNormFactor	= NewVector(nTDSCAQCFModel_CAP_Equ);
		pTDSCAQCFModel_CAP_ControlNormFactor	= NewVector(nTDSCAQCFModel_CAP_Control);
		pTDSCAQCFModel_CAP_NodeName				= new CString[nTDSCQDM_CAP_Equ1 * 2];

		// ----- Create SCAQCF device model from quadratized device model

		// ----- SCAQCF State Normalization Factors

		for (i = 0; i < nTDSCQDM_CAP_State; i++)
		{
			pTDSCAQCFModel_CAP_StateNormFactor[i] = pTDSCQDM_CAP_StateNormFactor[i];
			pTDSCAQCFModel_CAP_StateNormFactor[i + nTDSCQDM_CAP_State] = pTDSCQDM_CAP_StateNormFactor[i];
		}

		// ----- SCAQCF Control Normalization Factors

		for (i = 0; i < nTDSCQDM_CAP_Control; i++)
		{
			pTDSCAQCFModel_CAP_ControlNormFactor[i] = pTDSCQDM_CAP_ControlNormFactor[i];
			pTDSCAQCFModel_CAP_ControlNormFactor[i + nTDSCQDM_CAP_Control] = pTDSCQDM_CAP_ControlNormFactor[i];
		}

		// ----- SCAQCF Through Variables Normalization Factors

		for (i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			pTDSCAQCFModel_CAP_ThroughNormFactor[i] = pTDSCQDM_CAP_ThroughNormFactor1[i];
			pTDSCAQCFModel_CAP_ThroughNormFactor[i + nTDSCQDM_CAP_State] = pTDSCQDM_CAP_ThroughNormFactor1[i];
		}

		for (i = 0; i < nTDSCQDM_CAP_Equ2; i++)
		{
			pTDSCAQCFModel_CAP_ThroughNormFactor[i+nTDSCQDM_CAP_Equ1] = pTDSCQDM_CAP_ThroughNormFactor2[i];
			pTDSCAQCFModel_CAP_ThroughNormFactor[i+nTDSCQDM_CAP_Equ1 + nTDSCQDM_CAP_State] = pTDSCQDM_CAP_ThroughNormFactor2[i];
		}

		for (i = 0; i < nTDSCQDM_CAP_Equ3; i++)
		{
			pTDSCAQCFModel_CAP_ThroughNormFactor[i+nTDSCQDM_CAP_Equ1+nTDSCQDM_CAP_Equ2] = pTDSCQDM_CAP_ThroughNormFactor3[i];
			pTDSCAQCFModel_CAP_ThroughNormFactor[i+nTDSCQDM_CAP_Equ1+nTDSCQDM_CAP_Equ2+nTDSCQDM_CAP_State] = pTDSCQDM_CAP_ThroughNormFactor3[i];
		}


		//----contributions from Yeqx1 to Yeqx
		//----contributions from Yeqx1 to Neqx

		for ( i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			for ( j = 0; j < nTDSCQDM_CAP_State; j++)
			{
				dValue = pTDSCQDM_CAP_Yeqx1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] += dValue;
					pTDSCAQCFModel_CAP_Neqx[irow][icolumn] -= dValue;

					// Contributions to Neqx time tm part
					irow = i + nTDSCAQCFModel_CAP_EquOver2;
					pTDSCAQCFModel_CAP_Neqx[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yeqx time tm part
					icolumn = j + nTDSCQDM_CAP_State;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Yequ1 to Yequ
		//----contributions from Yequ1 to Nequ

		for (i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_CAP_Control; j++)
			{
				dValue = pTDSCQDM_CAP_Yequ1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yequ and Nequ time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_CAP_Yequ[irow][icolumn] += dValue;
					pTDSCAQCFModel_CAP_Nequ[irow][icolumn] -= dValue;

					// Contributions to Nequ time tm part
					irow = i + nTDSCAQCFModel_CAP_EquOver2;
					pTDSCAQCFModel_CAP_Nequ[irow][icolumn] += 0.5 * dValue;

					// Contributions to Yequ time tm part
					icolumn = j + nTDSCQDM_CAP_Control;
					pTDSCAQCFModel_CAP_Yequ[irow][icolumn] += dValue;
				}
			}
		}

		//----contributions from Deqxd1 to Yeqx
		//----contributions from Deqxd1 to Neqx

		for (i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			for (j = 0; j < nTDSCQDM_CAP_State; j++)
			{
				dValue = pTDSCQDM_CAP_Deqxd1[i][j];
				if (dValue != 0.0)
				{
					// Contributions to Yeqx and Neqx time t part
					irow = i;
					icolumn = j;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] += d4Overh * dValue;
					pTDSCAQCFModel_CAP_Neqx[irow][icolumn] += d4Overh * dValue;

					icolumn = j + nTDSCQDM_CAP_State;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] -= d8Overh * dValue;

					// Contributions to Yeqx time tm part
					irow = i + nTDSCAQCFModel_CAP_EquOver2;
					icolumn = j;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] += d0p5Overh * dValue;

					icolumn = j + nTDSCQDM_CAP_State;
					pTDSCAQCFModel_CAP_Yeqx[irow][icolumn] += d2Overh * dValue;

					// Contributions to Neqx time tm part
					icolumn = j;
					pTDSCAQCFModel_CAP_Neqx[irow][icolumn] -= d2p5Overh * dValue;
				}
			}
		}

		//----contributions from Ceq1 to Keq

		for (i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			dValue = pTDSCQDM_CAP_Ceqc1[i];
			if (dValue != 0.0)
			{
				irow = i + nTDSCAQCFModel_CAP_EquOver2;
				pTDSCAQCFModel_CAP_Keq[irow] = 1.5 * dValue;
			}
		}

		//----update of Meqx

		for (i = 0; i < nTDSCQDM_CAP_Equ1; i++)
		{
			pTDSCAQCFModel_CAP_Meq[i][i] = 1.0;
			pTDSCAQCFModel_CAP_Meq[i + nTDSCAQCFModel_CAP_EquOver2][i] = - 0.5;
		}

		break;
	default:
		AfxMessageBox("Wrong Model Selected in M801");
		break;
	}

	return true;
}
//----------------------------------------------------------------------------
BOOL M801::TQN_CreateConverterSCAQCFModel()
{

	//	this routine creates the ACQCF model 
	//		given:
	//			iConverterMode
	//

	int	i, j, i1, j1;
	int	iValve;
	
	TQN_ClearSCAQCF();

	nTDSCQDModel_Equ1					= 9;
	nTDSCQDModel_Equ2					= 18;
	nTDSCQDModel_State					= 27;

	nTDSCAQCFModel_Equ					= 54;
	nTDSCAQCFModel_EquOver2				= nTDSCAQCFModel_Equ / 2;
	nTDSCAQCFModel_State				= 54;
	nTDSCAQCFModel_Node					= 9;

	pTDSCAQCFModel_Yeqx					= NewMatrix(nTDSCAQCFModel_Equ, nTDSCAQCFModel_State);
	pTDSCAQCFModel_Yequ					= NewMatrix(nTDSCAQCFModel_Equ, nTDSCAQCFModel_Control);
	pTDSCAQCFModel_Feqxx				= new SP_CUBIX[nTDSCAQCFModel_Feqxx];
	pTDSCAQCFModel_Fequu				= new SP_CUBIX[nTDSCAQCFModel_Fequu];
	pTDSCAQCFModel_Fequx				= new SP_CUBIX[nTDSCAQCFModel_Fequx];
	pTDSCAQCFModel_Neqx					= NewMatrix(nTDSCAQCFModel_Equ, nTDSCAQCFModel_State / 2);
	pTDSCAQCFModel_Nequ					= NewMatrix(nTDSCAQCFModel_Equ, nTDSCAQCFModel_Control / 2);
	pTDSCAQCFModel_Meq					= NewMatrix(nTDSCAQCFModel_Equ, nTDSCAQCFModel_State / 2);
	pTDSCAQCFModel_Keq					= NewVector(nTDSCAQCFModel_Equ);

	pTDSCAQCFModel_StateNormFactor		= NewVector(nTDSCAQCFModel_State);
	pTDSCAQCFModel_ThroughNormFactor	= NewVector(nTDSCAQCFModel_Equ);
	pTDSCAQCFModel_ControlNormFactor	= NewVector(nTDSCAQCFModel_Control);

	pTDSCAQCFModel_NodeName				= new CString[nTDSCAQCFModel_Node];
	pTDSCAQCFModel_u					= NewVector(nTDSCAQCFModel_Control);

	//------Form matrix by summing up valve contributions

	for ( iValve = 0 ; iValve < nValves ; iValve++ )
	{
		if( iValvePresentTimeStatus[iValve]==1 )
		{
			for ( i=0 ; i<nValTerm_tq ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<nValTerm_tq ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCAQCFModel_Yeqx[i1][j1] += pTDSCAQCFModel_ValveON_Yeqx[i][j];
				}

				for ( j=0 ; j<nValTerm ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCAQCFModel_Neqx[i1][j1] += pTDSCAQCFModel_ValveON_Neqx[i][j];
					pTDSCAQCFModel_Meq[i1][j1] += pTDSCAQCFModel_ValveON_Meq[i][j];
				}
			}
		}
		else if ( iValvePresentTimeStatus[iValve]==0 )
		{
			for ( i=0 ; i<nValTerm_tq ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<nValTerm_tq ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCAQCFModel_Yeqx[i1][j1] += pTDSCAQCFModel_ValveOFF_Yeqx[i][j];
				}

				for ( j=0 ; j<nValTerm ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCAQCFModel_Neqx[i1][j1] += pTDSCAQCFModel_ValveOFF_Neqx[i][j];
					pTDSCAQCFModel_Meq[i1][j1] += pTDSCAQCFModel_ValveOFF_Meq[i][j];
				}
			}
		}
	}

	//------Capacitor contributions

	for ( i=0 ; i<nCapTerm_tq ; i++ )
	{
		i1 = iCapPointerM801_tq[i];
		for ( j=0 ; j<nCapTerm_tq ; j++ )
		{
			j1 = iCapPointerM801_tq[j];
			pTDSCAQCFModel_Yeqx[i1][j1] += pTDSCAQCFModel_CAP_Yeqx[i][j];
		}

		for ( j=0 ; j<nCapTerm ; j++ )
		{
			j1 = iCapPointerM801_tq[j];
			pTDSCAQCFModel_Neqx[i1][j1] += pTDSCAQCFModel_CAP_Neqx[i][j];
			pTDSCAQCFModel_Meq[i1][j1] += pTDSCAQCFModel_CAP_Meq[i][j];
		}
	}

	//------Control signal terminals

//	pTDSCAQCFModel_Yeqx[5][5] += 1.0;			// zero crossing
//	pTDSCAQCFModel_Yeqx[6][6] += 1.0;			// Mag
//	pTDSCAQCFModel_Yeqx[7][7] += 1.0;			// Power
//
//	pTDSCAQCFModel_Yeqx[5+26][5+26] += 1.0;		// zero crossing
//	pTDSCAQCFModel_Yeqx[6+26][6+26] += 1.0;		// Mag
//	pTDSCAQCFModel_Yeqx[7+26][7+26] += 1.0;		// Power
//
//	pTDSCAQCFModel_Neqx[5][5] = -1.0;
//	pTDSCAQCFModel_Neqx[6][6] = -1.0;
//	pTDSCAQCFModel_Neqx[7][7] = -1.0;
//	pTDSCAQCFModel_Neqx[5+26][5] = 0.5;
//	pTDSCAQCFModel_Neqx[6+26][6] = 0.5;
//	pTDSCAQCFModel_Neqx[7+26][7] = 0.5;

	pTDSCAQCFModel_Meq[0][0] = 1.0;
	pTDSCAQCFModel_Meq[1][1] = 1.0;
	pTDSCAQCFModel_Meq[2][2] = 1.0;
	pTDSCAQCFModel_Meq[3][3] = 1.0;
	pTDSCAQCFModel_Meq[4][4] = 1.0;
//	pTDSCAQCFModel_Meq[5][5] = 1.0;
//	pTDSCAQCFModel_Meq[6][6] = 1.0;
//	pTDSCAQCFModel_Meq[7][7] = 1.0;

	pTDSCAQCFModel_Meq[0+27][0] = -0.5;
	pTDSCAQCFModel_Meq[1+27][1] = -0.5;
	pTDSCAQCFModel_Meq[2+27][2] = -0.5;
	pTDSCAQCFModel_Meq[3+27][3] = -0.5;
	pTDSCAQCFModel_Meq[4+27][4] = -0.5;
//	pTDSCAQCFModel_Meq[5+26][5] = -0.5;
//	pTDSCAQCFModel_Meq[6+26][6] = -0.5;
//	pTDSCAQCFModel_Meq[7+26][7] = -0.5;

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the ACF model 
//		of entire converter for an time 
//        interval: ddtsecsM801 
//		The model is stored in arrays
//			yeq_real_tq, and
//			beq_real_tq
//
//-------------------------------------------------------------------
BOOL M801::TQN_UpdatePastHistory()
{
	int	i, j;

	//------initialize array beq_real_tq

	for (i = 0; i<devsta_tq; i++)
	{
		beq_real_tq[i] = 0.0;
	}

	for (i = 0; i<devequ_tq; i++)
	{
		for (j = 0; j<devsta; j++)
		{
			beq_real_tq[i] += dNeq_real_tq[i][j] * xa_real_tq[j];
		}

		for (j = 0; j<devequ; j++)
		{
			beq_real_tq[i] += dMeq_real_tq[i][j] * xi_real_tq[j];
		}
		beq_real_tq[i] += dKeq_real_tq[i];
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_DebugReportModel801()
{
	int iValve, k1, k2, k3, k4, iTemp;
	double dVoltage, dCurrent;
	CString text;

	DumpString("\nModel 801 Debug Report\n");
	DumpString("----------------------\n");

	DumpString("----- Feedback Control Values -----\n");
	DumpDoubleScalar("dVacZeroTime",dVacZeroTime);
	DumpDoubleScalar("VaMagn",dPosSeqVMagn/sqrt(3)*sqrt(2));
	DumpDoubleScalar("dRealPowerAC",dRealPowerAC);
	DumpDoubleScalar("dFrequency",dFrequency);

	DumpString("----- Valve Status -----\n");
	if (iInverterMode == 12)	DumpIntScalar("iInverterMode",-1);
	else						DumpIntScalar("iInverterMode",iInverterMode);
	if (bCommutationON)	DumpString("bCommutationON = true \n");
	else				DumpString("bCommutationON = false \n");
	DumpIntScalar("iNextCommutationValve",iNextCommutationValve);
	DumpIntScalar("iNextTurnONValve",iNextTurnONValve);
	for ( iValve=0; iValve<nValves; iValve++ )
	{
		text.Format("Valve %d Status", iValve);
		DumpIntScalar(text,iValveStatusPerMode[iValve][iInverterMode]);
		
		k1 = iValvePointer[1][iValve];
		k2 = iValvePointer[3][iValve];
		k3 = iValvePointer[0][iValve];
		k4 = iValvePointer[4][iValve];
		dVoltage = xa_real_tq[k2] - xa_real_tq[k1];
		text.Format("Valve %d Voltage Across Thyristor: %12.6f\n", iValve, dVoltage);
		DumpString(text);
		dVoltage = xa_real_tq[k3] - xa_real_tq[k1];
		text.Format("Valve %d Voltage Across Valve Terminals: %12.6f\n", iValve, dVoltage);
		DumpString(text);
		dCurrent = xa_real_tq[k4];
		text.Format("Valve %d Current Through Inductor: %12.6f\n\n", iValve, dCurrent);
		DumpString(text);
	}

	DumpString("----- Timing -----\n");
	DumpDoubleScalar("run_time",pNetSolver->run_time);
	DumpDoubleScalar("dConverterRelativeTime",dConverterRelativeTime);
	DumpDoubleScalar("dConverterTimeRef",dConverterTimeRef);
	DumpDoubleScalar("dCommutationTime",dCommutationTime);
	for ( iValve=0; iValve<nValves; iValve++ )
	{
		text.Format("Valve %d Turn ON Time", iValve);
		DumpDoubleScalar(text,dValveTurnONTimes[iValve]);
	}

	DumpString("----- Terminal Quantities -----\n");
	DumpDoubleScalar("v_a(t)",  xa_real_tq[0]);
	DumpDoubleScalar("v_b(t)",  xa_real_tq[1]);
	DumpDoubleScalar("v_c(t)",  xa_real_tq[2]);
	DumpDoubleScalar("v_ad(t)", xa_real_tq[3]);
	DumpDoubleScalar("v_kd(t)", xa_real_tq[4]);
	DumpDoubleScalar("v_ak(t)", (xa_real_tq[3]-xa_real_tq[4]));

	DumpDoubleScalar("i_a(t)",  xi_real_tq[0]);
	DumpDoubleScalar("i_b(t)",  xi_real_tq[1]);
	DumpDoubleScalar("i_c(t)",  xi_real_tq[2]);
	DumpDoubleScalar("i_ad(t)", xi_real_tq[3]);
	DumpDoubleScalar("i_kd(t)", xi_real_tq[4]);


	DumpString("\n---------------------------------------------\n\n");

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_DebugMatrixReportModel801()
{
	//DumpString("\nModel 801 Debug Matrix Report\n");
	//DumpString("----------------------\n");

	//DumpDoubleVector("xi_real_tq",devsta_tq,xi_real_tq);	
	//DumpDoubleVector("xa_real_tq",devsta_tq,xa_real_tq);

	//DumpIntVector("iValveStatusPerModeM801",6,iValveStatusPerModeM801);
	//DumpDoubleScalar("Run_time",pNetSolver->run_time);

	//// Checking the matrix of Y, P and Z of single valve

	DumpDoubleMatrix("dYmatrixSvalveON",10,10,dYmatrixSvalveON);
	DumpDoubleMatrix("dYmatrixSvalveOFF",10,10,dYmatrixSvalveOFF);
	DumpDoubleMatrix("dPmatrixSvalveON",10,5,dPmatrixSvalveON);
	DumpDoubleMatrix("dPmatrixSvalveOFF",10,5,dPmatrixSvalveOFF);
	DumpDoubleMatrix("dZmatrixSvalve",10,5,dZmatrixSvalve);

	DumpDoubleMatrix("dYmatrixCap",4,4,dYmatrixCap);
	DumpDoubleMatrix("dPmatrixCap",4,2,dPmatrixCap);
	DumpDoubleMatrix("dZmatrixCap",4,2,dZmatrixCap);

	//// Checking the matrix of Y, P, and Z of converter

	DumpDoubleMatrix("dYmatrixConverter",devsta_tq,devsta_tq,dYmatrixConverter);
	//DumpDoubleMatrix("dPmatrixConverter",devsta_tq,devsta_tq/2,dPmatrixConverter);
	//DumpDoubleMatrix("dZmatrixConverter",devsta_tq,devsta_tq/2,dZmatrixConverter);

	//// Checking the matrix of yeq and beq of converter

	//DumpDoubleMatrix("Yeq_real_tq",devsta_tq,devsta_tq,yeq_real_tq);
	//DumpDoubleVector("beq_real_tq",devsta_tq,beq_real_tq);

	DumpString("\n---------------------------------------------\n\n");

	return true;
}
//-------------------------------------------------------------------
void M801::Init_SCQDM_ValveON()
{
	pTDSCQDM_ValveON_Yeqx1						= NULL;				// pointer to SCQDM ValveON model linear state matrix in linear through equations
	pTDSCQDM_ValveON_Yequ1						= NULL;				// pointer to SCQDM ValveON model control state matrix in linear through equations
	pTDSCQDM_ValveON_Deqxd1						= NULL;				// pointer to SCQDM ValveON model defferential state matrix in linear through equations
	pTDSCQDM_ValveON_Ceqc1						= NULL;				// pointer to SCQDM ValveON model constant in linear through equations
	
	pTDSCQDM_ValveON_Yeqx2						= NULL;				// pointer to SCQDM ValveON model linear state matrix in linear internal equations
	pTDSCQDM_ValveON_Yequ2						= NULL;				// pointer to SCQDM ValveON model control state matrix in linear internal equations
	pTDSCQDM_ValveON_Deqxd2						= NULL;				// pointer to SCQDM ValveON model defferential state matrix in linear internal equations
	pTDSCQDM_ValveON_Ceqc2						= NULL;				// pointer to SCQDM ValveON model constant in linear internal equations
	
	pTDSCQDM_ValveON_Yeqx3						= NULL;				// pointer to SCQDM ValveON model linear state matrix in nonlinear equations
	pTDSCQDM_ValveON_Yequ3						= NULL;				// pointer to SCQDM ValveON model control state matrix in nonlinear equations
	pTDSCQDM_ValveON_Feqxx3						= NULL;				// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveON_Fequu3						= NULL;				// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveON_Fequx3						= NULL;				// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveON_Ceqc3						= NULL;				// pointer to SCQDM ValveON model constant in nonlinear equations
	
	pTDSCQDM_ValveON_NodeName					= NULL;				// pointer to SCQDM ValveON model connectivity vectors
	pTDSCQDM_ValveON_StateNormFactor			= NULL;				// pointer to SCQDM ValveON model state normalization factors
	pTDSCQDM_ValveON_ThroughNormFactor1			= NULL;				// pointer to SCQDM ValveON model through variable normalization factors in the first equation set
	pTDSCQDM_ValveON_ThroughNormFactor2			= NULL;				// pointer to SCQDM ValveON model through variable normalization factors in the second equation set
	pTDSCQDM_ValveON_ThroughNormFactor3			= NULL;				// pointer to SCQDM ValveON model through variable normalization factors in the third equation set
	pTDSCQDM_ValveON_ControlNormFactor			= NULL;				// pointer to SCQDM ValveON model control normalization factors
}
//-------------------------------------------------------------------
void M801::Init_SCQDM_ValveOFF()
{
	pTDSCQDM_ValveOFF_Yeqx1						= NULL;				// pointer to SCQDM ValveOFF model linear state matrix in linear through equations
	pTDSCQDM_ValveOFF_Yequ1						= NULL;				// pointer to SCQDM ValveOFF model control state matrix in linear through equations
	pTDSCQDM_ValveOFF_Deqxd1					= NULL;				// pointer to SCQDM ValveOFF model defferential state matrix in linear through equations
	pTDSCQDM_ValveOFF_Ceqc1						= NULL;				// pointer to SCQDM ValveOFF model constant in linear through equations
	
	pTDSCQDM_ValveOFF_Yeqx2						= NULL;				// pointer to SCQDM ValveOFF model linear state matrix in linear internal equations
	pTDSCQDM_ValveOFF_Yequ2						= NULL;				// pointer to SCQDM ValveOFF model control state matrix in linear internal equations
	pTDSCQDM_ValveOFF_Deqxd2					= NULL;				// pointer to SCQDM ValveOFF model defferential state matrix in linear internal equations
	pTDSCQDM_ValveOFF_Ceqc2						= NULL;				// pointer to SCQDM ValveOFF model constant in linear internal equations
	
	pTDSCQDM_ValveOFF_Yeqx3						= NULL;				// pointer to SCQDM ValveOFF model linear state matrix in nonlinear equations
	pTDSCQDM_ValveOFF_Yequ3						= NULL;				// pointer to SCQDM ValveOFF model control state matrix in nonlinear equations
	pTDSCQDM_ValveOFF_Feqxx3					= NULL;				// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveOFF_Fequu3					= NULL;				// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveOFF_Fequx3					= NULL;				// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
	pTDSCQDM_ValveOFF_Ceqc3						= NULL;				// pointer to SCQDM ValveOFF model constant in nonlinear equations
	
	pTDSCQDM_ValveOFF_NodeName					= NULL;				// pointer to SCQDM ValveOFF model connectivity vectors
	pTDSCQDM_ValveOFF_StateNormFactor			= NULL;				// pointer to SCQDM ValveOFF model state normalization factors
	pTDSCQDM_ValveOFF_ThroughNormFactor1		= NULL;				// pointer to SCQDM ValveOFF model through variable normalization factors in the first equation set
	pTDSCQDM_ValveOFF_ThroughNormFactor2		= NULL;				// pointer to SCQDM ValveOFF model through variable normalization factors in the second equation set
	pTDSCQDM_ValveOFF_ThroughNormFactor3		= NULL;				// pointer to SCQDM ValveOFF model through variable normalization factors in the third equation set
	pTDSCQDM_ValveOFF_ControlNormFactor			= NULL;				// pointer to SCQDM ValveOFF model control normalization factors
}
//-------------------------------------------------------------------
void M801::Clear_SCQDM_ValveON()
{
	if( pTDSCQDM_ValveON_Yeqx1 )					DeleteMatrix(pTDSCQDM_ValveON_Yeqx1);
	if( pTDSCQDM_ValveON_Yequ1 )					DeleteMatrix(pTDSCQDM_ValveON_Yequ1);
	if( pTDSCQDM_ValveON_Deqxd1	)					DeleteMatrix(pTDSCQDM_ValveON_Deqxd1);
	if( pTDSCQDM_ValveON_Ceqc1 )					delete	[] pTDSCQDM_ValveON_Ceqc1;

	if( pTDSCQDM_ValveON_Yeqx2 )					DeleteMatrix(pTDSCQDM_ValveON_Yeqx2);
	if( pTDSCQDM_ValveON_Yequ2 )					DeleteMatrix(pTDSCQDM_ValveON_Yequ2);
	if( pTDSCQDM_ValveON_Deqxd2	)					DeleteMatrix(pTDSCQDM_ValveON_Deqxd2);
	if( pTDSCQDM_ValveON_Ceqc2 )					delete	[] pTDSCQDM_ValveON_Ceqc2;

	if( pTDSCQDM_ValveON_Yeqx3 )					DeleteMatrix(pTDSCQDM_ValveON_Yeqx3);
	if( pTDSCQDM_ValveON_Yequ3 )					DeleteMatrix(pTDSCQDM_ValveON_Yequ3);
	if( pTDSCQDM_ValveON_Feqxx3	)					delete	[] pTDSCQDM_ValveON_Feqxx3;
	if( pTDSCQDM_ValveON_Fequu3	)					delete	[] pTDSCQDM_ValveON_Fequu3;
	if( pTDSCQDM_ValveON_Fequx3	)					delete	[] pTDSCQDM_ValveON_Fequx3;
	if( pTDSCQDM_ValveON_Ceqc3 )					delete	[] pTDSCQDM_ValveON_Ceqc3;

	if( pTDSCQDM_ValveON_NodeName )					delete	[] pTDSCQDM_ValveON_NodeName;
	if( pTDSCQDM_ValveON_StateNormFactor )			delete	[] pTDSCQDM_ValveON_StateNormFactor;
	if( pTDSCQDM_ValveON_ThroughNormFactor1	)		delete	[] pTDSCQDM_ValveON_ThroughNormFactor1;
	if( pTDSCQDM_ValveON_ThroughNormFactor2	)		delete	[] pTDSCQDM_ValveON_ThroughNormFactor2;
	if( pTDSCQDM_ValveON_ThroughNormFactor3	)		delete	[] pTDSCQDM_ValveON_ThroughNormFactor3;
	if( pTDSCQDM_ValveON_ControlNormFactor )		delete	[] pTDSCQDM_ValveON_ControlNormFactor;

	Init_SCQDM_ValveON();
}
//-------------------------------------------------------------------
void M801::Clear_SCQDM_ValveOFF()
{
	if( pTDSCQDM_ValveOFF_Yeqx1 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yeqx1);
	if( pTDSCQDM_ValveOFF_Yequ1 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yequ1);
	if( pTDSCQDM_ValveOFF_Deqxd1 )					DeleteMatrix(pTDSCQDM_ValveOFF_Deqxd1);
	if( pTDSCQDM_ValveOFF_Ceqc1 )					delete	[] pTDSCQDM_ValveOFF_Ceqc1;

	if( pTDSCQDM_ValveOFF_Yeqx2 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yeqx2);
	if( pTDSCQDM_ValveOFF_Yequ2 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yequ2);
	if( pTDSCQDM_ValveOFF_Deqxd2 )					DeleteMatrix(pTDSCQDM_ValveOFF_Deqxd2);
	if( pTDSCQDM_ValveOFF_Ceqc2 )					delete	[] pTDSCQDM_ValveOFF_Ceqc2;

	if( pTDSCQDM_ValveOFF_Yeqx3 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yeqx3);
	if( pTDSCQDM_ValveOFF_Yequ3 )					DeleteMatrix(pTDSCQDM_ValveOFF_Yequ3);
	if( pTDSCQDM_ValveOFF_Feqxx3 )					delete	[] pTDSCQDM_ValveOFF_Feqxx3;
	if( pTDSCQDM_ValveOFF_Fequu3 )					delete	[] pTDSCQDM_ValveOFF_Fequu3;
	if( pTDSCQDM_ValveOFF_Fequx3 )					delete	[] pTDSCQDM_ValveOFF_Fequx3;
	if( pTDSCQDM_ValveOFF_Ceqc3 )					delete	[] pTDSCQDM_ValveOFF_Ceqc3;

	if( pTDSCQDM_ValveOFF_NodeName )				delete	[] pTDSCQDM_ValveOFF_NodeName;
	if( pTDSCQDM_ValveOFF_StateNormFactor )			delete	[] pTDSCQDM_ValveOFF_StateNormFactor;
	if( pTDSCQDM_ValveOFF_ThroughNormFactor1 )		delete	[] pTDSCQDM_ValveOFF_ThroughNormFactor1;
	if( pTDSCQDM_ValveOFF_ThroughNormFactor2 )		delete	[] pTDSCQDM_ValveOFF_ThroughNormFactor2;
	if( pTDSCQDM_ValveOFF_ThroughNormFactor3 )		delete	[] pTDSCQDM_ValveOFF_ThroughNormFactor3;
	if( pTDSCQDM_ValveOFF_ControlNormFactor )		delete	[] pTDSCQDM_ValveOFF_ControlNormFactor;

	Init_SCQDM_ValveOFF();
}
//-------------------------------------------------------------------
void M801::Init_SCQDM_CAP()
{
	pTDSCQDM_CAP_Yeqx1						= NULL;				// pointer to SCQDM CAP model linear state matrix in linear through equations
	pTDSCQDM_CAP_Yequ1						= NULL;				// pointer to SCQDM CAP model control state matrix in linear through equations
	pTDSCQDM_CAP_Deqxd1						= NULL;				// pointer to SCQDM CAP model defferential state matrix in linear through equations
	pTDSCQDM_CAP_Ceqc1						= NULL;				// pointer to SCQDM CAP model constant in linear through equations
	
	pTDSCQDM_CAP_Yeqx2						= NULL;				// pointer to SCQDM CAP model linear state matrix in linear internal equations
	pTDSCQDM_CAP_Yequ2						= NULL;				// pointer to SCQDM CAP model control state matrix in linear internal equations
	pTDSCQDM_CAP_Deqxd2						= NULL;				// pointer to SCQDM CAP model defferential state matrix in linear internal equations
	pTDSCQDM_CAP_Ceqc2						= NULL;				// pointer to SCQDM CAP model constant in linear internal equations
	
	pTDSCQDM_CAP_Yeqx3						= NULL;				// pointer to SCQDM CAP model linear state matrix in nonlinear equations
	pTDSCQDM_CAP_Yequ3						= NULL;				// pointer to SCQDM CAP model control state matrix in nonlinear equations
	pTDSCQDM_CAP_Feqxx3						= NULL;				// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
	pTDSCQDM_CAP_Fequu3						= NULL;				// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
	pTDSCQDM_CAP_Fequx3						= NULL;				// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
	pTDSCQDM_CAP_Ceqc3						= NULL;				// pointer to SCQDM CAP model constant in nonlinear equations
	
	pTDSCQDM_CAP_NodeName					= NULL;				// pointer to SCQDM CAP model connectivity vectors
	pTDSCQDM_CAP_StateNormFactor			= NULL;				// pointer to SCQDM CAP model state normalization factors
	pTDSCQDM_CAP_ThroughNormFactor1			= NULL;				// pointer to SCQDM CAP model through variable normalization factors in the first equation set
	pTDSCQDM_CAP_ThroughNormFactor2			= NULL;				// pointer to SCQDM CAP model through variable normalization factors in the second equation set
	pTDSCQDM_CAP_ThroughNormFactor3			= NULL;				// pointer to SCQDM CAP model through variable normalization factors in the third equation set
	pTDSCQDM_CAP_ControlNormFactor			= NULL;				// pointer to SCQDM CAP model control normalization factors
}
//-------------------------------------------------------------------
void M801::Clear_SCQDM_CAP()
{
	if( pTDSCQDM_CAP_Yeqx1 )					DeleteMatrix(pTDSCQDM_CAP_Yeqx1);
	if( pTDSCQDM_CAP_Yequ1 )					DeleteMatrix(pTDSCQDM_CAP_Yequ1);
	if( pTDSCQDM_CAP_Deqxd1	)					DeleteMatrix(pTDSCQDM_CAP_Deqxd1);
	if( pTDSCQDM_CAP_Ceqc1 )					delete	[] pTDSCQDM_CAP_Ceqc1;

	if( pTDSCQDM_CAP_Yeqx2 )					DeleteMatrix(pTDSCQDM_CAP_Yeqx2);
	if( pTDSCQDM_CAP_Yequ2 )					DeleteMatrix(pTDSCQDM_CAP_Yequ2);
	if( pTDSCQDM_CAP_Deqxd2	)					DeleteMatrix(pTDSCQDM_CAP_Deqxd2);
	if( pTDSCQDM_CAP_Ceqc2 )					delete	[] pTDSCQDM_CAP_Ceqc2;

	if( pTDSCQDM_CAP_Yeqx3 )					DeleteMatrix(pTDSCQDM_CAP_Yeqx3);
	if( pTDSCQDM_CAP_Yequ3 )					DeleteMatrix(pTDSCQDM_CAP_Yequ3);
	if( pTDSCQDM_CAP_Feqxx3	)					delete	[] pTDSCQDM_CAP_Feqxx3;
	if( pTDSCQDM_CAP_Fequu3	)					delete	[] pTDSCQDM_CAP_Fequu3;
	if( pTDSCQDM_CAP_Fequx3	)					delete	[] pTDSCQDM_CAP_Fequx3;
	if( pTDSCQDM_CAP_Ceqc3 )					delete	[] pTDSCQDM_CAP_Ceqc3;

	if( pTDSCQDM_CAP_NodeName )					delete	[] pTDSCQDM_CAP_NodeName;
	if( pTDSCQDM_CAP_StateNormFactor )			delete	[] pTDSCQDM_CAP_StateNormFactor;
	if( pTDSCQDM_CAP_ThroughNormFactor1	)		delete	[] pTDSCQDM_CAP_ThroughNormFactor1;
	if( pTDSCQDM_CAP_ThroughNormFactor2	)		delete	[] pTDSCQDM_CAP_ThroughNormFactor2;
	if( pTDSCQDM_CAP_ThroughNormFactor3	)		delete	[] pTDSCQDM_CAP_ThroughNormFactor3;
	if( pTDSCQDM_CAP_ControlNormFactor )		delete	[] pTDSCQDM_CAP_ControlNormFactor;

	Init_SCQDM_CAP();
}
//-------------------------------------------------------------------
void M801::Init_SCAQCF_ValveON()
{
	pTDSCAQCFModel_ValveON_Yeqx					= NULL;					// pointer to SCAQCF ValveON model linear state matrix
	pTDSCAQCFModel_ValveON_Yequ					= NULL;					// pointer to SCAQCF ValveON model linear control matrix
	pTDSCAQCFModel_ValveON_Feqxx				= NULL;					// pointer to SCAQCF ValveON model quadratic state matrix
	pTDSCAQCFModel_ValveON_Fequu				= NULL;					// pointer to SCAQCF ValveON model quadratic control matrix
	pTDSCAQCFModel_ValveON_Fequx				= NULL;					// pointer to SCAQCF ValveON model quadratic product of state and control matrix 
	pTDSCAQCFModel_ValveON_Neqx					= NULL;					// pointer to SCAQCF ValveON model linear past history state matrix 
	pTDSCAQCFModel_ValveON_Nequ					= NULL;					// pointer to SCAQCF ValveON model linear past history control matrix 
	pTDSCAQCFModel_ValveON_Meq					= NULL;					// pointer to SCAQCF ValveON model linear past history through viariable matrix
	pTDSCAQCFModel_ValveON_Keq					= NULL;					// pointer to SCAQCF ValveON model constant part
	pTDSCAQCFModel_ValveON_NodeName				= NULL;					// pointer to SCAQCF ValveON model connectivity vectors
	pTDSCAQCFModel_ValveON_StateNormFactor		= NULL;					// pointer to SCAQCF ValveON model state normalization factors
	pTDSCAQCFModel_ValveON_ThroughNormFactor	= NULL;					// pointer to SCAQCF ValveON model through variable normalization factors
	pTDSCAQCFModel_ValveON_ControlNormFactor	= NULL;					// pointer to SCAQCF ValveON model control normalization factors
}
//-------------------------------------------------------------------
void M801::Init_SCAQCF_ValveOFF()
{
	pTDSCAQCFModel_ValveOFF_Yeqx				= NULL;					// pointer to SCAQCF ValveOFF model linear state matrix
	pTDSCAQCFModel_ValveOFF_Yequ				= NULL;					// pointer to SCAQCF ValveOFF model linear control matrix
	pTDSCAQCFModel_ValveOFF_Feqxx				= NULL;					// pointer to SCAQCF ValveOFF model quadratic state matrix
	pTDSCAQCFModel_ValveOFF_Fequu				= NULL;					// pointer to SCAQCF ValveOFF model quadratic control matrix
	pTDSCAQCFModel_ValveOFF_Fequx				= NULL;					// pointer to SCAQCF ValveOFF model quadratic product of state and control matrix 
	pTDSCAQCFModel_ValveOFF_Neqx				= NULL;					// pointer to SCAQCF ValveOFF model linear past history state matrix 
	pTDSCAQCFModel_ValveOFF_Nequ				= NULL;					// pointer to SCAQCF ValveOFF model linear past history control matrix 
	pTDSCAQCFModel_ValveOFF_Meq					= NULL;					// pointer to SCAQCF ValveOFF model linear past history through viariable matrix
	pTDSCAQCFModel_ValveOFF_Keq					= NULL;					// pointer to SCAQCF ValveOFF model constant part
	pTDSCAQCFModel_ValveOFF_NodeName			= NULL;					// pointer to SCAQCF ValveOFF model connectivity vectors
	pTDSCAQCFModel_ValveOFF_StateNormFactor		= NULL;					// pointer to SCAQCF ValveOFF model state normalization factors
	pTDSCAQCFModel_ValveOFF_ThroughNormFactor	= NULL;					// pointer to SCAQCF ValveOFF model through variable normalization factors
	pTDSCAQCFModel_ValveOFF_ControlNormFactor	= NULL;					// pointer to SCAQCF ValveOFF model control normalization factors
}
//-------------------------------------------------------------------
void M801::Clear_SCAQCF_ValveON()
{
	if (pTDSCAQCFModel_ValveON_Yeqx)				DeleteMatrix(pTDSCAQCFModel_ValveON_Yeqx);
	if (pTDSCAQCFModel_ValveON_Yequ)				DeleteMatrix(pTDSCAQCFModel_ValveON_Yequ);
	if (pTDSCAQCFModel_ValveON_Feqxx)				delete	[] pTDSCAQCFModel_ValveON_Feqxx;
	if (pTDSCAQCFModel_ValveON_Fequu)				delete	[] pTDSCAQCFModel_ValveON_Fequu;
	if (pTDSCAQCFModel_ValveON_Fequx)				delete	[] pTDSCAQCFModel_ValveON_Fequx;
	if (pTDSCAQCFModel_ValveON_Neqx)				DeleteMatrix(pTDSCAQCFModel_ValveON_Neqx);
	if (pTDSCAQCFModel_ValveON_Nequ)				DeleteMatrix(pTDSCAQCFModel_ValveON_Nequ);
	if (pTDSCAQCFModel_ValveON_Meq)					DeleteMatrix(pTDSCAQCFModel_ValveON_Meq);
	if (pTDSCAQCFModel_ValveON_Keq)					delete	[] pTDSCAQCFModel_ValveON_Keq;

	if (pTDSCAQCFModel_ValveON_NodeName)			delete	[] pTDSCAQCFModel_ValveON_NodeName;
	if (pTDSCAQCFModel_ValveON_StateNormFactor)		delete	[] pTDSCAQCFModel_ValveON_StateNormFactor;
	if (pTDSCAQCFModel_ValveON_ThroughNormFactor)	delete	[] pTDSCAQCFModel_ValveON_ThroughNormFactor;
	if (pTDSCAQCFModel_ValveON_ControlNormFactor)	delete	[] pTDSCAQCFModel_ValveON_ControlNormFactor;

	Init_SCAQCF_ValveON();
}
//-------------------------------------------------------------------
void M801::Clear_SCAQCF_ValveOFF()
{
	if (pTDSCAQCFModel_ValveOFF_Yeqx)					DeleteMatrix(pTDSCAQCFModel_ValveOFF_Yeqx);
	if (pTDSCAQCFModel_ValveOFF_Yequ)					DeleteMatrix(pTDSCAQCFModel_ValveOFF_Yequ);
	if (pTDSCAQCFModel_ValveOFF_Feqxx)					delete	[] pTDSCAQCFModel_ValveOFF_Feqxx;
	if (pTDSCAQCFModel_ValveOFF_Fequu)					delete	[] pTDSCAQCFModel_ValveOFF_Fequu;
	if (pTDSCAQCFModel_ValveOFF_Fequx)					delete	[] pTDSCAQCFModel_ValveOFF_Fequx;
	if (pTDSCAQCFModel_ValveOFF_Neqx)					DeleteMatrix(pTDSCAQCFModel_ValveOFF_Neqx);
	if (pTDSCAQCFModel_ValveOFF_Nequ)					DeleteMatrix(pTDSCAQCFModel_ValveOFF_Nequ);
	if (pTDSCAQCFModel_ValveOFF_Meq)					DeleteMatrix(pTDSCAQCFModel_ValveOFF_Meq);
	if (pTDSCAQCFModel_ValveOFF_Keq)					delete	[] pTDSCAQCFModel_ValveOFF_Keq;

	if (pTDSCAQCFModel_ValveOFF_NodeName)				delete	[] pTDSCAQCFModel_ValveOFF_NodeName;
	if (pTDSCAQCFModel_ValveOFF_StateNormFactor)		delete	[] pTDSCAQCFModel_ValveOFF_StateNormFactor;
	if (pTDSCAQCFModel_ValveOFF_ThroughNormFactor)		delete	[] pTDSCAQCFModel_ValveOFF_ThroughNormFactor;
	if (pTDSCAQCFModel_ValveOFF_ControlNormFactor)		delete	[] pTDSCAQCFModel_ValveOFF_ControlNormFactor;

	Init_SCAQCF_ValveOFF();
}
//-------------------------------------------------------------------
void M801::Init_SCAQCF_CAP()
{
	pTDSCAQCFModel_CAP_Yeqx					= NULL;					// pointer to SCAQCF CAP model linear state matrix
	pTDSCAQCFModel_CAP_Yequ					= NULL;					// pointer to SCAQCF CAP model linear control matrix
	pTDSCAQCFModel_CAP_Feqxx				= NULL;					// pointer to SCAQCF CAP model quadratic state matrix
	pTDSCAQCFModel_CAP_Fequu				= NULL;					// pointer to SCAQCF CAP model quadratic control matrix
	pTDSCAQCFModel_CAP_Fequx				= NULL;					// pointer to SCAQCF CAP model quadratic product of state and control matrix 
	pTDSCAQCFModel_CAP_Neqx					= NULL;					// pointer to SCAQCF CAP model linear past history state matrix 
	pTDSCAQCFModel_CAP_Nequ					= NULL;					// pointer to SCAQCF CAP model linear past history control matrix 
	pTDSCAQCFModel_CAP_Meq					= NULL;					// pointer to SCAQCF CAP model linear past history through viariable matrix
	pTDSCAQCFModel_CAP_Keq					= NULL;					// pointer to SCAQCF CAP model constant part
	pTDSCAQCFModel_CAP_NodeName				= NULL;					// pointer to SCAQCF CAP model connectivity vectors
	pTDSCAQCFModel_CAP_StateNormFactor		= NULL;					// pointer to SCAQCF CAP model state normalization factors
	pTDSCAQCFModel_CAP_ThroughNormFactor	= NULL;					// pointer to SCAQCF CAP model through variable normalization factors
	pTDSCAQCFModel_CAP_ControlNormFactor	= NULL;					// pointer to SCAQCF CAP model control normalization factors
}
//-------------------------------------------------------------------
void M801::Clear_SCAQCF_CAP()
{
	if (pTDSCAQCFModel_CAP_Yeqx)				DeleteMatrix(pTDSCAQCFModel_CAP_Yeqx);
	if (pTDSCAQCFModel_CAP_Yequ)				DeleteMatrix(pTDSCAQCFModel_CAP_Yequ);
	if (pTDSCAQCFModel_CAP_Feqxx)				delete	[] pTDSCAQCFModel_CAP_Feqxx;
	if (pTDSCAQCFModel_CAP_Fequu)				delete	[] pTDSCAQCFModel_CAP_Fequu;
	if (pTDSCAQCFModel_CAP_Fequx)				delete	[] pTDSCAQCFModel_CAP_Fequx;
	if (pTDSCAQCFModel_CAP_Neqx)				DeleteMatrix(pTDSCAQCFModel_CAP_Neqx);
	if (pTDSCAQCFModel_CAP_Nequ)				DeleteMatrix(pTDSCAQCFModel_CAP_Nequ);
	if (pTDSCAQCFModel_CAP_Meq)					DeleteMatrix(pTDSCAQCFModel_CAP_Meq);
	if (pTDSCAQCFModel_CAP_Keq)					delete	[] pTDSCAQCFModel_CAP_Keq;

	if (pTDSCAQCFModel_CAP_NodeName)			delete	[] pTDSCAQCFModel_CAP_NodeName;
	if (pTDSCAQCFModel_CAP_StateNormFactor)		delete	[] pTDSCAQCFModel_CAP_StateNormFactor;
	if (pTDSCAQCFModel_CAP_ThroughNormFactor)	delete	[] pTDSCAQCFModel_CAP_ThroughNormFactor;
	if (pTDSCAQCFModel_CAP_ControlNormFactor)	delete	[] pTDSCAQCFModel_CAP_ControlNormFactor;

	Init_SCAQCF_CAP();
}
//-------------------------------------------------------------------
BOOL M801::TQN_TimeDomainQDMModelInit()
{
	int i, j, i1, j1, iValve;

	TQN_ClearSCAQCF();

	nTDSCQDModel_Equ1 = 8;								// number of linear through equations
	nTDSCQDModel_Equ2 = 18;								// number of linear virtual equations
	nTDSCQDModel_Equ3 = 0;								// number of nonlinear equations
	nTDSCQDModel_State = 27;							// number of states in time domain quadratized model
	nTDSCQDModel_Control = 0;							// number of control variables in time domain quadratized model
	nTDSCQDModel_Feqxx = 0;								// number of quadratic terms for states
	nTDSCQDModel_Fequu = 0;								// number of quadratic terms for controls
	nTDSCQDModel_Fequx = 0;								// number of quadratic terms for the product of state and control variable

	if (!TQN_TimeDomainQuadratizedModelCreation()) return false;

	for ( iValve = 0 ; iValve < nValves ; iValve++ )
	{
		if( iValveStatusM801[iValve]==1 )
		{
			for ( i=0 ; i<2 ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<5 ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCQDModel_Yeqx1[i1][j1] += pTDSCQDM_ValveON_Yeqx1[i][j];
					pTDSCQDModel_Deqxd1[i1][j1] += pTDSCQDM_ValveON_Deqxd1[i][j];
				}
			}

			for ( i=2 ; i<5 ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<5 ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCQDModel_Yeqx2[i1-8][j1] += pTDSCQDM_ValveON_Yeqx2[i-2][j];
					pTDSCQDModel_Deqxd2[i1-8][j1] += pTDSCQDM_ValveON_Deqxd2[i-2][j];
				}
			}
		}
		else if ( iValveStatusM801[iValve]==0 )
		{
			for ( i=0 ; i<2 ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<5 ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCQDModel_Yeqx1[i1][j1] += pTDSCQDM_ValveOFF_Yeqx1[i][j];
					pTDSCQDModel_Deqxd1[i1][j1] += pTDSCQDM_ValveOFF_Deqxd1[i][j];
				}
			}

			for ( i=2 ; i<5 ; i++ )
			{
				i1 = iValvePointer[i][iValve];
				for ( j=0 ; j<5 ; j++ )
				{
					j1 = iValvePointer[j][iValve];
					pTDSCQDModel_Yeqx2[i1-8][j1] += pTDSCQDM_ValveOFF_Yeqx2[i-2][j];
					pTDSCQDModel_Deqxd2[i1-8][j1] += pTDSCQDM_ValveOFF_Deqxd2[i-2][j];
				}
			}
		}
	}

	//------Capacitor contributions

	pTDSCQDModel_Deqxd1[3][3] += pTDSCQDM_CAP_Deqxd1[0][0];
	pTDSCQDModel_Deqxd1[3][4] += pTDSCQDM_CAP_Deqxd1[0][1];
	pTDSCQDModel_Deqxd1[4][3] += pTDSCQDM_CAP_Deqxd1[1][0];
	pTDSCQDModel_Deqxd1[4][4] += pTDSCQDM_CAP_Deqxd1[1][1];

	//------Control signal terminals

	pTDSCQDModel_Yeqx1[5][5] = 1.0;			// zero crossing
	pTDSCQDModel_Yeqx1[6][6] = 1.0;			// Mag
	pTDSCQDModel_Yeqx1[7][7] = 1.0;			// Power


	return true;
}
#else
BOOL M801::TQN_TimeDomainModelInit()
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

	dRealPower801           = 0.0;
	dTimePeriod				= 1.0 / GetBaseFrequency();
	ddtsecsM801             = pNetSolver->dtsecs;        //----Time Interval 
	nBlockedCycles			= 1;
	dBlockedTime			= dTimePeriod * double(nBlockedCycles);
	
	dConverterRelativeTime			= 0.0;
	dConverterTimeRef			= 0.0;

	dPosSeqZeroTime			= 0.0;
	dPosSeqVMagn			= 0.0;
	dNetRealPower			= 0.0;

	dFiringAngle			= 0.0;
	Pre_FiringAngle			= 1.57;
	dTimeDelay				= 0.0;
	dDirectVoltage			= 0.0;
	dComutationTime 		= 0.0;
	dComutationCount		= 0.0;

	//------retrieve user defined parameters

	dValveONConductance		= dev_oparm[0];
	dValveOffConductance	= dev_oparm[1];
	dParasiticCapacitance	= dev_oparm[2];
	dSnubberCapacitance		= dev_oparm[3];
	dSnubberConductance		= 1.0 / dev_oparm[4];
	dCLConductance			= 1.0 / dev_oparm[5];
	dCLInductance			= dev_oparm[6];
	dSmoothingCapacitance	= dev_oparm[7];
	dRealPower801			= dev_oparm[10];
	Operating_mode_M801		= dev_oparm[13];


	//------allocate arrays

	TQN_AllocateLinearModelArrays();
	AllocateDataModel_801();

	//------initialize arrays

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		xi_real_tq[i]    = 0.0;
		xa_real_tq[i]    = 0.0;
		beq_real_tq[i]   = 0.0; 
		for ( j=0 ; j<devsta_tq ; j++ )
		{
			yeq_real_tq[i][j] = 0.0;
		}
	}
	
	//------initialize valve status

	for ( iValve = 0; iValve<nValves ; iValve++ )
	{
		iValveStatusPerModeM801[iValve]	= 0;
	}

	//------update converter normalized time and time reference

	if ( !TQN_UpdateConverterRunTimeRef_801() ) return false;

	//------define connectivity pointers

	if ( !TQN_ComputeModelPointers() ) return false;

	//------compute matrices Y P and Z for single valve and Capacitor

	if ( !TQN_ComputeValveAndCap_Y_PandZ_801() ) return false;

	//------compute converter matrices Y P and Z
	
	if ( !TQN_ComputeConverter_Y_PandZ_801() ) return false;

	//------compute overall converter ACF model 
	
	if ( !TQN_ComputeConverterFirstStepACF_801() ) return false;

	return true;
}
//-------------------------------------------------------------------
//	
//------this routine performs the time loop
//		computations for the model:
//		six-pulse converter
//
//-------------------------------------------------------------------
BOOL M801::TQN_TimeDomainModelTimeStep()
{
	int i;
	int j;
	int iValve;

	//------compute terminal currents

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		dx0 =  -beq_real_tq[i];
		for ( j=0 ; j<devsta_tq  ; j++ )
		{
			dx0 += yeq_real_tq[i][j] * xa_real_tq[j];
		} 
		xi_real_tq[i] = dx0;
	}

	//------perform diagnostic loop

	ndebg = 0;
	if ( ndebg == 1 )
	{
		dx0 = 0.0;
	  	for ( i=devnod-3 ; i<devsta_tq/2 ; i++ )
		{
			dx0 += fabs(xi_real_tq[i]);
		}
	  	if ( dx0 > 1.0e-6 )
		{
			amsr1.Format("NonZero InternalCurrents-Model 801: %10.8f",dx0); 
			DumpString(amsr1);
			AfxMessageBox(amsr1);
			return false;
		}
	}
	ndebg = 0;

	//------retrieve dPosSeqZeroTime and dPosSeqVMagn

	dPosSeqZeroTime		 = xa_real_tq[5];
	dPosSeqVMagn		 = xa_real_tq[6];
	dNetRealPower		 = xa_real_tq[7];


	//------prepare control action

	dNoLoadDirectVoltage = ((3.0*sqrt(2.0))/DPI)*dPosSeqVMagn;
	dDirectCurrent       = xi_real_tq[4];
	dDirectVoltage		 = (xa_real_tq[3]-xa_real_tq[4]);



	//--------transient simulation by changing power order

	if( pNetSolver->run_time > 4.0 ) dRealPower801=250.0;


	//------update converter normalized time and time reference

	if ( !TQN_UpdateConverterRunTimeRef_801() ) return false;

	//------Determine if run time is within the blocked cycles
	//		bConverterON = 1, means it is still within the blocked cycles
	//		and no need to compute valve status

	iAnySwitching = 0;
	bConverterON = 1;
	if ( (pNetSolver->run_time - dBlockedTime) > 0.0 ) bConverterON = 0;
	
	if ( bConverterON == 0 ) 
	{
		//---------- Operating mode is converter

		if ( Operating_mode_M801 == 1 )
		{
			//------Compute firing anlge

			if ( !TQN_CalculateFiringAngle_801() ) return false;

			//------Compute Valve ScheduleTime

			if ( !TQN_ComputeValveScheduleTime_801() ) return false;

			//------Compute Valve Switching time for each valve

			if ( !TQN_ComputeValveSwitchingTimes_801() ) return false;
		}
		
		//----------- Operating mode is inverter

		else if ( Operating_mode_M801 == 2 )
		{	
			//------Compute firing anlge

			if(!TQN_CalculateFiringAngleINV_801()) return false;

			//------Compute Valve ScheduleTime

			if ( !TQN_ComputeValveScheduleTime_801() ) return false;

			//------Compute Valve Switching time for each valve

			if ( !TQN_ComputeValveSwitchingTimesINV_801() ) return false;

		}

		//------Compute Valve ON / OFF status

		if ( !TQN_TimeUpdateValveStatus_801()) return false;

		//------compute converter matrices Y, P, and Z

		if ( iAnySwitching == 1)
		{
			if ( !TQN_ComputeConverter_Y_PandZ_801() ) return false;

			pNetSolver->bUpdateLnSystemFlag = true;
		}
	}

	//------compute overall converter ACF model for initialization, use fixed time step scheme

	if ( !TQN_ComputeConverterFirstStepACF_801() ) return false;

	//------do get Data all parameters and all matrix for M801

	ndebg = 0;
	if ( ndebg == 1 )
	{
//		TQN_DebugReportModel801();
		TQN_DebugMatrixReportModel801();
//		DumpDoubleScalar("Running Time=",pNetSolver->run_time);
	}
	ndebg = 0;

	return true;

}

//-------------------------------------------------------------------
BOOL M801::TQN_UpdateConverterRunTimeRef_801()
{
	int iValve;

	dConverterRelativeTime = pNetSolver->run_time - dConverterTimeRef;

	if ( dConverterRelativeTime > dTimePeriod )
	{
		dConverterTimeRef	+= dTimePeriod;
		dConverterRelativeTime	-= dTimePeriod;
	}

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeModelPointers()
{

	//------define valve pointers

	iValvePointer[0][0] =  0;
	iValvePointer[1][0] =  3;
	iValvePointer[2][0] =  8;
	iValvePointer[3][0] =  9;
	iValvePointer[4][0] = 10;
	iValvePointer[5][0] = 28;
	iValvePointer[6][0] = 31;
	iValvePointer[7][0] = 36;
	iValvePointer[8][0] = 37;
	iValvePointer[9][0] = 38;

	iValvePointer[0][1] =  4;
	iValvePointer[1][1] =  2;
	iValvePointer[2][1] = 11;
	iValvePointer[3][1] = 12;
	iValvePointer[4][1] = 13;
	iValvePointer[5][1] = 32;
	iValvePointer[6][1] = 30;
	iValvePointer[7][1] = 39;
	iValvePointer[8][1] = 40;
	iValvePointer[9][1] = 41;

	iValvePointer[0][2] =  1;
	iValvePointer[1][2] =  3;
	iValvePointer[2][2] = 14;
	iValvePointer[3][2] = 15;
	iValvePointer[4][2] = 16;
	iValvePointer[5][2] = 29;
	iValvePointer[6][2] = 31;
	iValvePointer[7][2] = 42;
	iValvePointer[8][2] = 43;
	iValvePointer[9][2] = 44;

	iValvePointer[0][3] =  4;
	iValvePointer[1][3] =  0;
	iValvePointer[2][3] = 17;
	iValvePointer[3][3] = 18;
	iValvePointer[4][3] = 19;
	iValvePointer[5][3] = 32;
	iValvePointer[6][3] = 28;
	iValvePointer[7][3] = 45;
	iValvePointer[8][3] = 46;
	iValvePointer[9][3] = 47;

	iValvePointer[0][4] =  2;
	iValvePointer[1][4] =  3;
	iValvePointer[2][4] = 20;
	iValvePointer[3][4] = 21;
	iValvePointer[4][4] = 22;
	iValvePointer[5][4] = 30;
	iValvePointer[6][4] = 31;
	iValvePointer[7][4] = 48;
	iValvePointer[8][4] = 49;
	iValvePointer[9][4] = 50;

	iValvePointer[0][5] =  4;
	iValvePointer[1][5] =  1;
	iValvePointer[2][5] = 23;
	iValvePointer[3][5] = 24;
	iValvePointer[4][5] = 25;
	iValvePointer[5][5] = 32;
	iValvePointer[6][5] = 29;
	iValvePointer[7][5] = 51;
	iValvePointer[8][5] = 52;
	iValvePointer[9][5] = 53;

	//------define capacitor pointers

	iCapPointerM801_tq[0] = 3;
	iCapPointerM801_tq[1] = 4;
	iCapPointerM801_tq[2] = 26;
	iCapPointerM801_tq[3] = 27;
	iCapPointerM801_tq[4] = 31;
	iCapPointerM801_tq[5] = 32;
	iCapPointerM801_tq[6] = 54;
	iCapPointerM801_tq[7] = 55;

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure Computes the model matrices Y ,P & Z 
//		for a single valve and for the capacitor model 
//		by using elementary Row operation.
//		The results are stored in arrays:
//			dYmatrixSvalve, 
//			dPmatrixSvalve, and
//          dZmatrixSvalve,
//			dYmatrixCap,
//			dPmatrixCap, and
//			dZmatrixCap
//
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeValveAndCap_Y_PandZ_801()
{
	int i;
	int j;

	//------ capacitor model for Algebraic Companion Form

	//------ initialize for Yeq and Peq of single valve

	for( i=0 ; i<nCapTerm_tq ; i++ )
	{ 
		for( j=0 ; j<nCapTerm_tq ; j++ )
		{	
			dYmatrixCap[i][j] = 0.0;
		}
	}
	for(i=0 ; i<nCapTerm_tq ; i++ )
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


	dYmatrixCap[2][2] =   1.0;
	dYmatrixCap[2][3] =  -( ddtsecsM801/6.0 );
	dYmatrixCap[2][7] =  -(2.0 * ddtsecsM801) / 3.0;


	dYmatrixCap[3][0] =  -1.0;
	dYmatrixCap[3][1] =   1.0;
	dYmatrixCap[3][2] =   1.0;

	dYmatrixCap[4][7] =   dSmoothingCapacitance;

	dYmatrixCap[5][7] =  -dSmoothingCapacitance;

	dYmatrixCap[6][3] =   ( ddtsecsM801/24.0 );
	dYmatrixCap[6][6] =   1.0;
	dYmatrixCap[6][7] =  -( ddtsecsM801/3.0 );

	dYmatrixCap[7][4] =  -1.0;
	dYmatrixCap[7][5] =   1.0;
	dYmatrixCap[7][6] =   1.0;


//-------------------- set up dPmatrixCap----------------------------------------

	dPmatrixCap[2][2] =  1.0;
	dPmatrixCap[2][3] =  ( ddtsecsM801/6.0 );

	dPmatrixCap[6][2] =  1.0;
	dPmatrixCap[6][3] =  ( (5.0*ddtsecsM801)/24.0 );


//------ single valve model for Algebraic Companion Form -------------------------

	for(i = 0 ; i<nValTerm_tq ; i++ )
	{
		for(j = 0 ; j<nValTerm_tq ; j++ )
		{
			dYmatrixSvalveON[i][j]=0.0;
			dYmatrixSvalveOFF[i][j]=0.0;

		}
	}
	for(i = 0 ; i<nValTerm_tq ; i++ )
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
	dYmatrixSvalveON[2][1] = -( ddtsecsM801 / 6.0 ) * dSnubberConductance;
	dYmatrixSvalveON[2][2] =  ( ddtsecsM801 / 6.0 ) * dSnubberConductance + dSnubberCapacitance;
	dYmatrixSvalveON[2][6] = -( 2.0*ddtsecsM801 / 3.0) * dSnubberConductance;
	dYmatrixSvalveON[2][7] =  ( 2.0*ddtsecsM801 / 3.0) * dSnubberConductance;

	dYmatrixSvalveON[3][0] = -( ddtsecsM801 / 6.0 ) * dCLConductance;
	dYmatrixSvalveON[3][1] = -( ddtsecsM801 / 6.0 ) * dValveONConductance - dParasiticCapacitance;
	dYmatrixSvalveON[3][3] =  ( ddtsecsM801 / 6.0 ) * dValveONConductance + ( ddtsecsM801 / 6.0 ) * dCLConductance + dParasiticCapacitance;
	dYmatrixSvalveON[3][4] = -( ddtsecsM801 / 6.0 );
	dYmatrixSvalveON[3][5] = -( 2.0*ddtsecsM801 / 3.0) * dCLConductance;
	dYmatrixSvalveON[3][6] = -( 2.0*ddtsecsM801 / 3.0) * dValveONConductance;
	dYmatrixSvalveON[3][8] =  ( 2.0*ddtsecsM801 / 3.0) * dCLConductance + ( 2.0*ddtsecsM801 / 3.0) * dValveONConductance;
	dYmatrixSvalveON[3][9] = -( 2.0*ddtsecsM801 / 3.0);

	dYmatrixSvalveON[4][0] = -( ddtsecsM801 / 6.0 );
	dYmatrixSvalveON[4][3] =  ( ddtsecsM801 / 6.0 );
	dYmatrixSvalveON[4][4] =  dCLInductance;
	dYmatrixSvalveON[4][5] = -( 2.0*ddtsecsM801 / 3.0);
	dYmatrixSvalveON[4][8] =  ( 2.0*ddtsecsM801 / 3.0);

	dYmatrixSvalveON[5][5] =  dCLConductance;
	dYmatrixSvalveON[5][6] = -dSnubberConductance;
	dYmatrixSvalveON[5][7] =  dSnubberConductance;
	dYmatrixSvalveON[5][8] = -dCLConductance;
	dYmatrixSvalveON[5][9] =  1.0;

	dYmatrixSvalveON[6][5] = -dCLConductance;
	dYmatrixSvalveON[6][6] =  dSnubberConductance;
	dYmatrixSvalveON[6][7] = -dSnubberConductance;
	dYmatrixSvalveON[6][8] =  dCLConductance;
	dYmatrixSvalveON[6][9] = -1.0;

	dYmatrixSvalveON[7][1] =  ( ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dYmatrixSvalveON[7][2] = -( ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dYmatrixSvalveON[7][5] = -dSnubberCapacitance; 
	dYmatrixSvalveON[7][6] = -( ddtsecsM801 / 3.0 ) * dSnubberConductance; 
	dYmatrixSvalveON[7][7] =  ( ddtsecsM801 / 3.0 ) * dSnubberConductance + dSnubberCapacitance;

	dYmatrixSvalveON[8][0] =  ( ddtsecsM801 / 24.0 ) * dCLConductance;
	dYmatrixSvalveON[8][1] =  ( ddtsecsM801 / 24.0 ) * dValveONConductance;
	dYmatrixSvalveON[8][3] = -( ddtsecsM801 / 24.0 ) * dCLConductance - ( ddtsecsM801 / 24.0 ) * dValveONConductance;
	dYmatrixSvalveON[8][4] =  ( ddtsecsM801 / 24.0 );
	dYmatrixSvalveON[8][5] = -( ddtsecsM801 / 3.0 ) * dCLConductance;
	dYmatrixSvalveON[8][6] = -( ddtsecsM801 / 3.0 ) * dValveONConductance-dParasiticCapacitance;
	dYmatrixSvalveON[8][8] =  ( ddtsecsM801 / 3.0 ) * dCLConductance + ( ddtsecsM801 / 3.0 ) * dValveONConductance+dParasiticCapacitance;
	dYmatrixSvalveON[8][9] = -( ddtsecsM801 / 3.0 );

	dYmatrixSvalveON[9][0] =  ( ddtsecsM801 / 24.0 );
	dYmatrixSvalveON[9][3] = -( ddtsecsM801 / 24.0 );
	dYmatrixSvalveON[9][5] = -( ddtsecsM801 / 3.0 );
	dYmatrixSvalveON[9][8] =  ( ddtsecsM801 / 3.0 );
	dYmatrixSvalveON[9][9] =  dCLInductance;

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
	dYmatrixSvalveOFF[2][1] = -( ddtsecsM801 / 6.0 ) * dSnubberConductance;
	dYmatrixSvalveOFF[2][2] =  ( ddtsecsM801 / 6.0 ) * dSnubberConductance + dSnubberCapacitance;
	dYmatrixSvalveOFF[2][6] = -( 2.0*ddtsecsM801 / 3.0) * dSnubberConductance;
	dYmatrixSvalveOFF[2][7] =  ( 2.0*ddtsecsM801 / 3.0) * dSnubberConductance;

	dYmatrixSvalveOFF[3][0] = -( ddtsecsM801 / 6.0 ) * dCLConductance;
	dYmatrixSvalveOFF[3][1] = -( ddtsecsM801 / 6.0 ) * dValveOffConductance - dParasiticCapacitance;
	dYmatrixSvalveOFF[3][3] =  ( ddtsecsM801 / 6.0 ) * dValveOffConductance + ( ddtsecsM801 / 6.0 ) * dCLConductance + dParasiticCapacitance;
	dYmatrixSvalveOFF[3][4] = -( ddtsecsM801 / 6.0 );
	dYmatrixSvalveOFF[3][5] = -( 2.0*ddtsecsM801 / 3.0) * dCLConductance;
	dYmatrixSvalveOFF[3][6] = -( 2.0*ddtsecsM801 / 3.0) * dValveOffConductance;
	dYmatrixSvalveOFF[3][8] =  ( 2.0*ddtsecsM801 / 3.0) * dCLConductance + ( 2.0*ddtsecsM801 / 3.0) * dValveOffConductance;
	dYmatrixSvalveOFF[3][9] = -( 2.0*ddtsecsM801 / 3.0);

	dYmatrixSvalveOFF[4][0] = -( ddtsecsM801 / 6.0 );
	dYmatrixSvalveOFF[4][3] =  ( ddtsecsM801 / 6.0 );
	dYmatrixSvalveOFF[4][4] =  dCLInductance;
	dYmatrixSvalveOFF[4][5] = -( 2.0*ddtsecsM801 / 3.0);
	dYmatrixSvalveOFF[4][8] =  ( 2.0*ddtsecsM801 / 3.0);

	dYmatrixSvalveOFF[5][5] =  dCLConductance;
	dYmatrixSvalveOFF[5][6] = -dSnubberConductance;
	dYmatrixSvalveOFF[5][7] =  dSnubberConductance;
	dYmatrixSvalveOFF[5][8] = -dCLConductance;
	dYmatrixSvalveOFF[5][9] =  1.0;

	dYmatrixSvalveOFF[6][5] = -dCLConductance;
	dYmatrixSvalveOFF[6][6] =  dSnubberConductance;
	dYmatrixSvalveOFF[6][7] = -dSnubberConductance;
	dYmatrixSvalveOFF[6][8] =  dCLConductance;
	dYmatrixSvalveOFF[6][9] = -1.0;

	dYmatrixSvalveOFF[7][1] =  ( ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dYmatrixSvalveOFF[7][2] = -( ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dYmatrixSvalveOFF[7][5] = -dSnubberCapacitance; 
	dYmatrixSvalveOFF[7][6] = -( ddtsecsM801 / 3.0 ) * dSnubberConductance; 
	dYmatrixSvalveOFF[7][7] =  ( ddtsecsM801 / 3.0 ) * dSnubberConductance + dSnubberCapacitance;

	dYmatrixSvalveOFF[8][0] =  ( ddtsecsM801 / 24.0 ) * dCLConductance;
	dYmatrixSvalveOFF[8][1] =  ( ddtsecsM801 / 24.0 ) * dValveOffConductance;
	dYmatrixSvalveOFF[8][3] = -( ddtsecsM801 / 24.0 ) * dCLConductance - ( ddtsecsM801 / 24.0 ) * dValveOffConductance;
	dYmatrixSvalveOFF[8][4] =  ( ddtsecsM801 / 24.0 );
	dYmatrixSvalveOFF[8][5] = -( ddtsecsM801 / 3.0 ) * dCLConductance;
	dYmatrixSvalveOFF[8][6] = -( ddtsecsM801 / 3.0 ) * dValveOffConductance-dParasiticCapacitance;
	dYmatrixSvalveOFF[8][8] =  ( ddtsecsM801 / 3.0 ) * dCLConductance + ( ddtsecsM801 / 3.0 ) * dValveOffConductance+dParasiticCapacitance;
	dYmatrixSvalveOFF[8][9] = -( ddtsecsM801 / 3.0 );

	dYmatrixSvalveOFF[9][0] =  ( ddtsecsM801 / 24.0 );
	dYmatrixSvalveOFF[9][3] = -( ddtsecsM801 / 24.0 );
	dYmatrixSvalveOFF[9][5] = -( ddtsecsM801 / 3.0 );
	dYmatrixSvalveOFF[9][8] =  ( ddtsecsM801 / 3.0 );
	dYmatrixSvalveOFF[9][9] =  dCLInductance;

//-------------dPmatrixSvalve during valve on----------------------------



	dPmatrixSvalveON[2][0] = -dSnubberCapacitance;
	dPmatrixSvalveON[2][1] =  ( ddtsecsM801 / 6.0 ) * dSnubberConductance;
	dPmatrixSvalveON[2][2] = -( ddtsecsM801 / 6.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveON[3][0] =  ( ddtsecsM801 / 6.0 ) * dCLConductance;
	dPmatrixSvalveON[3][1] =  ( ddtsecsM801 / 6.0 ) * dValveONConductance - dParasiticCapacitance;	
	dPmatrixSvalveON[3][3] = -( ddtsecsM801 / 6.0 ) * dCLConductance - ( ddtsecsM801 / 6.0 ) * dValveONConductance + dParasiticCapacitance;
	dPmatrixSvalveON[3][4] =  ( ddtsecsM801 / 6.0 );

	dPmatrixSvalveON[4][0] =  ( ddtsecsM801 / 6.0 );
	dPmatrixSvalveON[4][3] = -( ddtsecsM801 / 6.0 );
	dPmatrixSvalveON[4][4] =  dCLInductance;

	dPmatrixSvalveON[7][0] = -dSnubberCapacitance;
	dPmatrixSvalveON[7][1] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dPmatrixSvalveON[7][2] = -( 5.0 * ddtsecsM801 / 24.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveON[8][0] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dCLConductance;
	dPmatrixSvalveON[8][1] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dValveONConductance - dParasiticCapacitance;	
	dPmatrixSvalveON[8][3] = -( 5.0 * ddtsecsM801 / 24.0 ) * dCLConductance - ( 5.0 * ddtsecsM801 / 24.0 ) * dValveONConductance + dParasiticCapacitance;
	dPmatrixSvalveON[8][4] =  ( 5.0 * ddtsecsM801 / 24.0 );

	dPmatrixSvalveON[9][0] =  ( 5.0*ddtsecsM801 / 24.0 );
	dPmatrixSvalveON[9][3] = -( 5.0*ddtsecsM801 / 24.0 );
	dPmatrixSvalveON[9][4] =  dCLInductance;

//-------------dPmatrixSvalve during valve off---------------------------

	dPmatrixSvalveOFF[2][0] = -dSnubberCapacitance;
	dPmatrixSvalveOFF[2][1] =  ( ddtsecsM801 / 6.0 ) * dSnubberConductance;
	dPmatrixSvalveOFF[2][2] = -( ddtsecsM801 / 6.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveOFF[3][0] =  ( ddtsecsM801 / 6.0 ) * dCLConductance;
	dPmatrixSvalveOFF[3][1] =  ( ddtsecsM801 / 6.0 ) * dValveOffConductance - dParasiticCapacitance;	
	dPmatrixSvalveOFF[3][3] = -( ddtsecsM801 / 6.0 ) * dCLConductance - ( ddtsecsM801 / 6.0 ) * dValveOffConductance + dParasiticCapacitance;
	dPmatrixSvalveOFF[3][4] =  ( ddtsecsM801 / 6.0 );

	dPmatrixSvalveOFF[4][0] =  ( ddtsecsM801 / 6.0 );
	dPmatrixSvalveOFF[4][3] = -( ddtsecsM801 / 6.0 );
	dPmatrixSvalveOFF[4][4] =  dCLInductance;

	dPmatrixSvalveOFF[7][0] = -dSnubberCapacitance;
	dPmatrixSvalveOFF[7][1] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dSnubberConductance;
	dPmatrixSvalveOFF[7][2] = -( 5.0 * ddtsecsM801 / 24.0 ) * dSnubberConductance + dSnubberCapacitance;

	dPmatrixSvalveOFF[8][0] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dCLConductance;
	dPmatrixSvalveOFF[8][1] =  ( 5.0 * ddtsecsM801 / 24.0 ) * dValveOffConductance - dParasiticCapacitance;	
	dPmatrixSvalveOFF[8][3] = -( 5.0 * ddtsecsM801 / 24.0 ) * dCLConductance - ( 5.0 * ddtsecsM801 / 24.0 ) * dValveOffConductance + dParasiticCapacitance;
	dPmatrixSvalveOFF[8][4] =  ( 5.0 * ddtsecsM801 / 24.0 );

	dPmatrixSvalveOFF[9][0] =  ( 5.0*ddtsecsM801 / 24.0 );
	dPmatrixSvalveOFF[9][3] = -( 5.0*ddtsecsM801 / 24.0 );
	dPmatrixSvalveOFF[9][4] =  dCLInductance;


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
BOOL M801::TQN_CalculateFiringAngle_801()
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

	dVoltageRef = dRealPower801 / dDirectCurrent;

	//--------Compare reference voltage with no load direct voltage
	
	if( dVoltageRef >= dNoLoadDirectVoltage || dDirectCurrent <= 0.0) dVoltageRef = dNoLoadDirectVoltage;
	//--------Calculation of deviation of output power

	dDeviationOfDCVoltage	= dVoltageRef - dDirectVoltage;

	dDeviationOfFiringangle = - dDeviationOfDCVoltage/ ( dNoLoadDirectVoltage *sin(	Pre_FiringAngle));

	//--------Calculation of firing angle deviation

	dFilteredDeviationOfFiringangle= dPropositionalConstant * dDeviationOfFiringangle;

	//--------Calculation of New firing angle

	dFiringAngle	= (	Pre_FiringAngle + dFilteredDeviationOfFiringangle);


	//------compute time delay from zero crosing of Line-Line voltage

	if( dFiringAngle <= 10.0 / 180.0 * DPI )
	{
		dFiringAngle = 10.0 / 180.0 * DPI;
	}
	else if( dFiringAngle >= 85.0 / 180.0 * DPI ) dFiringAngle = 85.0 / 180.0 * DPI;

	//DumpDoubleScalar("Conv_Firing angle: ",Pre_FiringAngle);
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
BOOL M801::TQN_CalculateFiringAngleINV_801()
{	
	double dVoltageRef;
	double dDeviationOfDCVoltage;
	double dDeviationOfExtinct;
	double dFilteredDeviationOfFiringangle;
	double dPropositionalConstant;
	double NoloadVoltageRef;
	double dExtinctionAngleM801;
	double dComutationAngle;
	double dLimitationAngle;
	double dCalculationDCvolt;
	double dAngleofAdvance;

	dDeviationOfDCVoltage	= 0.0;
	dDeviationOfExtinct = 0.0;
	dPropositionalConstant  = 0.01;

	//--------Calculation of reference voltage of DC side
	dVoltageRef =  dRealPower801 / dDirectCurrent;
		

	//--------Compare reference voltage with no load direct voltage

	if( dVoltageRef >= dNoLoadDirectVoltage || dDirectCurrent <= 0.0) dVoltageRef = dNoLoadDirectVoltage;

	//--------Calculation of deviation of output power

	dDeviationOfDCVoltage	= -dVoltageRef - dDirectVoltage;
	

	dComutationAngle = ( dComutationTime / dTimePeriod ) * TWODPI;

	//--------Calculation of present extinction angle

	dExtinctionAngleM801	=	( DPI-Pre_FiringAngle -	dComutationAngle );
	dAngleofAdvance			=	( DPI-Pre_FiringAngle );
	dCalculationDCvolt		=	( dNoLoadDirectVoltage / 2 ) * ( cos( dExtinctionAngleM801 ) + cos( dAngleofAdvance ) ) ;
	
	dx1 = dDirectVoltage + dCalculationDCvolt;

	if(dDeviationOfDCVoltage>=-5.0)
	{
 			dDeviationOfDCVoltage	= -dCalculationDCvolt - dDirectVoltage;
	}

	dDeviationOfExtinct =   dDeviationOfDCVoltage/ ( dNoLoadDirectVoltage *sin(dExtinctionAngleM801));
	 
	//--------Calculation of firing angle deviation

	dFilteredDeviationOfFiringangle = dPropositionalConstant * dDeviationOfExtinct;

	//--------Calculation of New firing angle

	dFiringAngle		=  ( DPI - dExtinctionAngleM801 - dFilteredDeviationOfFiringangle -dComutationAngle );
	
	//if (abs( dfiringangle - pre_firingangle <= 0.001) ) 
	//{
	//	dfiringangle = pre_firingangle;
	//}
	//------compute time delay from zero crosing of Line-Line voltage
	
	dLimitationAngle = DPI - dComutationAngle - ( 20.0 * DPI ) / 180.0;
	if( dFiringAngle <= 95.0 / 180.0 * DPI ) dFiringAngle = 95.0 / 180.0 * DPI;
	if ( dFiringAngle >= dLimitationAngle ) 
	{
		dFiringAngle = dLimitationAngle ;
	}
		
//	DumpDoubleScalar("running time: ",pNetSolver->run_time);
	DumpDoubleScalar("INV_Firing angle: ",Pre_FiringAngle);

	return true;

}

//-------------------------------------------------------------------
//
//		This routine computes the scheduled ON time
//			dValve1TurnOnTime for the first valve.
//		in addition, it computes the scheduled ON time
//			dValveTurnONTimes[] for all the valves
//
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeValveScheduleTime_801()
{
	int iValve;

	dTimeDelay	=( dFiringAngle / TWODPI ) * dTimePeriod;
	
	//------update firing time signals

	dValve1TurnOnTime = dPosSeqZeroTime + dTimeDelay;

	dx0 = dTimePeriod / 6.0;
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveTurnONTimes[iValve] = dValve1TurnOnTime + dx0 * double(iValve+1);
	}

	//------adjust to have only positive values

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx1 = dValveTurnONTimes[iValve] - dConverterTimeRef;
		if ( dx1 <= 0.0 )
		{
			dValveTurnONTimes[iValve] += dTimePeriod;
		}
		if ( dx1 > dTimePeriod )
		{
			dValveTurnONTimes[iValve] -= dTimePeriod;
		}
	}
		
	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx1=dValveTurnONTimes[iValve]- pNetSolver->run_time ;

		if( dx1 <= -0.000001 )
		{
			dValveTurnONTimes[iValve] += dTimePeriod;
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
//			dValveSwitchTimeM801
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
BOOL M801::TQN_ComputeValveSwitchingTimes_801()
{
	int iValve;
	int k1;
	int k2;


//------initialize dValveSwitchTimeM801 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveSwitchTimeM801[iValve] = -1.0;
	}

//------compute dValveSwitchTimeM801 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{

//------compute dValveSwitchTimeM801 for ON -> OFF switching in [t-h,t]

		if ( iValveStatusPerModeM801[iValve] == 1 )
		{
			k1=iValvePointer[1][iValve];
			k2=iValvePointer[3][iValve];
			dx0 = xa_real_tq[k2] - xa_real_tq[k1];
			if ( dx0 < 0.0 )
			{
				dValveSwitchTimeM801[iValve] = 0.001 * ddtsecsM801;
			}
		}

//------compute dValveSwitchTimeM801 for OFF -> ON switching in [t-h,t]

		if ( iValveStatusPerModeM801[iValve] == 0 )
		{
			dx0 = dValveTurnONTimes[iValve] - (pNetSolver->run_time + ddtsecsM801 ) ;

			if  ( dx0 < 0.0)
				{
					k1=iValvePointer[1][iValve];
					k2=iValvePointer[3][iValve];
					dx1 = xa_real_tq[k2] - xa_real_tq[k1];
					if ( dx1 >= 0.0 )
					{
						dValveSwitchTimeM801[iValve]  = 0.1*ddtsecsM801;
						Pre_FiringAngle    =  dFiringAngle;
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
//			dValveSwitchTimeM801
//
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeValveSwitchingTimesINV_801()
{
	int iValve;
	int k1;
	int k2;

//------initialize dValveSwitchTimeM801 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dValveSwitchTimeM801[iValve] = -1.0;
	}

//------compute dValveSwitchTimeM801 

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{

//------compute dValveSwitchTimeM801 for ON -> OFF switching in [t-h,t]
		if ( iValveStatusPerModeM801[iValve] == 1 )
		{
			k1=iValvePointer[1][iValve];
			k2=iValvePointer[3][iValve];
			dx0 = xa_real_tq[k2] - xa_real_tq[k1];

			if ( dx0 < 0.0 )
			{
				dValveSwitchTimeM801[iValve] = 0.001 * ddtsecsM801;
				if( iValve == 0 )
				{
					if( iValveStatusPerModeM801[2] ==1 ) 
					{
						dComutationTime = pNetSolver->run_time - dComutationCount;
					}
				}
			}
		}

//------compute dValveSwitchTimeM801 for OFF -> ON switching in [t-h,t]

		if ( iValveStatusPerModeM801[iValve] == 0 )
		{
			dx0 = dValveTurnONTimes[iValve] - (pNetSolver->run_time + ddtsecsM801 ) ;

			if  ( dx0 < 0.0)
				{
					k1=iValvePointer[1][iValve];
					k2=iValvePointer[3][iValve];
					dx1 = xa_real_tq[k2] - xa_real_tq[k1];	


					if ( dx1 >= 0.0 )
					{
						dValveSwitchTimeM801[iValve]  = 0.1*ddtsecsM801;
						Pre_FiringAngle    =  dFiringAngle;  
						if (dFiringAngle < Pre_FiringAngle) Pre_FiringAngle    =  dFiringAngle;
						else Pre_FiringAngle    =  dFiringAngle;
						if( iValve == 2) dComutationCount		=	 pNetSolver->run_time + ddtsecsM801;
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
//		This routine updates the Valve Status.
//		if Valve state is ON,  iValveStatusPerModeM801 = 1
//		if Valve state is OFF, iValveStatusPerModeM801 = 0
//
//		A valve is turned off if the voltage bias goes negative
//		A valve is turned on with a equi-distant control
//
//-------------------------------------------------------------------
BOOL M801::TQN_TimeUpdateValveStatus_801()
{
	int iValve;
	int	iValve1;

	for ( iValve=0 ; iValve<nValves ; iValve++ )
	{
		dx0 = dValveSwitchTimeM801[iValve];
		if ( dx0 >= 0.0 )
		{
			if ( iValveStatusPerModeM801[iValve] == 0 )
			{
				iValveStatusPerModeM801[iValve] = 1;
				iAnySwitching = 1;
			}
			else if ( iValveStatusPerModeM801[iValve] == 1 )
			{
				iValveStatusPerModeM801[iValve] = 0;
				iAnySwitching = 1;

			}
		}
	}

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the model 801 matrices Y,P & Z 
//		(entire converter). The model is stored in arrays
//			dYmatrixConverter,
//			dPmatrixConverter, and							
//			dZmatrixConverter
//
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeConverter_Y_PandZ_801()
{
	int k;
	int m;
	int	i;
	int	i1;
	int	j;
	int	j1;
	int	iValve;
	
//---------initialize Y,P, and Z for converter

	for ( i=0 ; i<nValTerm_tq ; i++ )
	{
		for ( j=0 ; j<nValTerm_tq; j++ )
		{
			dYmatrixSvalve[i][j] = 0.0;
		}
	}
	for ( i=0 ; i<nValTerm_tq ; i++ )
	{
		for ( j=0 ; j<nValTerm; j++ )
		{
			dPmatrixSvalve[i][j] = 0.0;
		}
	}

	for ( i=0 ; i<devsta_tq ; i++ )
	{
		for ( j=0 ; j<devsta_tq; j++ )
		{
			dYmatrixConverter[i][j] = 0.0;
		}
	}
		
	for ( i=0 ; i<devsta_tq ; i++ )
	{
		for ( j=0 ; j<devsta; j++ )
		{
			dPmatrixConverter[i][j] = 0.0;
			dZmatrixConverter[i][j] = 0.0;
		}
	}

	//------Form matrix by summing up valve contributions

	for ( iValve = 0 ; iValve < nValves ; iValve++ )
	{
		if(iValveStatusPerModeM801[iValve]==1)
		{
			for ( i = 0 ; i < nValTerm_tq; i++ )
			{
				for ( j = 0 ; j < nValTerm_tq; j++ )
				{
					dYmatrixSvalve[i][j] = dYmatrixSvalveON[i][j]; 
				}
					
			}

			for ( i = 0 ; i < nValTerm_tq; i++ )
			{
				for ( j = 0 ; j < nValTerm; j++ )
				{
					dPmatrixSvalve[i][j] = dPmatrixSvalveON[i][j]; 
				}
					
			}
		}
		else if ( iValveStatusPerModeM801[iValve]==0 )
		{
			for ( i = 0 ; i < nValTerm_tq; i++ )
			{
				for ( j = 0 ; j < nValTerm_tq; j++ )
				{
					dYmatrixSvalve[i][j] = dYmatrixSvalveOFF[i][j]; 
				}
			}

			for ( i = 0 ; i < nValTerm_tq; i++ )
			{
				for ( j=0 ; j < nValTerm; j++ )
				{
					dPmatrixSvalve[i][j] = dPmatrixSvalveOFF[i][j]; 
				}
			}
		}

//--------Compute Y_P_and Z for a converter by using pointer

		for ( k = 0 ; k < nValTerm_tq ; k++ )
		{
			i1 = iValvePointer[k][iValve];
			for ( m = 0 ; m < nValTerm_tq ; m++ )
			{
				j1 = iValvePointer[m][iValve];
				dYmatrixConverter[i1][j1] += dYmatrixSvalve[k][m];
			}
		}

		for ( k = 0 ; k < nValTerm_tq; k++ )
		{
			i1 = iValvePointer[k][iValve];
			for ( m = 0 ; m < nValTerm; m++ )
			{
				j1 = iValvePointer[m][iValve];
				dPmatrixConverter[i1][j1] += dPmatrixSvalve[k][m];
				dZmatrixConverter[i1][j1] += dZmatrixSvalve[k][m];
			}
		}
	}
	
	//------capacitor contributions

	for ( i=0 ; i<nCapTerm_tq ; i++ )
	{
		i1 = iCapPointerM801_tq[i];
		for ( j=0 ; j<nCapTerm_tq ; j++ )
		{
			j1 = iCapPointerM801_tq[j];
			dYmatrixConverter[i1][j1] += dYmatrixCap[i][j];
		}
	}
		
	for ( i=0 ; i<nCapTerm_tq ; i++ )
	{
		i1 = iCapPointerM801_tq[i];
		for ( j=0 ; j<nCapTerm ; j++ )
		{
			j1 = iCapPointerM801_tq[j];
			dPmatrixConverter[i1][j1] += dPmatrixCap[i][j];
			dZmatrixConverter[i1][j1] += dZmatrixCap[i][j];
		}
	}

	return true;

}
//-------------------------------------------------------------------
//
//------This procedure computes the ACF model 
//		of entire converter for an time 
//        interval: ddtsecsM801 
//		The model is stored in arrays
//			yeq_real_tq, and
//			beq_real_tq
//
//-------------------------------------------------------------------
BOOL M801::TQN_ComputeConverterFirstStepACF_801()
{
	int i;
	int j;

		
	for ( i=0 ; i<devsta_tq ; i++ )
	{		
		for ( j=0 ; j<devsta_tq ; j++ )
		{	
			yeq_real_tq[i][j] = dYmatrixConverter[i][j];
		}
	}

	for ( i=0 ; i<devsta_tq ; i++ )
	{	
		dx1=0.0;
		for ( j=0 ; j<devsta ; j++ )
		{	
			dx1 += dPmatrixConverter[i][j] * xa_real_tq[j];
			
		}
		beq_real_tq[i]= dx1;
	}


	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_DebugReportModel801()
{
	DumpString("\nModel 801 Debug Report\n");
	DumpString("----------------------\n");

	DumpDoubleScalar("dTimePeriod",dTimePeriod);
	DumpDoubleScalar("ddtsecsM801",ddtsecsM801);
	DumpDoubleScalar("nBlockedCycles",nBlockedCycles);
	DumpDoubleScalar("dBlockedTime",dBlockedTime);

	DumpDoubleScalar("dValveONConductance",dValveONConductance);
	DumpDoubleScalar("dValveOffConductance",dValveOffConductance);
	DumpDoubleScalar("dParasiticCapacitance",dParasiticCapacitance);
	DumpDoubleScalar("dSnubberCapacitance",dSnubberCapacitance);
	DumpDoubleScalar("dSnubberConductance",dSnubberConductance);
	DumpDoubleScalar("dCLConductance",dCLConductance);
	DumpDoubleScalar("dCLInductance",dCLInductance);
	DumpDoubleScalar("dSmoothingCapacitance",dSmoothingCapacitance);
	DumpDoubleScalar("dRealPower801",dRealPower801);

	DumpString("\n---------------------------------------------\n\n");

	return true;
}
//-------------------------------------------------------------------
BOOL M801::TQN_DebugMatrixReportModel801()
{
	//DumpString("\nModel 801 Debug Matrix Report\n");
	//DumpString("----------------------\n");

	//DumpDoubleVector("xi_real_tq",devsta_tq,xi_real_tq);	
	//DumpDoubleVector("xa_real_tq",devsta_tq,xa_real_tq);

	//DumpIntVector("iValveStatusPerModeM801",6,iValveStatusPerModeM801);
	//DumpDoubleScalar("Run_time",pNetSolver->run_time);

	//// Checking the matrix of Y, P and Z of single valve

	DumpDoubleMatrix("dYmatrixSvalveON",10,10,dYmatrixSvalveON);
	DumpDoubleMatrix("dYmatrixSvalveOFF",10,10,dYmatrixSvalveOFF);
	DumpDoubleMatrix("dPmatrixSvalveON",10,5,dPmatrixSvalveON);
	DumpDoubleMatrix("dPmatrixSvalveOFF",10,5,dPmatrixSvalveOFF);
	DumpDoubleMatrix("dZmatrixSvalve",10,5,dZmatrixSvalve);

	DumpDoubleMatrix("dYmatrixCap",4,4,dYmatrixCap);
	DumpDoubleMatrix("dPmatrixCap",4,2,dPmatrixCap);
	DumpDoubleMatrix("dZmatrixCap",4,2,dZmatrixCap);

	//// Checking the matrix of Y, P, and Z of converter

	DumpDoubleMatrix("dYmatrixConverter",devsta_tq,devsta_tq,dYmatrixConverter);
	//DumpDoubleMatrix("dPmatrixConverter",devsta_tq,devsta_tq/2,dPmatrixConverter);
	//DumpDoubleMatrix("dZmatrixConverter",devsta_tq,devsta_tq/2,dZmatrixConverter);

	//// Checking the matrix of yeq and beq of converter

	//DumpDoubleMatrix("Yeq_real_tq",devsta_tq,devsta_tq,yeq_real_tq);
	//DumpDoubleVector("beq_real_tq",devsta_tq,beq_real_tq);

	DumpString("\n---------------------------------------------\n\n");

	return true;
}

#endif

//-------------------------------------------------------------------