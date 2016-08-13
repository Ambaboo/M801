//-------------------------------------------------------------------
//
//		File: WMASTER\DEVICES\M801\m801tdm.cpp
//
//		Six-Pulse Converter Model
//
//		Quadratic Integration Method
//
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\agc\agc.h"
#include "\wmaster\util\DbgDump.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\Multerm.h"
#include "\wmaster\apps\scaqcf\PowerDeviceT.h"
#include "M801.h"
#include <fstream>
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

//-------------------------------------------------------------------
/*OLD
BOOL M801::TQN_TimeDomainModelInit()
{
	int i;
	int	j;
	int iValve;
	int	iMode;

	//------debug report / input data

	ndebg = 0;
	if ( ndebg == 1 )
	{
		DebugDeviceGuiData();
		DebugDeviceInternalData();
		DebugDeviceOptimalData();
	}
	ndebg = 0;

	//------initialize constants

	iInverterSStatusModel	= 12;
	iNextCommutationValve	= -1;
	iNextTurnONValve		= -1;

	bConverterON			= false;
	bCommutationON			= false;
	bLogicStart				= false;

	dTimePeriod				= 1.0 / GetBaseFrequency();
	dCommutationTime 		= 5.0 / 360.0 * dTimePeriod;
	dCommutationStartTime	= 0.0;
	dConverterRelativeTime	= 0.0;
	dConverterTimeRef		= 0.0;

	//--------debug variable

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

	//----- Given the inverter mode, copy SCAQCF Model M801 to device SCAQCF Object 

	iMode = 12;		//------initialize to OFF mode
	if ( !TQN_CopyAQCFModelM801toAQCFStandard(iMode) ) return false;

	// ----- Create AQCF model by substituting control values to SCAQCF model

	if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;

	//------ Copy AQCF model to the object AQCF model for simulation 

	if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;


	return true;
}
*/
BOOL M801::TQN_TimeDomainModelInit()
{
	int i;
	int	j;
	int iValve;
	int	iMode;

	//------debug report / input data

	ndebg = 0;
	if (ndebg == 1)
	{
		DebugDeviceGuiData();
		DebugDeviceInternalData();
		DebugDeviceOptimalData();
	}
	ndebg = 0;

	//------initialize constants

	iInverterSStatusModel = 2; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	iNextCommutationValve = -1;
	iNextTurnONValve = -1;

	bConverterON = false;
	bCommutationON = false;
	bLogicStart = false;

	dTimePeriod = 1.0 / GetBaseFrequency();
	dCommutationTime = 5.0 / 360.0 * dTimePeriod;
	dCommutationStartTime = 0.0;
	dConverterRelativeTime = 0.0;
	dConverterTimeRef = 0.0;

	//--------debug variable

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
	dLogicStartTime = dev_oparm[11];			// seconds after logic starts
	dConverterStartTime = dev_oparm[12];			// seconds after simulation initiation

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

	//----- Given the inverter mode, copy SCAQCF Model M801 to device SCAQCF Object 

	iMode = 2;		//------initialize to OFF mode
	if (!TQN_CopyAQCFModelM801toAQCFStandard(iMode)) return false;

	// ----- Create AQCF model by substituting control values to SCAQCF model

	if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;

	//------ Copy AQCF model to the object AQCF model for simulation 

	if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;


	return true;
}
//-------------------------------------------------------------------
//	
//      this routine performs the time loop
//		computations for the model:
//		six-pulse converter
//
//-------------------------------------------------------------------
/*OLD
BOOL M801::TQN_TimeDomainModelTimeStep()
{

	int i, j, k1, k2;
	int iValve;
	double dActualRunTime;

	// ----- update converter time reference (every second) 
	// ----- and the converter relative time

	dConverterTimeRef		= int(pNetSolver->run_time);
	dActualRunTime			= pNetSolver->run_time;
	dConverterRelativeTime	= dActualRunTime - dConverterTimeRef;

	if (dConverterRelativeTime < 0.0)
	{
		AfxMessageBox("Check M801 converter relative time in TQN_TimeDomainModelTimeStep");
	}

	// ----- update information from DSP
	
	dFrequency			= xa_real_tq[8];
	dVacZeroTime		= xa_real_tq[5] + 60.0 / (360.0*dFrequency); // Translate from Vab zero crossing to Vac zero crossing 
	dPosSeqVMagn		= xa_real_tq[6];
	dRealPowerAC		= xa_real_tq[7];
	
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


	if (bLogicStart && (dActualRunTime > dConverterStartTime - 0.003) && !bConverterON)
	{
		DumpString("\nModel 801 Debug Report\n");
		DumpString("\nreport on entering time routine\n\n");
		TQN_DebugReportModel801();
	}

	if (bConverterON && (dActualRunTime < dConverterStartTime + 0.033))
	{
		DumpString("\nModel 801 Debug Report\n");
		DumpString("\nreport on entering time routine\n\n");
		TQN_DebugReportModel801();
	}

	// ---- Check whether converter is ON

	if ( bConverterON )
	{
		// ---- Converter is ON
		// ---- Check commutation is ON

		if ( bCommutationON ) 
		{
			// ---- Commutation is ON
			// ---- Compute the current through valve under commutation

			if ( iValveStatusPerMode[iNextCommutationValve][iInverterSStatusModel] == 1)
			{
				k1 = iValvePointer[1][iNextCommutationValve];
				k2 = iValvePointer[3][iNextCommutationValve];

				dCurrentNextCommuationValve = (xa_real_tq[k2] - xa_real_tq[k1]) * dValveONConductance;

				if ( dCurrentNextCommuationValve > 0.0 )
				{
					// Commutation has not ended

					// If commutation has not ended until the next valve turn one time arrives
					if ( dConverterRelativeTime>dValveTurnONTimes[iNextTurnONValve] )
					{
						//------ issue diagnostic
						
						AfxMessageBox("Check M801 converter relative time in TQN_TimeDomainModelTimeStep");
						TQN_DebugReportModel801();

						//------ recovery , force next switching state

						dCommutationTime		= dActualRunTime - dCommutationStartTime;
						iInverterSStatusModel	= iNextTurnONValve * 2;
						iNextCommutationValve	= ( iNextTurnONValve+4 ) % 6;
						iNextTurnONValve		= (iNextTurnONValve+1) % 6;

						bCommutationON			= true;
						dCommutationStartTime	= dActualRunTime;			
						
						// ---- Update Valve ScheduleTime

						if ( !TQN_UpdateValveScheduleTime() ) return false;

						// Return the AQCF converter model given iInverterSStatusModel

						if ( !TQN_CopyAQCFModelM801toAQCFStandard(iInverterSStatusModel) ) return false;

						// ----- Create AQCF model by substituting control values to SCAQCF model

						if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;
							
						//------ Copy AQCF model to the object AQCF model for simulation 

						if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;

						pNetSolver->bUpdateLnSystemFlag = true;
					}

					// ------ Update past history
					if (!TQN_UpdatePastHistory()) return false;

				}
				else
				{
					// Commutation completed

					dCommutationTime	= dActualRunTime - dCommutationStartTime;
					iInverterSStatusModel++;
					bCommutationON		= false;

					//-------- if iInverterSStatusModel > 11 issue diagnostic

					if ( iInverterSStatusModel > 11 )
					{
						AfxMessageBox("Wrong iInverterSStatusModel in M801");
						return false;
					}

					// Return the AQCF converter model given iInverterSStatusModel

					if ( !TQN_CopyAQCFModelM801toAQCFStandard(iInverterSStatusModel) ) return false;

					// ----- Create AQCF model by substituting control values to SCAQCF model

					if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;
					
					//------ Copy AQCF model to the object AQCF model for simulation 

					if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;

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
				// ---- Turn next valve ON

				iInverterSStatusModel	= iNextTurnONValve * 2;
				iNextCommutationValve	= ( iNextTurnONValve+4 ) % 6;
				iNextTurnONValve		= (iNextTurnONValve+1) % 6;

				bCommutationON			= true;
				dCommutationStartTime	= dActualRunTime;			

				// ----- if iInverterSStatusModel > 11 issue diagnostic

					if ( iInverterSStatusModel > 11 )
					{
						AfxMessageBox("Wrong iInverterSStatusModel in M801");
						return false;
					}

				if ( !TQN_CalculateFiringAngleINV() ) return false;

				// ---- Update Valve ScheduleTime
	
				if ( !TQN_UpdateValveScheduleTime() ) return false;

				// Return the AQCF converter model given iInverterSStatusModel

				if ( !TQN_CopyAQCFModelM801toAQCFStandard(iInverterSStatusModel) ) return false;

				// ----- Create AQCF model by substituting control values to SCAQCF model

				if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;
					
				//------ Copy AQCF model to the object AQCF model for simulation 

				if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;

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

		if ( dLogicStartTime < dActualRunTime ) 
		{
			if ( bLogicStart )
			{
				// ---- Logic part has started
				// ---- Compute firing anlge
		
				if ( !TQN_CalculateFiringAngleINV() ) return false;

				// ---- Computer Valve ScheduleTime
	
				if ( !TQN_ComputeValveScheduleTime() ) return false;

				// ---- Check Converter Start Time

				if ( dConverterStartTime < dActualRunTime )
				{
					// ---- Converter just Started
					// Intialize the converter model
					
					// ----- Define iInverterSStatusModel 
					
					dx0 = dConverterRelativeTime - dValveTurnONTimes[0];
					dx1 = dx0 / (dTimePeriod/6);

					if ( dx1 >= 0 )	
					{
						iInverterSStatusModel = int(dx1) * 2 + 1; 
						iNextTurnONValve = (int(dx1) + 1) % 6;
					}
					else
					{
						iInverterSStatusModel = (5 - int(abs(dx1))) * 2 + 1;
						iNextTurnONValve = (5 - int(abs(dx1)) + 1) % 6;
					}

					// ----- if iInverterSStatusModel > 11 issue diagnostic
					
					if ( iInverterSStatusModel>11 ) 
					{
						AfxMessageBox("Wrong iInverterSStatusModel in M801");
						return false;
					}
			
					// ----- Return the AQCF converter model given iInverterSStatusModel

					if ( !TQN_CopyAQCFModelM801toAQCFStandard(iInverterSStatusModel) ) return false;

					// ----- Create AQCF model by substituting control values to SCAQCF model

					if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;
					
					//------ Copy AQCF model to the object AQCF model for simulation 

					if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;

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

	if ( bLogicStart && (dActualRunTime > dConverterStartTime-0.003) && !bConverterON )
	{
		DumpString("\nreport on exiting time routine\n\n");
		TQN_DebugReportModel801();
	}

	if ( bConverterON && (dActualRunTime < dConverterStartTime+0.033) )
	{
		DumpString("\nreport on exiting time routine\n\n");
		TQN_DebugReportModel801();
	}
	

	return true;

}
*/

BOOL M801::TQN_TimeDomainModelTimeStep()
{

	int i, j, k1, k2;
	int iValve;
	double dActualRunTime;
	double valveOnTime;

	ofstream outfile44("Jingfan.txt", ios_base::out | ios_base::trunc);
	outfile44 << " yeq_real_tq " << endl;
	for (i = 0; i<devsta_tq; i++)
	{
	for (j = 0; j<devsta_tq; j++)
	{
	if (yeq_real_tq[i][j] != 0.0) outfile44 << i << "  " << j << "  " << yeq_real_tq[i][j] << endl;
	}
	}
	outfile44 << endl;


	// ----- update converter time reference (every second) 
	// ----- and the converter relative time

	dConverterTimeRef = int(pNetSolver->run_time);
	dActualRunTime = pNetSolver->run_time;
	dConverterRelativeTime = dActualRunTime - dConverterTimeRef;

	if (dConverterRelativeTime < 0.0)
	{
		AfxMessageBox("Check M801 converter relative time in TQN_TimeDomainModelTimeStep");
	}

	// ----- update information from DSP

	dFrequency = xa_real_tq[7];
	dVacZeroTime = xa_real_tq[4] + 60.0 / (360.0*dFrequency); // Translate from Vab zero crossing to Vac zero crossing 
	dPosSeqVMagn = xa_real_tq[5];
	dRealPowerAC = xa_real_tq[6];

	//------compute through variable currents

	for (i = 0; i<devsta_tq; i++)
	{
		dx0 = -beq_real_tq[i];
		for (j = 0; j<devsta_tq; j++)
		{
			dx0 += yeq_real_tq[i][j] * xa_real_tq[j];
		}
		xi_real_tq[i] = dx0;
	}

	//------perform diagnostic loop

	ndebg = 0;
	if (ndebg == 1)
	{
		dx0 = 0.0;
		for (i = devnod - 3; i<devsta_tq / 2; i++)
		{
			dx0 += fabs(xi_real_tq[i]);
		}
		if (dx0 > 1.0e-6)
		{
			amsr1.Format("NonZero InternalCurrents-Model 801: %10.8f", dx0);
			DumpString(amsr1);
			AfxMessageBox(amsr1);
			return false;
		}
	}
	ndebg = 0;


	if (bLogicStart && (dActualRunTime > dConverterStartTime - 0.003) && !bConverterON)
	{
		DumpString("\nModel 801 Debug Report\n");
		DumpString("\nreport on entering time routine\n\n");
		TQN_DebugReportModel801();
	}

	if (bConverterON && (dActualRunTime < dConverterStartTime + 0.033))
	{
		DumpString("\nModel 801 Debug Report\n");
		DumpString("\nreport on entering time routine\n\n");
		TQN_DebugReportModel801();
	}

	// ---- Check whether converter is ON

	if (bConverterON)
	{

	}
	else
	{
		// ----- Converter is NOT ON
		// ----- Check Logic Start Time

		if (dLogicStartTime < dActualRunTime)
		{
			valveOnTime = dActualRunTime - dLogicStartTime;

			iInverterSStatusModel = int(valveOnTime * 100) % 2;

			if (iInverterSStatusModel>1)
			{
				AfxMessageBox("Wrong iInverterSStatusModel in M801");
				return false;
			}

			// ----- Return the AQCF converter model given iInverterSStatusModel

			if (!TQN_CopyAQCFModelM801toAQCFStandard(iInverterSStatusModel)) return false;

			// ----- Create AQCF model by substituting control values to SCAQCF model

			if (!pPowerDeviceT->TQN_TimeDomainSCAQCFtoAQCFModelCreation()) return false;

			//------ Copy AQCF model to the object AQCF model for simulation 

			if (!pPowerDeviceT->TQN_TimeDomainCopySAQCFtoObject()) return false;

			pNetSolver->bUpdateLnSystemFlag = true;

			// Set bConverterON to be true

			//bConverterON = true;

			// Update past history

			if (!TQN_UpdatePastHistory()) return false;
		}
		else
		{
			// Logic has not started
			// ---- Update past history

			if (!TQN_UpdatePastHistory()) return false;
		}
	}

	// Debug Report

	if (bLogicStart && (dActualRunTime > dConverterStartTime - 0.003) && !bConverterON)
	{
		DumpString("\nreport on exiting time routine\n\n");
		TQN_DebugReportModel801();
	}

	if (bConverterON && (dActualRunTime < dConverterStartTime + 0.033))
	{
		DumpString("\nreport on exiting time routine\n\n");
		TQN_DebugReportModel801();
	}


	return true;

}
//-------------------------------------------------------------------
//	
//      this routine computes the pointers for the model
//		1. state index in each valve/cap -> system state index
//		   i.e. iValvePointer / iCapPointerM801_tq
//		2. valve status for each mode
//		   i.e. iValveStatusPerMode
//
//-------------------------------------------------------------------
//UPDATED
BOOL M801::TQN_ComputeModelPointers()
{
	//------define valve pointers

	iValvePointer[0][0] =  0;
	iValvePointer[1][0] =  2;
	iValvePointer[2][0] =  8;
	iValvePointer[3][0] =  9;
	iValvePointer[4][0] = 10;
	iValvePointer[5][0] = 20;
	iValvePointer[6][0] = 22;
	iValvePointer[7][0] = 24;
	iValvePointer[8][0] = 25;
	iValvePointer[9][0] = 26;

	iValvePointer[0][1] =  3;
	iValvePointer[1][1] =  0;
	iValvePointer[2][1] = 11;
	iValvePointer[3][1] = 12;
	iValvePointer[4][1] = 13;
	iValvePointer[5][1] = 23;
	iValvePointer[6][1] = 20;
	iValvePointer[7][1] = 27;
	iValvePointer[8][1] = 28;
	iValvePointer[9][1] = 29;

	iValvePointer[0][2] =  1;
	iValvePointer[1][2] =  2;
	iValvePointer[2][2] = 14;
	iValvePointer[3][2] = 15;
	iValvePointer[4][2] = 16;
	iValvePointer[5][2] = 21;
	iValvePointer[6][2] = 22;
	iValvePointer[7][2] = 30;
	iValvePointer[8][2] = 31;
	iValvePointer[9][2] = 32;

	iValvePointer[0][3] =  3;
	iValvePointer[1][3] =  1;
	iValvePointer[2][3] = 17;
	iValvePointer[3][3] = 18;
	iValvePointer[4][3] = 19;
	iValvePointer[5][3] = 23;
	iValvePointer[6][3] = 21;
	iValvePointer[7][3] = 33;
	iValvePointer[8][3] = 34;
	iValvePointer[9][3] = 35;

	//------define capacitor pointers

	iCapPointerM801_tq[0] = 2;
	iCapPointerM801_tq[1] = 3;
	iCapPointerM801_tq[2] = 22;
	iCapPointerM801_tq[3] = 23;

	//------define ON/OFF state sequence for each valve
	//------iValveStatusPerMode[i][j]=k; i: ith valve; j: jth mode; k: ON (1) or OFF (0)

	//------Mode 0
	iValveStatusPerMode[0][0]	= 0;
	iValveStatusPerMode[1][0]	= 1;
	iValveStatusPerMode[2][0]	= 1;
	iValveStatusPerMode[3][0]	= 0;

	//------Mode 1
	iValveStatusPerMode[0][1]	= 1;
	iValveStatusPerMode[1][1]	= 0;
	iValveStatusPerMode[2][1]	= 0;
	iValveStatusPerMode[3][1]	= 1;

	//------Mode 2
	iValveStatusPerMode[0][2]	= 0;
	iValveStatusPerMode[1][2]	= 0;
	iValveStatusPerMode[2][2]	= 0;
	iValveStatusPerMode[3][2]	= 0;

	return true;

}
//-------------------------------------------------------------------
//	
//      this routine prepares the converter AQCF model
//		for all the modes
//
//-------------------------------------------------------------------
//UPDATED
BOOL M801::TQN_PrepareAQCFConverterModel()
{
	int iMode, nMode, iValve;
	int i, j, i1, j1;
	
	// this routine prepares the converter AQCF model for all modes

	// ---- define the converter AQCF model size: 12 + 1 modes

	nMode = 3; //UPDATED
	pTDSCAQCFModel_M801.resize(nMode);

	for ( iMode=0; iMode<nMode; iMode++ )
	{
		pTDSCAQCFModel_M801[iMode] = new CPowerDevice_M801();

		pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_Equ			= nConverterEqu;
		pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_EquOver2	= nConverterEqu / 2;
		pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_State		= nConverterState;
		pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_Control		= 0;
		
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yeqx		= NewMatrix(nConverterEqu, nConverterState);
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yequ		= NewMatrix(nConverterEqu, 0);
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Neqx		= NewMatrix(nConverterEqu, nConverterEqu / 2);
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Nequ		= NewMatrix(nConverterEqu, 0);
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq			= NewMatrix(nConverterEqu, nConverterEqu / 2);
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Keq			= NewVector(nConverterEqu);

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
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yeqx[i1][j1] 
							+= pTDSCAQCFModel_ValveON->pTDSCAQCFModel_Yeqx[i][j];
					}

					for ( j=0 ; j<nValTerm ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Neqx[i1][j1] 
							+= pTDSCAQCFModel_ValveON->pTDSCAQCFModel_Neqx[i][j];
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[i1][j1] 
							+= pTDSCAQCFModel_ValveON->pTDSCAQCFModel_Meq[i][j];
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
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yeqx[i1][j1] 
							+= pTDSCAQCFModel_ValveOFF->pTDSCAQCFModel_Yeqx[i][j];
					}

					for ( j=0 ; j<nValTerm ; j++ )
					{
						j1 = iValvePointer[j][iValve];
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Neqx[i1][j1] 
							+= pTDSCAQCFModel_ValveOFF->pTDSCAQCFModel_Neqx[i][j];
						pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[i1][j1] 
							+= pTDSCAQCFModel_ValveOFF->pTDSCAQCFModel_Meq[i][j];
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
				pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yeqx[i1][j1] 
					+= pTDSCAQCFModel_Cap->pTDSCAQCFModel_Yeqx[i][j];
			}

			for ( j=0 ; j<nCapTerm ; j++ )
			{
				j1 = iCapPointerM801_tq[j];
				pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Neqx[i1][j1] 
					+= pTDSCAQCFModel_Cap->pTDSCAQCFModel_Neqx[i][j];
				pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[i1][j1] 
					+= pTDSCAQCFModel_Cap->pTDSCAQCFModel_Meq[i][j];
			}
		}

		// ---- Adjust Meq

		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[0][0] = 1.0;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[1][1] = 1.0;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[2][2] = 1.0;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[3][3] = 1.0;
		//UPDATED

		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[0+nConverterEqu/2][0] = -0.5;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[1+nConverterEqu/2][1] = -0.5;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[2+nConverterEqu/2][2] = -0.5;
		pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[3+nConverterEqu/2][3] = -0.5;
		//UPDATED
	}
	
	return true;
}
//-------------------------------------------------------------------
//	
//      this routine copies the AQCF model to
//		simulation object
//
//-------------------------------------------------------------------
BOOL M801::TQN_CopyAQCFModelM801toAQCFStandard(int iMode)
{
	int i, j;
	int nEqu, nEquOver2, nStates, nStatesOver2, nControl;

	pPowerDeviceT->TQN_ClearSCAQCF();

	pPowerDeviceT->nTDSCAQCFModel_Equ			= pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_Equ;		
	pPowerDeviceT->nTDSCAQCFModel_EquOver2		= pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_EquOver2;
	pPowerDeviceT->nTDSCAQCFModel_State			= pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_State;	
	pPowerDeviceT->nTDSCAQCFModel_Control		= pTDSCAQCFModel_M801[iMode]->nTDSCAQCFModel_M801_Control;	

	nEqu										= pPowerDeviceT->nTDSCAQCFModel_Equ;		
	nEquOver2									= pPowerDeviceT->nTDSCAQCFModel_Equ / 2;		
	nStates										= pPowerDeviceT->nTDSCAQCFModel_State;		
	nStatesOver2								= pPowerDeviceT->nTDSCAQCFModel_State / 2;		
	nControl									= pPowerDeviceT->nTDSCAQCFModel_Control;		

	pPowerDeviceT->pTDSCAQCFModel_Yeqx			= NewMatrix(nEqu, nStates);
	pPowerDeviceT->pTDSCAQCFModel_Yequ			= NewMatrix(nEqu, nControl);
	pPowerDeviceT->pTDSCAQCFModel_Neqx			= NewMatrix(nEqu, nStatesOver2);
	pPowerDeviceT->pTDSCAQCFModel_Nequ			= NewMatrix(nEqu, nControl);
	pPowerDeviceT->pTDSCAQCFModel_Meq			= NewMatrix(nEqu, nEquOver2);
	pPowerDeviceT->pTDSCAQCFModel_Keq			= NewVector(nEqu);

	// Copy pTDSAQCFModel_Yeqx

	for (i = 0; i < nEqu; i++)
	{
		for (j = 0; j < nStates; j++)
		{
			pPowerDeviceT->pTDSCAQCFModel_Yeqx[i][j] 
				= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yeqx[i][j];
		}
	}

	// Copy pTDSAQCFModel_Yequ

	for (i = 0; i < nEqu; i++)
	{
		for (j = 0; j < nControl; j++)
		{
			pPowerDeviceT->pTDSCAQCFModel_Yequ[i][j] 
				= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Yequ[i][j];
		}
	}
	
	// Copy pTDSAQCFModel_Neqx
	for (i = 0; i < nEqu; i++)
	{
		for (j = 0; j < nStatesOver2; j++)
		{
			pPowerDeviceT->pTDSCAQCFModel_Neqx[i][j]
				= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Neqx[i][j];
		}
	}

	// Copy pTDSAQCFModel_Nequ
	for (i = 0; i < nEqu; i++)
	{
		for (j = 0; j < nControl/2; j++)
		{
			pPowerDeviceT->pTDSCAQCFModel_Nequ[i][j]
				= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Nequ[i][j];
		}
	}

	// Copy pTDSAQCFModel_Meq
	for (i = 0; i < nEqu; i++)
	{
		for (j = 0; j < nEquOver2; j++)
		{
			pPowerDeviceT->pTDSCAQCFModel_Meq[i][j] 
				= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Meq[i][j];
		}
	}

	// Copy pTDSCAQCFModel_Keq
	for (i = 0; i < nEqu; i++)
	{
		pPowerDeviceT->pTDSCAQCFModel_Keq[i] 
			= pTDSCAQCFModel_M801[iMode]->pTDSCAQCFModel_M801_Keq[i];
	}

	return true;
}
//-------------------------------------------------------------------
//	
//      this routine prepares the single valve model
//		with valve ON status
//
//-------------------------------------------------------------------
BOOL M801::TQN_SingleValveModelStatusOnModel()
{
	int i;
	int j;

	pTDSCAQCFModel_ValveON = new CPowerDeviceT(this);

	//----- Dimension of the SCQDM array

	pTDSCAQCFModel_ValveON->nTDSCQDModel_Equ1		= 2;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Equ2		= 3;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Equ3		= 0;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_State		= 5;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Control	= 0;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Feqxx		= 0;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Fequu		= 0;
	pTDSCAQCFModel_ValveON->nTDSCQDModel_Fequx		= 0;

	if (!pTDSCAQCFModel_ValveON->TQN_TimeDomainQuadratizedModelCreation()) return false;

	//------ Quadratized Model
	//----- Equation Set 1

	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[0][0]	=  dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[0][1]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[0][2]	=  dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[0][3]	= -dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[0][4]	=  1.0;

	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[1][0]	= -dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[1][1]	=  dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[1][2]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[1][3]	=  dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx1[1][4]	= -1.0;

	//----- Equation Set 2

	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[0][1]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[0][2]	= dSnubberConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Deqxd2[0][0]	= -dSnubberCapacitance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Deqxd2[0][2]	= dSnubberCapacitance;

	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[1][0]	= -dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[1][1]	= -dValveONConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[1][3]	= dValveONConductance + dCLConductance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[1][4]	= -1.0;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Deqxd2[1][1]	= -dParasiticCapacitance;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Deqxd2[1][3]	= dParasiticCapacitance;

	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[2][0]	= -1.0;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Yeqx2[2][3]	= 1.0;
	pTDSCAQCFModel_ValveON->pTDSCQDModel_Deqxd2[2][4]	= dCLInductance;

	//------ SCAQCF creation from quadratized model automatically

	if (!pTDSCAQCFModel_ValveON->TQN_TimeDomainQM_SCAQCFModelCreation1()) return false;

	return true;
}
//-------------------------------------------------------------------
//	
//      this routine prepares the single valve model
//		with valve OFF status
//
//-------------------------------------------------------------------
BOOL M801::TQN_SingleValveModelStatusOffModel()
{
	int i;
	int j;

	pTDSCAQCFModel_ValveOFF = new CPowerDeviceT(this);
	pTDSCAQCFModel_ValveOFF->TQN_ClearSCAQCF();

	//----- Dimension of the SCQDM array

	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Equ1		= 2;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Equ2		= 3;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Equ3		= 0;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_State		= 5;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Control	= 0;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Feqxx		= 0;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Fequu		= 0;
	pTDSCAQCFModel_ValveOFF->nTDSCQDModel_Fequx		= 0;

	if (!pTDSCAQCFModel_ValveOFF->TQN_TimeDomainQuadratizedModelCreation()) return false;

	//------ Quadratized Model
	//----- Equation Set 1

	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[0][0]	=  dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[0][1]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[0][2]	=  dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[0][3]	= -dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[0][4]	=  1.0;

	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[1][0]	= -dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[1][1]	=  dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[1][2]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[1][3]	=  dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx1[1][4]	= -1.0;

	//----- Equation Set 2

	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[0][1]	= -dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[0][2]	= dSnubberConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Deqxd2[0][0]	= -dSnubberCapacitance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Deqxd2[0][2]	= dSnubberCapacitance;

	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[1][0]	= -dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[1][1]	= -dValveOffConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[1][3]	= dValveOffConductance + dCLConductance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[1][4]	= -1.0;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Deqxd2[1][1]	= -dParasiticCapacitance;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Deqxd2[1][3]	= dParasiticCapacitance;

	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[2][0]	= -1.0;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Yeqx2[2][3]	= 1.0;
	pTDSCAQCFModel_ValveOFF->pTDSCQDModel_Deqxd2[2][4]	= dCLInductance;

	//------ SCAQCF creation from quadratized model automatically

	if (!pTDSCAQCFModel_ValveOFF->TQN_TimeDomainQM_SCAQCFModelCreation1()) return false;



/*
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

*/
	return true;
}
//-------------------------------------------------------------------
//	
//      this routine prepares the single capacitor model
//
//-------------------------------------------------------------------
BOOL M801::TQN_SingleCapacitorModel()
{
	int i;
	int j;

	pTDSCAQCFModel_Cap = new CPowerDeviceT(this);
	pTDSCAQCFModel_Cap->TQN_ClearSCAQCF();

	//----- Dimension of the SCQDM array

	pTDSCAQCFModel_Cap->nTDSCQDModel_Equ1		= 2;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Equ2		= 0;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Equ3		= 0;
	pTDSCAQCFModel_Cap->nTDSCQDModel_State		= 2;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Control	= 0;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Feqxx		= 0;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Fequu		= 0;
	pTDSCAQCFModel_Cap->nTDSCQDModel_Fequx		= 0;

	if (!pTDSCAQCFModel_Cap->TQN_TimeDomainQuadratizedModelCreation()) return false;

	//------ Quadratized Model
	//----- Equation Set 1

	pTDSCAQCFModel_Cap->pTDSCQDModel_Deqxd1[0][0]	=  dSmoothingCapacitance;
	pTDSCAQCFModel_Cap->pTDSCQDModel_Deqxd1[0][1]	=  -dSmoothingCapacitance;
	pTDSCAQCFModel_Cap->pTDSCQDModel_Deqxd1[1][0]	=  -dSmoothingCapacitance;
	pTDSCAQCFModel_Cap->pTDSCQDModel_Deqxd1[1][1]	=  dSmoothingCapacitance;

	//------ SCAQCF creation from quadratized model automatically

	if (!pTDSCAQCFModel_Cap->TQN_TimeDomainQM_SCAQCFModelCreation1()) return false;


/*
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
*/
	return true;
}
//----------------------------------------------------------------------------
//
//		This routine is to calculate firing angle for next step
//		firing angle = firing angle of present time +deviation of firing angle
//		the deviation can be calcuated from deviation between measured power 
//		and prospected output power.
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
//		the process of firing angle calculation is for inverter.
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
//		This routine updates the valve turn ON schedule time 
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

		// adjust the turn on times in case that it goes to the next zero crossing time
		if ( dValveTurnONTimes[iValve] - dValveTurnONTimesOld[iValve] > dx1/2 )
		{
			dValveTurnONTimes[iValve] -= dTimePeriod;
		}
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
//
//		This routine computes the valve turn ON time 
//		when the logic just starts
//
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
//		This routine updates the past history 
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
//-------------------------------------------------------------------
//
//		This routine updates the past history 
//
//-------------------------------------------------------------------
BOOL M801::TQN_DebugReportModel801()
{
	int iValve, k1, k2, k3, k4, k5, iTemp;
	double dVoltage, dCurrent, dTemp;
	CString text;

	DumpString("----- Feedback Control Values -----\n");
	DumpDoubleScalar("dVacZeroTime",dVacZeroTime);
	DumpDoubleScalar("VaMagn",dPosSeqVMagn/sqrt(3)*sqrt(2));
	DumpDoubleScalar("dRealPowerAC",dRealPowerAC);
	DumpDoubleScalar("dFrequency",dFrequency);

	DumpString("----- Valve Status -----\n");
	if (iInverterSStatusModel == 12)	DumpIntScalar("iInverterSStatusModel",-1);
	else						DumpIntScalar("iInverterSStatusModel",iInverterSStatusModel);
	if (bCommutationON)	DumpString("bCommutationON = true \n");
	else				DumpString("bCommutationON = false \n");
	DumpIntScalar("iNextCommutationValve",iNextCommutationValve);
	DumpIntScalar("iNextTurnONValve",iNextTurnONValve);
	for ( iValve=0; iValve<nValves; iValve++ )
	{
		text.Format("Valve %d Status", iValve);
		DumpIntScalar(text,iValveStatusPerMode[iValve][iInverterSStatusModel]);
		
		k1 = iValvePointer[1][iValve];
		k2 = iValvePointer[3][iValve];
		k3 = iValvePointer[0][iValve];
		k4 = iValvePointer[4][iValve];
		k5 = iValvePointer[2][iValve];
		dVoltage = xa_real_tq[k2] - xa_real_tq[k1];
		text.Format("Valve %d Voltage Across Thyristor: %12.6f\n", iValve, dVoltage);
		DumpString(text);
		dVoltage = xa_real_tq[k3] - xa_real_tq[k1];
		text.Format("Valve %d Voltage Across Valve Terminals: %12.6f\n", iValve, dVoltage);
		DumpString(text);
		dCurrent = xa_real_tq[k4];
		text.Format("Valve %d Current Through Inductor: %12.6f\n", iValve, dCurrent);
		DumpString(text);

		dTemp = xa_real_tq[k3];
		text.Format("	Valve %d State 0: %12.6f\n", iValve, dTemp);
		DumpString(text);
		dTemp = xa_real_tq[k1];
		text.Format("	Valve %d State 1: %12.6f\n", iValve, dTemp);
		DumpString(text);
		dTemp = xa_real_tq[k5];
		text.Format("	Valve %d State 2: %12.6f\n", iValve, dTemp);
		DumpString(text);
		dTemp = xa_real_tq[k2];
		text.Format("	Valve %d State 3: %12.6f\n", iValve, dTemp);
		DumpString(text);
		dTemp = xa_real_tq[k4];
		text.Format("	Valve %d State 4: %12.6f\n\n", iValve, dTemp);
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