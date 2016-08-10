
//-------------------------------------------------------------------
//		File: WMASTER\DEVICES\M801\m801.cpp
//-------------------------------------------------------------------
#include "stdafx.h"
#include "\wmaster\devices\resource.h"
#include "\wmaster\devices\Device.h"
#include "\wmaster\devices\Series.h"
#include "\wmaster\util\VicDraw.h"
#include "\wmaster\solver\solvutil.h"
#include "\wmaster\dfe\xdialog.h"
#include "M801.h"
//-------------------------------------------------------------------
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
//-------------------------------------------------------------------
//
//	Single Phase Bridge Rectifier
//
//-------------------------------------------------------------------
//
//	------documentation of array dev_term
//
//	dev_term[0]	: bus name, AC side
//	dev_term[1]	: bus name, DC side
//	dev_term[2]	: node name of the zero crossing time of reference
//	dev_term[3]	: node name of the PosSeq. magnitude of reference
//	dev_term[4]	: node name of the power of reference
//  dev_term[5] : node name of freuency
//
//	------documentation of array dev_parm

//	DEVICE PARAMETERS SHOWN IN DEVICE WINDOW FOR S-P CONVERTER
	

//	dev_parm[ 0]	: thyristor 'on' conductance, g_on(Mhos)
//	dev_parm[ 1]    : thyristor 'off' conductance, g_off(micMhos)
//	dev_parm[ 2]    : thyristor parasitic capacitance, Cp(nanoFarads)
//	dev_parm[ 3]    : snubber circuit capacitance, C_s(microFarads)
//	dev_parm[ 4]    : snubber circuit resistance, R_s(ohms)
//	dev_parm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_parm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_parm[ 7]    : DC side smoothing capacitor, C(microFarads)
//	dev_parm[ 8]    : rated voltage (kV, L-L, rms)
//	dev_parm[ 9]    : DC Voltage setpoint (kV)
//	dev_parm[10]	: connected DSP device
//	dev_parm[11]	: logic start time
//	dev_parm[12]	: converter start time

//	dev_iparm[ 0]	: thyristor 'on' conductance, g_on(Mhos)
//	dev_iparm[ 1]    : thyristor 'off' conductance, g_off(micMhos)
//	dev_iparm[ 2]    : thyristor parasitic capacitance, Cp(nanoFarads)
//	dev_iparm[ 3]    : snubber circuit capacitance, C_s(microFarads)
//	dev_iparm[ 4]    : snubber circuit resistance, R_s(ohms)
//	dev_iparm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_iparm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_iparm[ 7]    : DC side smoothing capacitor, C(microFarads)
//	dev_iparm[ 8]    : rated voltage (kV, L-L, rms)
//	dev_iparm[ 9]    : DC Voltage setpoint (kV)
//	dev_iparm[10]	: connected DSP device
//	dev_iparm[11]	: logic start time
//	dev_iparm[12]	: converter start time

//	dev_oparm[ 0]	: thyristor 'on' conductance, g_on(Mhos)
//	dev_oparm[ 1]    : thyristor 'off' conductance, g_off(micMhos)
//	dev_oparm[ 2]    : thyristor parasitic capacitance, Cp(nanoFarads)
//	dev_oparm[ 3]    : snubber circuit capacitance, C_s(microFarads)
//	dev_oparm[ 4]    : snubber circuit resistance, R_s(ohms)
//	dev_oparm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_oparm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_oparm[ 7]    : DC side smoothing capacitor, C(microFarads)
//	dev_oparm[ 8]    : rated voltage (kV, L-L, rms)
//	dev_oparm[ 9]    : DC Voltage setpoint (kV)
//	dev_oparm[10]	: connected DSP device
//	dev_oparm[11]	: logic start time
//	dev_oparm[12]	: converter start time

//  M117 OLD

//	dev_parm[ 0]	: thyristor 'on' conductance, g_on(Mhos)
//	dev_parm[ 1]    : thyristor 'off' conductance, g_off(micMhos)
//	dev_parm[ 2]    : thyristor parasitic capacitance, Cp(nanoFarads)
//	dev_parm[ 3]    : snubber circuit capacitance, C_s(microFarads)
//	dev_parm[ 4]    : snubber circuit resistance, R_s(ohms)
//	dev_parm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_parm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_parm[ 7]    : DC side smoothing capacitor, C(microFarads)
//	dev_parm[ 8]    : rated voltage (kV, L-L, rms)
//	dev_parm[ 9]    : firing angle (degrees)	(use if control is 1)
//	dev_parm[10]    : real power setpoint (MWs) (use if control is 2)
//	dev_parm[11]    : DC Voltage setpoint (kV)	(use if control is 3)
//	dev_parm[12]	: control code: 1: equidistant, 2: real power, 3: voltage
//	dev_parm[13]	: operating mode 1=rectifier, 2=inverter
//	dev_parm[14]	: connected DSP device
//	dev_parm[15]	: power flow control 1: firing angle, 2: real power, 3: voltage
//	dev_parm[16]	: logic start time
//	dev_parm[17]	: converter start time

//	dev_iparm[ 0]	: thyristor 'on' conductance, g_on(mhos)
//	dev_iparm[ 1]   : thyristor 'off' conductance, g_off(mhos)
//	dev_iparm[ 2]   : thyristor parasitic capacitance, Cp(farads)
//	dev_iparm[ 3]   : snubber circuit capacitance, C_s(farads)
//	dev_iparm[ 4]   : snubber circuit resistance, R_s(ohms)
//	dev_iparm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_iparm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_iparm[ 7]   : dc side smoothing capacitor, C(farads)
//	dev_iparm[ 8]   : rated voltage (kV, L-L, rms)
//	dev_iparm[ 9]   : firing angle (degrees)	(use if control is 1)
//	dev_iparm[10]   : real power setpoint (MWs) (use if control is 2)
//	dev_iparm[11]   : DC Voltage setpoint (kV)	    (use if control is 3)
//	dev_iparm[12]	: control code: 1: equidistant, 2: real power, 3: voltage
//	dev_iparm[13]	: operating mode 1=rectifier, 2=inverter
//	dev_iparm[14]	: connected DSP device
//	dev_iparm[15]	: power flow control 1: firing angle, 2: real power, 3: voltage
//	dev_iparm[16]	: logic start time
//	dev_iparm[17]	: converter start time

//	dev_oparm[ 0]	: thyristor 'on' conductance, g_on(mhos)
//	dev_oparm[ 1]	: thyristor 'off' conductance, g_off(mhos)
//	dev_oparm[ 2]	: thyristor parasitic capacitance, Cp(farads)
//	dev_oparm[ 3]	: snubber circuit capacitance, C_s(farads)
//	dev_oparm[ 4]	: snubber circuit resistance, R_s(ohms)
//	dev_oparm[ 5]    : current rate limiting inductance, L(millihenries)
//	dev_oparm[ 6]    : resistor in parallel with reactor, r(ohms)
//	dev_oparm[ 7]	: dc side smoothing capacitor, C(farads)
//	dev_oparm[ 8]	: rated voltage (kV, L-L, rms)
//	dev_oparm[ 9]	: firing angle (degrees)	(use if control is 1)
//	dev_oparm[10]	: real power setpoint (MWs) (use if control is 2)
//	dev_oparm[11]	: DC Voltage setpoint (kV)	    (use if control is 3)
//	dev_oparm[12]	: control code: 1: equidistant, 2: real power, 3: voltage
//	dev_oparm[13]	: operating mode 1=rectifier, 2=inverter
//	dev_oparm[14]	: connected DSP device
//	dev_oparm[15]	: power flow control 1: firing angle, 2: real power, 3: voltage
//	dev_oparm[16]	: logic start time
//	dev_oparm[17]	: converter start time
//
//-------------------------------------------------------------------
M801::M801(void* pDoc)
{
	dev_ncode	 = 801;
	dev_mod_type = MOD_NONLINEAR;

	//--------Set pointers

	pDocument	= pDoc;
	pNetSolver	= GetNetSolverPointer(pDoc);

	//--------initialize device code and icon

	//--------initialize nurid and file name (if necessary)

	dev_nurid	= GetUniqueNurid(pDoc);
//	dev_fname = GetUniqueFname(pDocument);

	//--------other initialization

	NullDataModel_801();

	nValves			= 4;
	nValTerm		= 5;
	nCapTerm		= 2;
	nValTerm_tq		= 10;
	nCapTerm_tq		= 4;
	iViconType		= 2;
	nConverterEqu	= 54;
	nConverterState = 54;
	
	/*
	nValves = 6;
	nValTerm = 5;
	nCapTerm = 2;
	nValTerm_tq = 10;
	nCapTerm_tq = 4;
	iViconType = 2;
	nConverterEqu = 54;
	nConverterState = 54;
	*/
	
}
//-------------------------------------------------------------------
M801::~M801()
{
	DeleteDataModel_801();
}
//-------------------------------------------------------------------
BOOL M801::NullDataModel_801()
{
	iValvePointerM801		= NULL;
	iCapPointerM801			= NULL;

	dSValveCurrentM801		= NULL;
	dSValvePHCurrentM801	= NULL;

	dCpCurrent				= NULL;
	dCpPHCurrent			= NULL;
	dSnubberCurrent			= NULL;
	dValveCurrentM801		= NULL;
	dValveCurrentPastM801	= NULL;

	dMmatrixConverter		= NULL;
	dNmatrixConverter		= NULL;
	dYeqSubStepM801			= NULL;
	dPeqSubStepM801			= NULL;
	dY22M801				= NULL;
	dInvY22M801				= NULL;
	dY21M801				= NULL;
	dDM801					= NULL;
	b_vector				= NULL;
	x_vector				= NULL;
	y_vector				= NULL;

	dPastState				= NULL;
	iValveStatusM801		= NULL;	
	dValveConductanceM801	= NULL;
	dValveNextTurnOn		= NULL;
	dValveTurnONTimes		= NULL;
	dValveTurnONTimesOld	= NULL;
	dValveTurnONTimesNew	= NULL;
	nValveDelayM801			= NULL;
	dValveSwitchTime		= NULL;


//--------Single valve Matix form

	dYmatrixSvalveON		= NULL;
	dYmatrixSvalveOFF		= NULL;
	dPmatrixSvalveON		= NULL;
	dPmatrixSvalveOFF		= NULL;
	dYmatrixSvalve			= NULL;
	dPmatrixSvalve			= NULL;
	dZmatrixSvalve			= NULL;

//--------Capacitor Matix form

	dYmatrixCap				= NULL;
	dPmatrixCap				= NULL;
	dZmatrixCap				= NULL;

//--------Converter Matix form

	dYmatrixConverter		= NULL;
	dPmatrixConverter		= NULL;	
	dZmatrixConverter		= NULL;

	iValvePointer			= NULL;
	iCapPointerM801_tq		= NULL;

	iValveStatusPerMode		= NULL;
	iValvePresentTimeStatus	= NULL;

	Init_SCQDM_ValveON();
	Init_SCQDM_ValveOFF();
	Init_SCQDM_CAP();
	Init_SCAQCF_ValveON();
	Init_SCAQCF_ValveOFF();
	Init_SCAQCF_CAP();

	return true;
}
//-------------------------------------------------------------------
BOOL M801::AllocateDataModel_801()
{
	iValvePointerM801		= NewIMatrix (nValTerm,nValves);
	iCapPointerM801			= NewIVector (nCapTerm);
	
	dSValveCurrentM801		= NewMatrix (nValTerm,nValves);
	dSValvePHCurrentM801	= NewMatrix (nValTerm,nValves);

	dCpCurrent				= NewVector (nValves);
	dCpPHCurrent			= NewVector (nValves);
	dSnubberCurrent			= NewVector (nValves);
	dValveCurrentM801		= NewVector (nValves);
	dValveCurrentPastM801	= NewVector (nValves);

	dMmatrixConverter		= NewMatrix (devsta);
	dNmatrixConverter		= NewMatrix (devsta);
	dYeqSubStepM801			= NewMatrix (devsta);
	dPeqSubStepM801			= NewMatrix (devsta);
	dPastState				= NewVector (devsta);

	iValveStatusM801		= NewIVector (nValves);
	dValveConductanceM801	= NewVector (nValves);
	dValveNextTurnOn		= NewVector (nValves);
	dValveTurnONTimes		= NewVector (nValves*2);
	dValveTurnONTimesOld	= NewVector (nValves*2);
	dValveTurnONTimesNew	= NewVector (nValves*2);
	nValveDelayM801			= NewIVector (nValves);

	dValveSwitchTime		= NewVector (nValves);

	//--------Single valve Matix form

	dYmatrixSvalveON		= NewMatrix (nValTerm_tq);
	dYmatrixSvalveOFF		= NewMatrix (nValTerm_tq);
	dPmatrixSvalveON		= NewMatrix (nValTerm_tq,nValTerm);
	dPmatrixSvalveOFF		= NewMatrix (nValTerm_tq,nValTerm);
	dYmatrixSvalve			= NewMatrix (nValTerm_tq);
	dPmatrixSvalve			= NewMatrix (nValTerm_tq,nValTerm);
	dZmatrixSvalve			= NewMatrix (nValTerm_tq,nValTerm);

	//--------Capacitor Matix form

	dYmatrixCap				= NewMatrix (nCapTerm_tq);
	dPmatrixCap				= NewMatrix (nCapTerm_tq,nCapTerm);
	dZmatrixCap				= NewMatrix (nCapTerm_tq,nCapTerm);

	//--------Converter Matix form

	dYmatrixConverter		= NewMatrix (devequ_tq);	
	dPmatrixConverter		= NewMatrix (devequ_tq,devsta);	
	dZmatrixConverter		= NewMatrix (devequ_tq,devsta);	

	iValvePointer			= NewIMatrix (nValTerm_tq,nValves);
	iCapPointerM801_tq		= NewIVector (nCapTerm_tq);

	iValveStatusPerMode		= NewIMatrix(6,13);
	iValvePresentTimeStatus	= NewIVector (nValves);

	return true;
}

//-------------------------------------------------------------------
BOOL M801::DeleteDataModel_801()
{
	if(iValvePointerM801)			DeleteMatrix(iValvePointerM801);
	if(iCapPointerM801)				DeleteVector(iCapPointerM801);

	if(dSValveCurrentM801)			DeleteMatrix(dSValveCurrentM801);
	if(dSValvePHCurrentM801)		DeleteMatrix(dSValvePHCurrentM801);

	if(dCpCurrent)					DeleteVector(dCpCurrent);
	if(dCpPHCurrent)				DeleteVector(dCpPHCurrent);
	if(dSnubberCurrent)				DeleteVector(dSnubberCurrent);
	if(dValveCurrentM801)			DeleteVector(dValveCurrentM801);
	if(dValveCurrentPastM801)		DeleteVector(dValveCurrentPastM801);

	if(dMmatrixConverter)			DeleteMatrix(dMmatrixConverter);
	if(dNmatrixConverter)			DeleteMatrix(dNmatrixConverter);
	if(dYeqSubStepM801)				DeleteMatrix(dYeqSubStepM801);
	if(dPeqSubStepM801)				DeleteMatrix(dPeqSubStepM801);
	if(dPastState)					DeleteVector(dPastState);

	if(iValveStatusM801)			DeleteVector(iValveStatusM801);
	if(dValveConductanceM801)		DeleteVector(dValveConductanceM801);
	if(dValveNextTurnOn)			DeleteVector(dValveNextTurnOn);
	if(dValveTurnONTimes)			DeleteVector(dValveTurnONTimes);
	if(dValveTurnONTimesOld)		DeleteVector(dValveTurnONTimesOld);
	if(dValveTurnONTimesNew)		DeleteVector(dValveTurnONTimesNew);
	if(nValveDelayM801)				DeleteVector(nValveDelayM801);
	if(dValveSwitchTime)			DeleteVector(dValveSwitchTime);

	//--------Single valve Matix form

	if(dYmatrixSvalveON)			DeleteMatrix(dYmatrixSvalveON);	
	if(dYmatrixSvalveOFF)			DeleteMatrix(dYmatrixSvalveOFF);	
	if(dPmatrixSvalveON)			DeleteMatrix(dPmatrixSvalveON);	
	if(dPmatrixSvalveOFF)			DeleteMatrix(dPmatrixSvalveOFF);	
	if(dZmatrixSvalve)				DeleteMatrix(dZmatrixSvalve);
	if(dYmatrixSvalve)				DeleteMatrix(dYmatrixSvalve);		
	if(dPmatrixSvalve)				DeleteMatrix(dPmatrixSvalve);		

	//--------Capacitor Matix form

	if(dYmatrixCap)					DeleteMatrix(dYmatrixCap);				
	if(dPmatrixCap)					DeleteMatrix(dPmatrixCap);				
	if(dZmatrixCap)					DeleteMatrix(dZmatrixCap);				

	//--------Converter Matix form

	if(dYmatrixConverter)			DeleteMatrix(dYmatrixConverter);		
	if(dPmatrixConverter)			DeleteMatrix(dPmatrixConverter);		
	if(dZmatrixConverter)			DeleteMatrix(dZmatrixConverter);

	if(iValvePointer)				DeleteMatrix(iValvePointer);
	if(iCapPointerM801_tq)			DeleteVector(iCapPointerM801_tq);

	if(iValveStatusPerMode)			DeleteMatrix(iValveStatusPerMode);
	if(iValvePresentTimeStatus)		DeleteVector(iValvePresentTimeStatus);

	Clear_SCQDM_ValveON();	
	Clear_SCQDM_ValveOFF();	
	Clear_SCQDM_CAP();
	Clear_SCAQCF_ValveON();	
	Clear_SCAQCF_ValveOFF();
	Clear_SCAQCF_CAP();

	vector<M801ConverterAQCF*>::iterator it;
	for (it = vConverterAQCFModel.begin(); it != vConverterAQCFModel.end(); it++)
	{
		delete (*it);
	}
	vConverterAQCFModel.clear();
	
	return true;

}
//-------------------------------------------------------------------
void M801::SetVicon()
{
	SetVicData("M801");
}
//-------------------------------------------------------------------
void M801::SetDefaultValues()
{
	DEVINFO* devinfo = GetDeviceInfo(dev_ncode);
	dev_title.Format("%s (ID = %d)",devinfo->dev_class_name, dev_nurid);

	//--------terminals

	nd_term	= 6;
	if( dev_term ) delete [] dev_term;
	dev_term = new CString [nd_term];
	dev_term[0] = "NEWBUS1";
	dev_term[1]	= "NEWBUS2";
	dev_term[2]	= "ZEROX";
	dev_term[3]	= "POSSEQMAG";
	dev_term[4]	= "POWER";
	dev_term[5] = "Frequency";

	//--------circuits

	nd_circ = 1;
	if( dev_circ ) delete [] dev_circ;
	dev_circ = new CString [nd_circ];
	dev_circ[0]	= "1";

	//--------parameters

	/*
	nd_parm	= 18;
	if( dev_parm ) delete []	dev_parm;
	dev_parm = new CString [nd_parm];

	dev_parm[ 0] = "100.0";		
	dev_parm[ 1] = "0.1";		
	dev_parm[ 2] = "20.0";		
	dev_parm[ 3] = "0.1";		
	dev_parm[ 4] = "100.0";		
	dev_parm[ 5] = "100.0";		
	dev_parm[ 6] = "3.0";	
	dev_parm[ 7] = "500.0";		
	dev_parm[ 8] = "13.8";		
	dev_parm[ 9] = "45.0";		
	dev_parm[10] = "50.0";		
	dev_parm[11] = "0.5";		
	dev_parm[12] = "1";		
	dev_parm[13] = "1";		
	dev_parm[14] = " ";	
	dev_parm[15] = "1";
	dev_parm[16] = "0.1";
	dev_parm[17] = "0.156";
	*/

	//	dev_parm[ 0]	: thyristor 'on' conductance, g_on(Mhos)
	//	dev_parm[ 1]    : thyristor 'off' conductance, g_off(micMhos)
	//	dev_parm[ 2]    : thyristor parasitic capacitance, Cp(nanoFarads)
	//	dev_parm[ 3]    : snubber circuit capacitance, C_s(microFarads)
	//	dev_parm[ 4]    : snubber circuit resistance, R_s(ohms)
	//	dev_parm[ 5]    : current rate limiting inductance, L(millihenries)
	//	dev_parm[ 6]    : resistor in parallel with reactor, r(ohms)
	//	dev_parm[ 7]    : DC side smoothing capacitor, C(microFarads)
	//	dev_parm[ 8]    : rated voltage (kV, L-L, rms)
	//	dev_parm[ 9]    : DC Voltage setpoint (kV)
	//	dev_parm[10]	: connected DSP device
	//	dev_parm[11]	: logic start time
	//	dev_parm[12]	: converter start time

	
	nd_parm = 13;
	if (dev_parm) delete[]	dev_parm;
	dev_parm = new CString[nd_parm];

	dev_parm[0] = "100.0";
	dev_parm[1] = "0.1";
	dev_parm[2] = "20.0";
	dev_parm[3] = "0.1";
	dev_parm[4] = "100.0";
	dev_parm[5] = "100.0";
	dev_parm[6] = "3.0";
	dev_parm[7] = "500.0";
	dev_parm[8] = "13.8";
	dev_parm[9] = "0.5";
	dev_parm[10] = " ";
	dev_parm[11] = "0.1";
	dev_parm[12] = "0.156";
	

	SetVicon();
}
//-------------------------------------------------------------------
BOOL M801::GuiToInternal()
{
	int i;

	//------convert parameters from ASCII to internal format

	nd_iparm = nd_parm;
	if( dev_iparm ) delete [] dev_iparm;
	dev_iparm = new double [nd_iparm];
	for ( i=0 ; i<nd_iparm ; i++ )
	{
		dev_iparm[i] = atof(dev_parm[i]);
	}
	/*
	dValveONConductance		= atof(dev_parm[0]);
	dValveOffConductance	= atof(dev_parm[1]) * 1.0e-6;
	dParasiticCapacitance	= atof(dev_parm[2]) * 1.0e-9;
	dSnubberCapacitance		= atof(dev_parm[3]) * 1.0e-6;
	dSnubberConductance		= 1 / atof(dev_parm[4]);
	dCLInductance			= atof(dev_parm[5]) * 1.0e-3;
	dCLConductance			= 1 / atof(dev_parm[6]);
	dSmoothingCapacitance	= atof(dev_parm[7]) * 1.0e-6;
	dFiringAngle			= atof(dev_parm[9]) / 180 * DPI;
	dRealPowerDC			= atof(dev_parm[10]);

	dev_iparm[0]	= dValveONConductance	;
	dev_iparm[1]	= dValveOffConductance	;
	dev_iparm[2]	= dParasiticCapacitance	;
	dev_iparm[3]	= dSnubberCapacitance	;
	dev_iparm[4]	= dSnubberConductance	;
	dev_iparm[5]	= dCLInductance			;
	dev_iparm[6]	= dCLConductance		;
	dev_iparm[7]	= dSmoothingCapacitance	;
	dev_iparm[9]	= dFiringAngle			;
	dev_iparm[10]	= dRealPowerDC			;
	*/

	
	dValveONConductance		= atof(dev_parm[0]);
	dValveOffConductance	= atof(dev_parm[1]) * 1.0e-6;
	dParasiticCapacitance	= atof(dev_parm[2]) * 1.0e-9;
	dSnubberCapacitance		= atof(dev_parm[3]) * 1.0e-6;
	dSnubberConductance		= 1 / atof(dev_parm[4]);
	dCLInductance			= atof(dev_parm[5]) * 1.0e-3;
	dCLConductance			= 1 / atof(dev_parm[6]);
	dSmoothingCapacitance	= atof(dev_parm[7]) * 1.0e-6;

	dev_iparm[0]	= dValveONConductance	;
	dev_iparm[1]	= dValveOffConductance	;
	dev_iparm[2]	= dParasiticCapacitance	;
	dev_iparm[3]	= dSnubberCapacitance	;
	dev_iparm[4]	= dSnubberConductance	;
	dev_iparm[5]	= dCLInductance			;
	dev_iparm[6]	= dCLConductance		;
	dev_iparm[7]	= dSmoothingCapacitance	;
	

	//------assign internal nodes

	//nd_iterm = 9;
	nd_iterm = 8;
	if( dev_iterm ) delete [] dev_iterm;
	dev_iterm = new CString [nd_iterm];
	devnod = nd_iterm;

	NODE_NAMES bus_node;
	if ( !pNetSolver->MakePrimaryBusNodes(dev_term[0], bus_node) ) return false;
	i = 0;
	dev_iterm[i++] = bus_node.A;
	dev_iterm[i++] = bus_node.B;
	//dev_iterm[i++] = bus_node.C;
	//NEW delete this line

	if ( !pNetSolver->MakeBusNodeNames(dev_term[1], bus_node) ) return false;
	dev_iterm[i++] = bus_node.AD;
	dev_iterm[i++] = bus_node.KD;

	dev_iterm[i++] = dev_term[2];
	dev_iterm[i++] = dev_term[3];
	dev_iterm[i++] = dev_term[4];
	dev_iterm[i++] = dev_term[5];

	//------allocate memory for optimal node number array

	nd_oterm = nd_iterm;
	if( dev_oterm ) delete [] dev_oterm;
	dev_oterm = new int [nd_oterm];

	//------transfer circuit names to internal circuit numbers

	nd_icirc = nd_circ;
	if( dev_icirc ) delete [] dev_icirc;
	dev_icirc = new int [nd_icirc];
	for ( i=0 ; i<nd_icirc ; i++ ) dev_icirc[i] = atoi(dev_circ[i]);

	//------define internal state names

	switch ( pNetSolver->iSolverSelect )
	{
	case 1:							//	FD-Multiphase Asymmetric
		nd_iista = 7;
		break;
//	case 2:							//	FD-Positive Sequence
//		nd_iista = 0;
//		break;
//	case 3:							//	FD-Symmetric Components
//		nd_iista = 0;
//		break;
	case 4:							//	Trapezoidal Time Domain
//		nd_iista = 20;
//		break;
	case 5:							//	Quadratic Time Domain
		nd_iista = 18;
		break;
	case 6:							//	Quasistatic Time Domain
		nd_iista = 7;
		break;
	default:
		pNetSolver->sErrorMessage = "Unsupported Solver Selection in Model 801";
		return false;
		break;
	}

	nd_oista = nd_iista;
	devsta	 = nd_oterm + nd_oista;
	MakeInternalNodeNames();
	
	//------allocate memory for optimal internal-state number array
	
	if( dev_oista ) delete [] dev_oista;
	if ( nd_oista > 0 )	dev_oista = new int [nd_oista];
	else dev_oista = NULL;
	//amsr1.Format("nd_iista= %d\n",nd_iista);
	//AfxMessageBox(amsr1);

	return true;
}
//-------------------------------------------------------------------
BOOL M801::InternalToOptimal()
{
	int i;

	//------transfer parameters

	nd_oparm = nd_iparm;
	if( dev_oparm ) delete [] dev_oparm;
	dev_oparm = new double [nd_oparm];
	for ( i=0 ; i<nd_oparm ; i++ )
	{
		dev_oparm[i] = dev_iparm[i];
	}

	//------transfer circuit numbers

	nd_ocirc = nd_icirc;
	if( dev_ocirc ) delete [] dev_ocirc;
	dev_ocirc = new int [nd_ocirc];
	for ( i=0 ; i<nd_ocirc ; i++ )
	{
		dev_ocirc[i] = dev_icirc[i];
	}

	//------allocate and fill optimal combined (external+internal) node number array

	devsta = nd_oterm + nd_oista;
	if( devosd ) delete [] devosd;
	devosd = new int [devsta];
	for ( i=0 ; i<nd_oterm ; i++ )
	{
		devosd[i] = dev_oterm[i];
	}
	for ( i=0 ; i<nd_oista ; i++ )
	{
		devosd[i + nd_oterm] = dev_oista[i];
	}
	Update_devosd_q();

	return true;
}
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
//	Device Properties Dialog Functions
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
BOOL M801::DeviceProperties()
{
	switch ( program_code )
	{
	case PCODE_IGS:
	case PCODE_IGSF:
	case PCODE_IGST:
	case PCODE_IGSQ:
		return DevicePropertiesIGS();
		break;
	case PCODE_HMS:
		break;
	case PCODE_PSA:
		break;
	case PCODE_GEMI:
		break;
	case PCODE_MGRD:
		return DevicePropertiesIGS();
		break;
	case PCODE_PQLITE:
		break;
	}

	AfxMessageBox("Model 801 Device Properties not Available");
	return false;
}
//-------------------------------------------------------------------
void M801_SelectDevice(CxDialog* dlg,void* p)
{	
	int i;
	Device*		part;
	M801* pThis = (M801*)p;
	int	nUserDevices = pThis->pNetSolver->nUserDevices;
	vector <Device*>& pUserDevices801 =pThis-> pNetSolver->pUserDevices;

	pThis->pNetSolver->Form_DEV_SEL(pThis->MeasureDeviceIDM801,NULL);
	for(i=0 ; i<nUserDevices ; i++ )
	{
		part=pUserDevices801 [i];
		if( part->dev_nurid == pThis->MeasureDeviceIDM801  )
		{
			dlg->pButton[5]->caption = part->dev_title;
		}
	}
}

/*
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#define B_DSP 8
#define EB_AC_BUS 5
#define EB_CIRCUIT 3
#define EB_CONVT_START_TIME 21
#define EB_DC_BUS 20
#define EB_DC_C 12
#define EB_DC_VOLTAGE 8
#define EB_FIRING_ANGLE 6
#define EB_FREQ 19
#define EB_LIMIT_L 14
#define EB_LIMIT_R 13
#define EB_LOGIC_START_TIME 15
#define EB_POS_MAG 17
#define EB_POWER 18
#define EB_RATED_VOLTAGE 4
#define EB_REAL_POWER 7
#define EB_SNUBBER_C 1
#define EB_SNUBBER_R 2
#define EB_THYRISTOR_C 11
#define EB_THYRISTOR_OFF 10
#define EB_THYRISTOR_ON 9
#define EB_TITLE 0
#define EB_ZERO_XING 16
//-----------------------------------------------------------------------------
static int b_firing[] = {0,1,2,-1};
static int b_oper[] = {3,4,-1};
static int rb_pf_ctrl[] = {5,6,7,-1};
*/

//-----------------------------------------------------------------------------
#define B_DSP 0
#define EB_AC_BUS 5
#define EB_CIRCUIT 3
#define EB_CONVT_START_TIME 19
#define EB_DC_BUS 16
#define EB_DC_C 12
#define EB_DC_VOLTAGE 6
#define EB_FREQ 17
#define EB_LIMIT_L 14
#define EB_LIMIT_R 13
#define EB_LOGIC_START_TIME 15
#define EB_POS_MAG 18
#define EB_POWER 7
#define EB_RATED_VOLTAGE 4
#define EB_SNUBBER_C 1
#define EB_SNUBBER_R 2
#define EB_THYRISTOR_C 11
#define EB_THYRISTOR_OFF 10
#define EB_THYRISTOR_ON 9
#define EB_TITLE 0
#define EB_ZERO_XING 8
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/*
BOOL M801::DevicePropertiesIGS()
{
	CxDialog dlg;

	dlg.sFormFile = GetDevicePath() + "M801\\IGS_M801.DFE";
	dlg.iHelpTopic = dev_ncode;

	if ( !dlg.PrepareControls() ) return false;

	//-------- transfer parameters to dialog

	dlg.pCaller			= this;
	dlg.sProgramName	= GetProgramName();

	dlg.pEditBox[EB_TITLE			]->text = dev_title;
	dlg.pEditBox[EB_THYRISTOR_ON	]->text = dev_parm[ 0];		// Thyristor on conductance (mhos)
	dlg.pEditBox[EB_THYRISTOR_OFF	]->text = dev_parm[ 1];		// Thyristor off conductance (micro-mhos)
	dlg.pEditBox[EB_THYRISTOR_C		]->text = dev_parm[ 2];		// Thyristor Parasitic Capacitance (nanoFarads)
	dlg.pEditBox[EB_SNUBBER_C		]->text = dev_parm[ 3];		// Snubber Capacitance (microFarads)
	dlg.pEditBox[EB_SNUBBER_R		]->text = dev_parm[ 4];		// Snubber Resistance (Ohms)
	dlg.pEditBox[EB_LIMIT_L			]->text = dev_parm[ 5];		// Current Limiting Reactance (miliHenries)
	dlg.pEditBox[EB_LIMIT_R			]->text = dev_parm[ 6];		// Resistor Across Above Inductor (Ohms)
	dlg.pEditBox[EB_DC_C			]->text = dev_parm[ 7];		// DC Smoothing Capacitor (microFarads)
	dlg.pEditBox[EB_RATED_VOLTAGE	]->text = dev_parm[ 8];		// Voltage Rating (kV, L-L, rms)
	dlg.pEditBox[EB_FIRING_ANGLE	]->text = dev_parm[ 9];		// Firing Angle (Degrees)
	dlg.pEditBox[EB_REAL_POWER		]->text = dev_parm[10];		// Real Power (MW)
	dlg.pEditBox[EB_DC_VOLTAGE		]->text = dev_parm[11];		// Voltage Magnitude (kV)
	dlg.pEditBox[EB_LOGIC_START_TIME]->text = dev_parm[16];		// logic start time
	dlg.pEditBox[EB_CONVT_START_TIME]->text = dev_parm[17];		// converter start time

	dlg.pButton[B_DSP]->ButtonFunc	= M801_SelectDevice;			// Connected DSP selection
	dlg.pButton[B_DSP]->caption		= dev_parm[14];

	if		( dev_parm[13] == "1" ) dlg.pButton[3]->button_state = 1;
	else if ( dev_parm[13] == "2" ) dlg.pButton[4]->button_state = 1;

	if		( dev_parm[12] == "1" ) dlg.pButton[0]->button_state = 1;
	else if ( dev_parm[12] == "2" ) dlg.pButton[1]->button_state = 1;
	else if ( dev_parm[12] == "3" ) dlg.pButton[2]->button_state = 1;

	if		( dev_parm[15] == "1" ) dlg.pButton[5]->button_state = 1;
	else if ( dev_parm[15] == "2" ) dlg.pButton[6]->button_state = 1;
	else if ( dev_parm[15] == "3" ) dlg.pButton[7]->button_state = 1;

	dlg.pEditBox[EB_AC_BUS		]->text = dev_term[0];		// bus name, AC side
	dlg.pEditBox[EB_DC_BUS		]->text = dev_term[1];		// bus name, DC side
	dlg.pEditBox[EB_ZERO_XING	]->text = dev_term[2];		// node name Zero Crossing
	dlg.pEditBox[EB_POS_MAG		]->text = dev_term[3];		// node name Magnitude
	dlg.pEditBox[EB_POWER		]->text = dev_term[4];		// node name Power
	dlg.pEditBox[EB_FREQ		]->text = dev_term[5];		// node name Frequency
	dlg.pEditBox[EB_CIRCUIT		]->text = dev_circ[0];

	//-------- run the dialog

	if ( dlg.DoModal() != IDOK ) return false;

	//-------- retrieve edited parameters from dialog

	dev_title	 = dlg.pEditBox[EB_TITLE]->text;

	dev_parm[ 0] = dlg.pEditBox[EB_THYRISTOR_ON	]->text;
	dev_parm[ 1] = dlg.pEditBox[EB_THYRISTOR_OFF]->text; 
	dev_parm[ 2] = dlg.pEditBox[EB_THYRISTOR_C	]->text;  
	dev_parm[ 3] = dlg.pEditBox[EB_SNUBBER_C	]->text;
	dev_parm[ 4] = dlg.pEditBox[EB_SNUBBER_R	]->text;
	dev_parm[ 5] = dlg.pEditBox[EB_LIMIT_L		]->text;
	dev_parm[ 6] = dlg.pEditBox[EB_LIMIT_R		]->text;
	dev_parm[ 7] = dlg.pEditBox[EB_DC_C			]->text;
	dev_parm[ 8] = dlg.pEditBox[EB_RATED_VOLTAGE]->text;
	dev_parm[ 9] = dlg.pEditBox[EB_FIRING_ANGLE	]->text;
	dev_parm[10] = dlg.pEditBox[EB_REAL_POWER	]->text;
	dev_parm[11] = dlg.pEditBox[EB_DC_VOLTAGE	]->text;
	dev_parm[14] = dlg.pButton [B_DSP			]->caption; 
	dev_parm[16] = dlg.pEditBox[EB_LOGIC_START_TIME]->text;
	dev_parm[17] = dlg.pEditBox[EB_CONVT_START_TIME]->text;

	if		( dlg.pButton[3]->button_state ) dev_parm[13] = "1";
	else if ( dlg.pButton[4]->button_state ) dev_parm[13] = "2";

	if		( dlg.pButton[0]->button_state ) dev_parm[12] = "1";
	else if ( dlg.pButton[1]->button_state ) dev_parm[12] = "2";
	else if ( dlg.pButton[2]->button_state ) dev_parm[12] = "3";

	if		( dlg.pButton[5]->button_state ) dev_parm[15] = "1";
	else if ( dlg.pButton[6]->button_state ) dev_parm[15] = "2";
	else if ( dlg.pButton[7]->button_state ) dev_parm[15] = "3";

	dev_term[0] = dlg.pEditBox[EB_AC_BUS	]->text;
	dev_term[1] = dlg.pEditBox[EB_DC_BUS	]->text;
	dev_term[2] = dlg.pEditBox[EB_ZERO_XING	]->text;
	dev_term[3] = dlg.pEditBox[EB_POS_MAG	]->text;
	dev_term[4] = dlg.pEditBox[EB_POWER		]->text;
	dev_term[5] = dlg.pEditBox[EB_FREQ		]->text;
	
	dev_circ[0] = dlg.pEditBox[EB_CIRCUIT	]->text;

	SetVicon();

	return true;
}*/

BOOL M801::DevicePropertiesIGS()
{
	CxDialog dlg;

	dlg.sFormFile = GetDevicePath() + "M801\\IGS_M801.DFE";
	dlg.iHelpTopic = dev_ncode;

	if (!dlg.PrepareControls()) return false;

	//-------- transfer parameters to dialog

	dlg.pCaller = this;
	dlg.sProgramName = GetProgramName();

	dlg.pEditBox[EB_TITLE]->text = dev_title;
	dlg.pEditBox[EB_THYRISTOR_ON]->text = dev_parm[0];		// Thyristor on conductance (mhos)
	dlg.pEditBox[EB_THYRISTOR_OFF]->text = dev_parm[1];		// Thyristor off conductance (micro-mhos)
	dlg.pEditBox[EB_THYRISTOR_C]->text = dev_parm[2];		// Thyristor Parasitic Capacitance (nanoFarads)
	dlg.pEditBox[EB_SNUBBER_C]->text = dev_parm[3];		// Snubber Capacitance (microFarads)
	dlg.pEditBox[EB_SNUBBER_R]->text = dev_parm[4];		// Snubber Resistance (Ohms)
	dlg.pEditBox[EB_LIMIT_L]->text = dev_parm[5];		// Current Limiting Reactance (miliHenries)
	dlg.pEditBox[EB_LIMIT_R]->text = dev_parm[6];		// Resistor Across Above Inductor (Ohms)
	dlg.pEditBox[EB_DC_C]->text = dev_parm[7];		// DC Smoothing Capacitor (microFarads)
	dlg.pEditBox[EB_RATED_VOLTAGE]->text = dev_parm[8];		// Voltage Rating (kV, L-L, rms)
	dlg.pEditBox[EB_DC_VOLTAGE]->text = dev_parm[9];		// Voltage Magnitude (kV)
	dlg.pEditBox[EB_LOGIC_START_TIME]->text = dev_parm[11];		// logic start time
	dlg.pEditBox[EB_CONVT_START_TIME]->text = dev_parm[12];		// converter start time

	dlg.pButton[B_DSP]->ButtonFunc = M801_SelectDevice;			// Connected DSP selection
	dlg.pButton[B_DSP]->caption = dev_parm[10];

	dlg.pEditBox[EB_AC_BUS]->text = dev_term[0];		// bus name, AC side
	dlg.pEditBox[EB_DC_BUS]->text = dev_term[1];		// bus name, DC side
	dlg.pEditBox[EB_ZERO_XING]->text = dev_term[2];		// node name Zero Crossing
	dlg.pEditBox[EB_POS_MAG]->text = dev_term[3];		// node name Magnitude
	dlg.pEditBox[EB_POWER]->text = dev_term[4];		// node name Power
	dlg.pEditBox[EB_FREQ]->text = dev_term[5];		// node name Frequency
	dlg.pEditBox[EB_CIRCUIT]->text = dev_circ[0];

	//-------- run the dialog

	if (dlg.DoModal() != IDOK) return false;

	//-------- retrieve edited parameters from dialog

	dev_title = dlg.pEditBox[EB_TITLE]->text;

	dev_parm[0] = dlg.pEditBox[EB_THYRISTOR_ON]->text;
	dev_parm[1] = dlg.pEditBox[EB_THYRISTOR_OFF]->text;
	dev_parm[2] = dlg.pEditBox[EB_THYRISTOR_C]->text;
	dev_parm[3] = dlg.pEditBox[EB_SNUBBER_C]->text;
	dev_parm[4] = dlg.pEditBox[EB_SNUBBER_R]->text;
	dev_parm[5] = dlg.pEditBox[EB_LIMIT_L]->text;
	dev_parm[6] = dlg.pEditBox[EB_LIMIT_R]->text;
	dev_parm[7] = dlg.pEditBox[EB_DC_C]->text;
	dev_parm[8] = dlg.pEditBox[EB_RATED_VOLTAGE]->text;
	dev_parm[9] = dlg.pEditBox[EB_DC_VOLTAGE]->text;
	dev_parm[10] = dlg.pButton[B_DSP]->caption;
	dev_parm[11] = dlg.pEditBox[EB_LOGIC_START_TIME]->text;
	dev_parm[12] = dlg.pEditBox[EB_CONVT_START_TIME]->text;

	dev_term[0] = dlg.pEditBox[EB_AC_BUS]->text;
	dev_term[1] = dlg.pEditBox[EB_DC_BUS]->text;
	dev_term[2] = dlg.pEditBox[EB_ZERO_XING]->text;
	dev_term[3] = dlg.pEditBox[EB_POS_MAG]->text;
	dev_term[4] = dlg.pEditBox[EB_POWER]->text;
	dev_term[5] = dlg.pEditBox[EB_FREQ]->text;

	dev_circ[0] = dlg.pEditBox[EB_CIRCUIT]->text;

	SetVicon();

	return true;
}
//-------------------------------------------------------------------