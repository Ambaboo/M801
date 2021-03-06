//-------------------------------------------------------------------
//
//		file: wmaster\devices\m801\m801.h
//
//-------------------------------------------------------------------
#ifndef AGC_M801_INCLUDED
#define AGC_M801_INCLUDED
#include "\wmaster\devices\device.h"
#include "\wmaster\devices\multerm.h"

//-------------------------------------------------------------------
//		This class is used to store the 12+1 modes 
//		of the inverter
//-------------------------------------------------------------------
class CPowerDevice_M801
{
public:
	CPowerDevice_M801()
	{
		// ---- Time domain SCQDM device model

		nTDSCQDModel_M801_Equ1					= 0;
		nTDSCQDModel_M801_Equ2					= 0;			
		nTDSCQDModel_M801_Equ3					= 0;	
		nTDSCQDModel_M801_State					= 0;
		nTDSCQDModel_M801_Control				= 0;
		nTDSCQDModel_M801_Feqxx					= 0;
		nTDSCQDModel_M801_Fequu					= 0;
		nTDSCQDModel_M801_Fequx					= 0;
		nTDSCQDModel_M801_Fconstraints			= 0;
		nTDSCQDModel_M801_Ffeqxx				= 0;
		nTDSCQDModel_M801_Ffequu				= 0;
		nTDSCQDModel_M801_Ffequx				= 0;

		pTDSCQDModel_M801_Yeqx1					= NULL;
		pTDSCQDModel_M801_Yequ1					= NULL;
		pTDSCQDModel_M801_Deqxd1				= NULL;
		pTDSCQDModel_M801_Ceqc1					= NULL;
		pTDSCQDModel_M801_Yeqx2					= NULL;
		pTDSCQDModel_M801_Yequ2					= NULL;
		pTDSCQDModel_M801_Deqxd2				= NULL;
		pTDSCQDModel_M801_Ceqc2					= NULL;
		pTDSCQDModel_M801_Yeqx3					= NULL;
		pTDSCQDModel_M801_Yequ3					= NULL;
		pTDSCQDModel_M801_Feqxx3				= NULL;
		pTDSCQDModel_M801_Fequu3				= NULL;
		pTDSCQDModel_M801_Fequx3				= NULL;
		pTDSCQDModel_M801_Ceqc3					= NULL;

		pTDSCQDModel_M801_StateNormFactor		= NULL;
		pTDSCQDModel_M801_ThroughNormFactor1	= NULL;
		pTDSCQDModel_M801_ThroughNormFactor2	= NULL;
		pTDSCQDModel_M801_ThroughNormFactor3	= NULL;
		pTDSCQDModel_M801_ControlNormFactor		= NULL;

		pTDSCQDModel_M801_NodeName				= NULL;
		pTDSCQDModel_M801_OptimalNodeNumber		= NULL;
		pTDSCQDModel_M801_u						= NULL;
		pTDSCQDModel_M801_x						= NULL;

		pTDSCQDModel_M801_Yfeqx					= NULL;
		pTDSCQDModel_M801_Yfequ					= NULL;
		pTDSCQDModel_M801_Cfeqc					= NULL;
		pTDSCQDModel_M801_Ffeqxx				= NULL;
		pTDSCQDModel_M801_Ffequu				= NULL;
		pTDSCQDModel_M801_Ffequx				= NULL;

		pTDSCQDModel_M801_xmin					= NULL;
		pTDSCQDModel_M801_xmax					= NULL;
		pTDSCQDModel_M801_umin					= NULL;
		pTDSCQDModel_M801_umax					= NULL;
		pTDSCQDModel_M801_hmin					= NULL;
		pTDSCQDModel_M801_hmax					= NULL;

		// ---- Time domain SCAQCF device model

		nTDSCAQCFModel_M801_Equ					= 0;
		nTDSCAQCFModel_M801_EquOver2			= 0;
		nTDSCAQCFModel_M801_State				= 0;
		nTDSCAQCFModel_M801_Control				= 0;
		nTDSCAQCFModel_M801_Feqxx				= 0;
		nTDSCAQCFModel_M801_Fequu				= 0;
		nTDSCAQCFModel_M801_Fequx				= 0;
		nTDSCAQCFModel_M801_Feqxxv				= 0;
		nTDSCAQCFModel_M801_Fequuv				= 0;
		nTDSCAQCFModel_M801_Fequxv				= 0;
		nTDSCAQCFModel_M801_Yeqxv_xh			= 0;
		nTDSCAQCFModel_M801_Yeqxv_uh			= 0;
		nTDSCAQCFModel_M801_Yequv_xh			= 0;
		nTDSCAQCFModel_M801_Yequv_uh			= 0;
		nTDSCAQCFModel_M801_Node				= 0;
		nTDSCAQCFModel_M801_Fconstraints		= 0;
		nTDSCAQCFModel_M801_Ffeqxx				= 0;
		nTDSCAQCFModel_M801_Ffequu				= 0;
		nTDSCAQCFModel_M801_Ffequx				= 0;

		pTDSCAQCFModel_M801_Yeqx				= NULL;
		pTDSCAQCFModel_M801_Yequ				= NULL;
		pTDSCAQCFModel_M801_Feqxx				= NULL;
		pTDSCAQCFModel_M801_Fequu				= NULL;
		pTDSCAQCFModel_M801_Fequx				= NULL;
		pTDSCAQCFModel_M801_Neqx				= NULL;
		pTDSCAQCFModel_M801_Nequ				= NULL;
		pTDSCAQCFModel_M801_Meq					= NULL;
		pTDSCAQCFModel_M801_Keq					= NULL;

		pTDSCAQCFModel_M801_Yeqxv_xh			= NULL;
		pTDSCAQCFModel_M801_Yeqxv_uh			= NULL;
		pTDSCAQCFModel_M801_Yequv_xh			= NULL;
		pTDSCAQCFModel_M801_Yequv_uh			= NULL;
		pTDSCAQCFModel_M801_Feqxxv				= NULL;
		pTDSCAQCFModel_M801_Fequuv				= NULL;
		pTDSCAQCFModel_M801_Fequxv				= NULL;

		pTDSCAQCFModel_M801_StateNormFactor		= NULL;
		pTDSCAQCFModel_M801_ThroughNormFactor	= NULL;
		pTDSCAQCFModel_M801_ControlNormFactor	= NULL;

		pTDSCAQCFModel_M801_NodeName			= NULL;
		pTDSCAQCFModel_M801_OptimalNodeNumber	= NULL;
		pTDSCAQCFModel_M801_u					= NULL;
		pTDSCAQCFModel_M801_x					= NULL;

		pTDSCAQCFModel_M801_Yfeqx				= NULL;
		pTDSCAQCFModel_M801_Yfequ				= NULL;
		pTDSCAQCFModel_M801_Cfeqc				= NULL;
		pTDSCAQCFModel_M801_Ffeqxx				= NULL;
		pTDSCAQCFModel_M801_Ffequu				= NULL;
		pTDSCAQCFModel_M801_Ffequx				= NULL;

		pTDSCAQCFModel_M801_xmin				= NULL;
		pTDSCAQCFModel_M801_xmax				= NULL;
		pTDSCAQCFModel_M801_umin				= NULL;
		pTDSCAQCFModel_M801_umax				= NULL;
		pTDSCAQCFModel_M801_hmin				= NULL;
		pTDSCAQCFModel_M801_hmax				= NULL;
	};

	// -------- Time Domain Quadratized Device Model, SCQDM - ANALOG PART

	double**	pTDSCQDModel_M801_Yeqx1;		// pointer to quadratized device model linear state matrix in linear through equations
	double**	pTDSCQDModel_M801_Yequ1;		// pointer to quadratized device model linear control matrix in linear through equations
	double**	pTDSCQDModel_M801_Deqxd1;		// pointer to quadratized device model defferential state matrix in linear through equations
	double*		pTDSCQDModel_M801_Ceqc1;		// pointer to quadratized device model constant in linear through equations

	double**	pTDSCQDModel_M801_Yeqx2;		// pointer to quadratized device model linear state matrix in linear internal equations
	double**	pTDSCQDModel_M801_Yequ2;		// pointer to quadratized device model linear control matrix in linear internal equations
	double**	pTDSCQDModel_M801_Deqxd2;		// pointer to quadratized device model defferential state matrix in linear internal equations
	double*		pTDSCQDModel_M801_Ceqc2;		// pointer to quadratized device model constant in linear internal equations

	double**	pTDSCQDModel_M801_Yeqx3;		// pointer to quadratized device model linear state matrix in nonlinear equations
	double**	pTDSCQDModel_M801_Yequ3;		// pointer to quadratized device model linear control matrix in nonlinear equations
	SP_CUBIX*	pTDSCQDModel_M801_Feqxx3;		// pointer to quadratized device model quadratic state matrix in nonlinear equations
	SP_CUBIX*	pTDSCQDModel_M801_Fequu3;		// pointer to quadratized device model quadratic control matrix in nonlinear equations
	SP_CUBIX*	pTDSCQDModel_M801_Fequx3;		// pointer to quadratized device model quadratic product of state and control matrix in nonlinear equations
	double*		pTDSCQDModel_M801_Ceqc3;		// pointer to quadratized device model constant in nonlinear equations
	
	double**	pTDSCQDModel_M801_Yfeqx;		// pointer to quadratized device model linear state matrix in functional constraints
	double**	pTDSCQDModel_M801_Yfequ;		// pointer to quadratized device model linear control matrix in functional constraints
	SP_CUBIX*	pTDSCQDModel_M801_Ffeqxx;		// pointer to quadratized device model quadratic state matrix in functional constraints
	SP_CUBIX*	pTDSCQDModel_M801_Ffequu;		// pointer to quadratized device model quadratic control matrix in functional constraints
	SP_CUBIX*	pTDSCQDModel_M801_Ffequx;		// pointer to quadratized device model quadratic product of state and control matrix in functional constraints
	double*		pTDSCQDModel_M801_Cfeqc;		// pointer to quadratized device model constant in functional constraints
		
	// -------- Time Domain Quadratized Device Model - x-VARIABLES

	double*		pTDSCQDModel_M801_x;			// state variable
		
	// -------- Time Domain Quadratized Device Model - u-VARIABLES

	double*		pTDSCQDModel_M801_u;			// Values for control variables

	// -------- Time Domain Quadratized Device Model - CONNECTIVITY

	CString*	pTDSCQDModel_M801_NodeName;				// pointer to terminal names
	int*		pTDSCQDModel_M801_OptimalNodeNumber;	// pointer to system optimal node number which device terminals are connected to

	// -------- Time Domain Quadratized Device Model - NORMALIZATION

	double*		pTDSCQDModel_M801_StateNormFactor;		// pointer to state normalization factors
	double*		pTDSCQDModel_M801_ThroughNormFactor1;	// pointer to through variable normalization factors in the first equation set
	double*		pTDSCQDModel_M801_ThroughNormFactor2;	// pointer to through variable normalization factors in the second equation set
	double*		pTDSCQDModel_M801_ThroughNormFactor3;	// pointer to through variable normalization factors in the third equation set
	double*		pTDSCQDModel_M801_ControlNormFactor;	// pointer to control normalization factors
	
	// -------- Time Domain Quadratized Device Model - h-LIMITS

	double*		pTDSCQDModel_M801_hmax;				// h max limit
	double*		pTDSCQDModel_M801_hmin;				// h min limit

	// -------- Time Domain Quadratized Device Model - x-LIMITS

	double*		pTDSCQDModel_M801_xmax;				// state max limit
	double*		pTDSCQDModel_M801_xmin;				// state min limit

	// -------- Time Domain Quadratized Device Model - u-LIMITS

	double*		pTDSCQDModel_M801_umax;				// control max limit
	double*		pTDSCQDModel_M801_umin;				// control min limit

	// -------- Time Domain Quadratized Device Model - DIMENSIONS

	int			nTDSCQDModel_M801_Equ1;				// number of linear through equations
	int			nTDSCQDModel_M801_Equ2;				// number of linear internal equations
	int			nTDSCQDModel_M801_Equ3;				// number of nonlinear internal equations
	int			nTDSCQDModel_M801_State;			// number of states in time domain quadratized model (check if it is equal to Equ1 + Equ2 + Equ3)
	int			nTDSCQDModel_M801_Control;			// number of control variables in time domain quadratized model
	int			nTDSCQDModel_M801_Feqxx;			// number of quadratic terms for states in the third set of equations
	int			nTDSCQDModel_M801_Fequu;			// number of quadratic terms for controls in the third set of equations
	int			nTDSCQDModel_M801_Fequx;			// number of quadratic terms for the product of state and control variable in the third set of equations
	int			nTDSCQDModel_M801_Fconstraints;		// number of functional constraints
	int			nTDSCQDModel_M801_Ffeqxx;			// number of quadratic terms for states in functional constraints
	int			nTDSCQDModel_M801_Ffequu;			// number of quadratic terms for controls in functional constraints
	int			nTDSCQDModel_M801_Ffequx;			// number of quadratic terms for the product of state and control variable in functional constraints

	// -------- Time Domain SCAQCF Device Model - ANALOG PART

	double**	pTDSCAQCFModel_M801_Yeqx;			// pointer to SCAQCF device model linear state matrix
	double**	pTDSCAQCFModel_M801_Yequ;			// pointer to SCAQCF device model linear control matrix
	SP_CUBIX*	pTDSCAQCFModel_M801_Feqxx;			// pointer to SCAQCF device model quadratic state matrix
	SP_CUBIX*	pTDSCAQCFModel_M801_Fequu;			// pointer to SCAQCF device model quadratic control matrix
	SP_CUBIX*	pTDSCAQCFModel_M801_Fequx;			// pointer to SCAQCF device model quadratic product of state and control matrix 
	double**	pTDSCAQCFModel_M801_Neqx;			// pointer to SCAQCF device model linear past history state matrix 
	double**	pTDSCAQCFModel_M801_Nequ;			// pointer to SCAQCF device model linear past history control matrix 
	double**	pTDSCAQCFModel_M801_Meq;			// pointer to SCAQCF device model linear past history through viariable matrix
	double*		pTDSCAQCFModel_M801_Keq;			// pointer to SCAQCF device model constant part

	SP_CUBIX*	pTDSCAQCFModel_M801_Yeqxv_xh;		// pointer to SCAQCF device model time varying linear state matrix (consist of x(t-h))
	SP_CUBIX*	pTDSCAQCFModel_M801_Yeqxv_uh;		// pointer to SCAQCF device model time varying linear state matrix (consist of u(t-h))
	SP_CUBIX*	pTDSCAQCFModel_M801_Yequv_xh;		// pointer to SCAQCF device model time varying linear control matrix (consist of x(t-h))
	SP_CUBIX*	pTDSCAQCFModel_M801_Yequv_uh;		// pointer to SCAQCF device model time varying linear control matrix (consist of u(t-h))
	SP_CUBIX*	pTDSCAQCFModel_M801_Feqxxv;			// pointer to SCAQCF device model time varying quadratic state matrix
	SP_CUBIX*	pTDSCAQCFModel_M801_Fequuv;			// pointer to SCAQCF device model time varying quadratic control matrix
	SP_CUBIX*	pTDSCAQCFModel_M801_Fequxv;			// pointer to SCAQCF device model time varying quadratic product of state and control matrix 

	double**	pTDSCAQCFModel_M801_Yfeqx;			// pointer to SCAQCF device model linear state matrix in functional constraints
	double**	pTDSCAQCFModel_M801_Yfequ;			// pointer to SCAQCF device model linear control matrix in functional constraints
	SP_CUBIX*	pTDSCAQCFModel_M801_Ffeqxx;			// pointer to SCAQCF device model quadratic state matrix in functional constraints
	SP_CUBIX*	pTDSCAQCFModel_M801_Ffequu;			// pointer to SCAQCF device model quadratic control matrix in functional constraints
	SP_CUBIX*	pTDSCAQCFModel_M801_Ffequx;			// pointer to SCAQCF device model quadratic product of state and control matrix in functional constraints
	double*		pTDSCAQCFModel_M801_Cfeqc;			// pointer to SCAQCF device model constant in functional constraints

	// -------- Time Domain SCAQCF Device Model - x-VARIABLES

	double*		pTDSCAQCFModel_M801_x;				// state variables

	// -------- Time Domain SCAQCF Device Model - u-VARIABLES

	double*		pTDSCAQCFModel_M801_u;				// Values for control variables

	// -------- Time Domain SCAQCF Device Model - CONNECTIVITY

	CString*	pTDSCAQCFModel_M801_NodeName;			// pointer to terminal names
	int*		pTDSCAQCFModel_M801_OptimalNodeNumber;	// pointer to system optimal node number which device terminals are connected to

	// -------- Time Domain SCAQCF Device Model - NORMALIZATION

	double*		pTDSCAQCFModel_M801_StateNormFactor;	// pointer to state normalization factors
	double*		pTDSCAQCFModel_M801_ThroughNormFactor;	// pointer to through variable normalization factors
	double*		pTDSCAQCFModel_M801_ControlNormFactor;	// pointer to control normalization factors

	// -------- Time Domain SCAQCF Device Model - h-LIMITS

	double*		pTDSCAQCFModel_M801_hmax;				// h max limit
	double*		pTDSCAQCFModel_M801_hmin;				// h min limit

	// -------- Time Domain SCAQCF Device Model - x-LIMITS

	double*		pTDSCAQCFModel_M801_xmax;				// state max limit
	double*		pTDSCAQCFModel_M801_xmin;				// state min limit

	// -------- Time Domain SCAQCF Device Model - u-LIMITS

	double*		pTDSCAQCFModel_M801_umax;				// control max limit
	double*		pTDSCAQCFModel_M801_umin;				// control min limit

	// -------- Time Domain SCAQCF Device Model - DIMENSIONS

	int			nTDSCAQCFModel_M801_Node;				// number of nodes (time t only)
	int			nTDSCAQCFModel_M801_Equ;				// number of equations (time t and tm)
	int			nTDSCAQCFModel_M801_EquOver2;			// number of equations (time t only)
	int			nTDSCAQCFModel_M801_State;				// number of states (time t and tm)
	int			nTDSCAQCFModel_M801_Control;			// number of control variables (time t and tm)
	int			nTDSCAQCFModel_M801_Feqxx;				// number of quadratic terms for states (time t and tm)
	int			nTDSCAQCFModel_M801_Fequu;				// number of quadratic terms for controls (time t and tm)
	int			nTDSCAQCFModel_M801_Fequx;				// number of quadratic terms for the product of state and control variable (time t and tm)
	int			nTDSCAQCFModel_M801_Feqxxv;				// number of time varying quadratic terms for states (time t and tm)
	int			nTDSCAQCFModel_M801_Fequuv;				// number of time varying quadratic terms for controls (time t and tm)
	int			nTDSCAQCFModel_M801_Fequxv;				// number of time varying quadratic terms for the product of state and control variable (time t and tm)
	int			nTDSCAQCFModel_M801_Yeqxv_xh;			// number of time varying Yeqxv_xh (time t and tm)
	int			nTDSCAQCFModel_M801_Yeqxv_uh;			// number of time varying Yeqxv_uh (time t and tm)
	int			nTDSCAQCFModel_M801_Yequv_xh;			// number of time varying Yequv_xh (time t and tm)
	int			nTDSCAQCFModel_M801_Yequv_uh;			// number of time varying Yequv_uh (time t and tm)
	int			nTDSCAQCFModel_M801_Fconstraints;		// number of functional constraints
	int			nTDSCAQCFModel_M801_Ffeqxx;				// number of quadratic terms for states in functional constraints
	int			nTDSCAQCFModel_M801_Ffequu;				// number of quadratic terms for controls in functional constraints
	int			nTDSCAQCFModel_M801_Ffequx;				// number of quadratic terms for the product of state and control variable in functional constraints

};
class M801 : public MulTerm
{
public:

//----------------------member functions-----------------------------

	M801(void* pDoc);
	~M801();

	//------virtual functions

	virtual	BOOL GuiToInternal();
	virtual	BOOL InternalToOptimal();
	virtual void SetDefaultValues();
	virtual void SetVicon();

	//------device properties

	virtual BOOL DeviceProperties();
			BOOL DevicePropertiesIGS();


			BOOL UpdateConverterRunTimeNormRef_801();
			BOOL CalculateFiringAngle_801();
			BOOL ComputeValveScheduleTimes_801();
			BOOL ComputeValveSwitchingTimes_801();
			BOOL TimeUpdateValveStatus_801();
			BOOL ComputeValveState_801();
			BOOL CalculateFiringAngleINV_801();
			BOOL ComputeValveSwitchingTimesINV_801();

	//------time domain analysis - Quadratic Integration ------------

	virtual BOOL TQN_TimeDomainModelInit();
	virtual BOOL TQN_TimeDomainModelTimeStep();
				 
	// ---- test function

			BOOL TQN_TimeDomainQDMModelInit();

	// ---- 

			BOOL TQN_UpdatePastHistory();
			BOOL TQN_SingleValveModelStatusOnModel();
			BOOL TQN_SingleValveModelStatusOffModel();
			BOOL TQN_SingleCapacitorModel();
			BOOL TQN_CreateConverterSCAQCFModel();
			BOOL TQN_TimeDomainQM_SCAQCFModel_M801(int iModel);
			BOOL TQN_PrepareAQCFConverterModel();
			BOOL TQN_CopyAQCFModelM801toAQCFStandard(int iMode);

//	// ---- SCQDM single valve ON and OFF model
//
//	void		Init_SCQDM_ValveON();							// SCQDM ValveON model Initialization
//	void		Init_SCQDM_ValveOFF();							// SCQDM ValveOFF model Initialization
//	void		Clear_SCQDM_ValveON();							// SCQDM ValveON model clear
//	void		Clear_SCQDM_ValveOFF();							// SCQDM ValveOFF model clear
//
//	double**	pTDSCQDM_ValveON_Yeqx1;							// pointer to SCQDM ValveON model linear state matrix in linear through equations
//	double**	pTDSCQDM_ValveON_Yequ1;							// pointer to SCQDM ValveON model control state matrix in linear through //equations
//	double**	pTDSCQDM_ValveON_Deqxd1;						// pointer to SCQDM ValveON model defferential state matrix in linear through //equations
//	double*		pTDSCQDM_ValveON_Ceqc1;							// pointer to SCQDM ValveON model constant in linear through equations
//
//	double**	pTDSCQDM_ValveON_Yeqx2;							// pointer to SCQDM ValveON model linear state matrix in linear internal //equations
//	double**	pTDSCQDM_ValveON_Yequ2;							// pointer to SCQDM ValveON model control state matrix in linear internal //equations
//	double**	pTDSCQDM_ValveON_Deqxd2;						// pointer to SCQDM ValveON model defferential state matrix in linear internal //equations
//	double*		pTDSCQDM_ValveON_Ceqc2;							// pointer to SCQDM ValveON model constant in linear internal equations
//
//	double**	pTDSCQDM_ValveON_Yeqx3;							// pointer to SCQDM ValveON model linear state matrix in nonlinear equations
//	double**	pTDSCQDM_ValveON_Yequ3;							// pointer to SCQDM ValveON model control state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveON_Feqxx3;						// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveON_Fequu3;						// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveON_Fequx3;						// pointer to SCQDM ValveON model quadratic state matrix in nonlinear equations
//	double	*	pTDSCQDM_ValveON_Ceqc3;							// pointer to SCQDM ValveON model constant in nonlinear equations
//
//	int			nTDSCQDM_ValveON_Equ1;							// number of SCQDM ValveON model linear through equations
//	int			nTDSCQDM_ValveON_Equ2;							// number of SCQDM ValveON model linear internal equations
//	int			nTDSCQDM_ValveON_Equ3;							// number of SCQDM ValveON model nonlinear internal equations
//	int			nTDSCQDM_ValveON_State;							// number of SCQDM ValveON model states in time domain quadratized model
//	int			nTDSCQDM_ValveON_Control;						// number of SCQDM ValveON model control variables in time domain quadratized //model
//	int			nTDSCQDM_ValveON_Feqxx;							// number of SCQDM ValveON model quadratic terms for states in the third set of //equations
//	int			nTDSCQDM_ValveON_Fequu;							// number of SCQDM ValveON model quadratic terms for controls in the third set /of/ equations
//	int			nTDSCQDM_ValveON_Fequx;							// number of SCQDM ValveON model quadratic terms for the product of state and //control variable in the third set of equations
//
//	CString*	pTDSCQDM_ValveON_NodeName;						// pointer to SCQDM ValveON model connectivity vectors
//	double*		pTDSCQDM_ValveON_StateNormFactor;				// pointer to SCQDM ValveON model state normalization factors
//	double*		pTDSCQDM_ValveON_ThroughNormFactor1;			// pointer to SCQDM ValveON model through variable normalization factors in the //first equation set
//	double*		pTDSCQDM_ValveON_ThroughNormFactor2;			// pointer to SCQDM ValveON model through variable normalization factors in the //second equation set
//	double*		pTDSCQDM_ValveON_ThroughNormFactor3;			// pointer to SCQDM ValveON model through variable normalization factors in the //third equation set
//	double*		pTDSCQDM_ValveON_ControlNormFactor;				// pointer to SCQDM ValveON model control normalization factors
//
//	double**	pTDSCQDM_ValveOFF_Yeqx1;						// pointer to SCQDM ValveOFF model linear state matrix in linear through //equations
//	double**	pTDSCQDM_ValveOFF_Yequ1;						// pointer to SCQDM ValveOFF model control state matrix in linear through //equations
//	double**	pTDSCQDM_ValveOFF_Deqxd1;						// pointer to SCQDM ValveOFF model defferential state matrix in linear through //equations
//	double*		pTDSCQDM_ValveOFF_Ceqc1;						// pointer to SCQDM ValveOFF model constant in linear through equations
//
//	double**	pTDSCQDM_ValveOFF_Yeqx2;						// pointer to SCQDM ValveOFF model linear state matrix in linear internal //equations
//	double**	pTDSCQDM_ValveOFF_Yequ2;						// pointer to SCQDM ValveOFF model control state matrix in linear internal //equations
//	double**	pTDSCQDM_ValveOFF_Deqxd2;						// pointer to SCQDM ValveOFF model defferential state matrix in linear internal //equations
//	double*		pTDSCQDM_ValveOFF_Ceqc2;						// pointer to SCQDM ValveOFF model constant in linear internal equations
//
//	double**	pTDSCQDM_ValveOFF_Yeqx3;						// pointer to SCQDM ValveOFF model linear state matrix in nonlinear equations
//	double**	pTDSCQDM_ValveOFF_Yequ3;						// pointer to SCQDM ValveOFF model control state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveOFF_Feqxx3;						// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveOFF_Fequu3;						// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_ValveOFF_Fequx3;						// pointer to SCQDM ValveOFF model quadratic state matrix in nonlinear equations
//	double*		pTDSCQDM_ValveOFF_Ceqc3;						// pointer to SCQDM ValveOFF model constant in nonlinear equations
//
//	CString*	pTDSCQDM_ValveOFF_NodeName;						// pointer to SCQDM ValveOFF model connectivity vectors
//	double*		pTDSCQDM_ValveOFF_StateNormFactor;				// pointer to SCQDM ValveOFF model state normalization factors
//	double*		pTDSCQDM_ValveOFF_ThroughNormFactor1;			// pointer to SCQDM ValveOFF model through variable normalization factors in /the /first equation set
//	double*		pTDSCQDM_ValveOFF_ThroughNormFactor2;			// pointer to SCQDM ValveOFF model through variable normalization factors in /the /second equation set
//	double*		pTDSCQDM_ValveOFF_ThroughNormFactor3;			// pointer to SCQDM ValveOFF model through variable normalization factors in /the /third equation set
//	double*		pTDSCQDM_ValveOFF_ControlNormFactor;			// pointer to SCQDM ValveOFF model control normalization factors
//
//	int			nTDSCQDM_ValveOFF_Equ1;							// number of SCQDM ValveOFF model linear through equations
//	int			nTDSCQDM_ValveOFF_Equ2;							// number of SCQDM ValveOFF model linear internal equations
//	int			nTDSCQDM_ValveOFF_Equ3;							// number of SCQDM ValveOFF model nonlinear internal equations
//	int			nTDSCQDM_ValveOFF_State;						// number of SCQDM ValveOFF model states in time domain quadratized model
//	int			nTDSCQDM_ValveOFF_Control;						// number of SCQDM ValveOFF model control variables in time domain quadratized //model
//	int			nTDSCQDM_ValveOFF_Feqxx;						// number of SCQDM ValveOFF model quadratic terms for states in the third set /of /equations
//	int			nTDSCQDM_ValveOFF_Fequu;						// number of SCQDM ValveOFF model quadratic terms for controls in the third set //of equations
//	int			nTDSCQDM_ValveOFF_Fequx;						// number of SCQDM ValveOFF model quadratic terms for the product of state and //control variable in the third set of equations
//
//	// ---- SCQDM capacitor model
//
//	void		Init_SCQDM_CAP();								// SCQDM Capacitor model Initialization
//	void		Clear_SCQDM_CAP();								// SCQDM Capacitor model clear
//
//	double**	pTDSCQDM_CAP_Yeqx1;								// pointer to SCQDM CAP model linear state matrix in linear through equations
//	double**	pTDSCQDM_CAP_Yequ1;								// pointer to SCQDM CAP model control state matrix in linear through equations
//	double**	pTDSCQDM_CAP_Deqxd1;							// pointer to SCQDM CAP model defferential state matrix in linear through //equations
//	double*		pTDSCQDM_CAP_Ceqc1;								// pointer to SCQDM CAP model constant in linear through equations
//
//	double**	pTDSCQDM_CAP_Yeqx2;								// pointer to SCQDM CAP model linear state matrix in linear internal equations
//	double**	pTDSCQDM_CAP_Yequ2;								// pointer to SCQDM CAP model control state matrix in linear internal equations
//	double**	pTDSCQDM_CAP_Deqxd2;							// pointer to SCQDM CAP model defferential state matrix in linear internal //equations
//	double*		pTDSCQDM_CAP_Ceqc2;								// pointer to SCQDM CAP model constant in linear internal equations
//
//	double**	pTDSCQDM_CAP_Yeqx3;								// pointer to SCQDM CAP model linear state matrix in nonlinear equations
//	double**	pTDSCQDM_CAP_Yequ3;								// pointer to SCQDM CAP model control state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_CAP_Feqxx3;							// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_CAP_Fequu3;							// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
//	SP_CUBIX*	pTDSCQDM_CAP_Fequx3;							// pointer to SCQDM CAP model quadratic state matrix in nonlinear equations
//	double*		pTDSCQDM_CAP_Ceqc3;								// pointer to SCQDM CAP model constant in nonlinear equations
//
//	CString*	pTDSCQDM_CAP_NodeName;							// pointer to SCQDM CAP model connectivity vectors
//	double*		pTDSCQDM_CAP_StateNormFactor;					// pointer to SCQDM CAP model state normalization factors
//	double*		pTDSCQDM_CAP_ThroughNormFactor1;				// pointer to SCQDM CAP model through variable normalization factors in the /first/ equation set
//	double*		pTDSCQDM_CAP_ThroughNormFactor2;				// pointer to SCQDM CAP model through variable normalization factors in the //second equation set
//	double*		pTDSCQDM_CAP_ThroughNormFactor3;				// pointer to SCQDM CAP model through variable normalization factors in the /third/ equation set
//	double*		pTDSCQDM_CAP_ControlNormFactor;					// pointer to SCQDM CAP model control normalization factors
//
//	int			nTDSCQDM_CAP_Equ1;								// number of SCQDM CAP model linear through equations
//	int			nTDSCQDM_CAP_Equ2;								// number of SCQDM CAP model linear internal equations
//	int			nTDSCQDM_CAP_Equ3;								// number of SCQDM CAP model nonlinear internal equations
//	int			nTDSCQDM_CAP_State;								// number of SCQDM CAP model states in time domain quadratized model
//	int			nTDSCQDM_CAP_Control;							// number of SCQDM CAP model control variables in time domain quadratized model
//	int			nTDSCQDM_CAP_Feqxx;								// number of SCQDM CAP model quadratic terms for states in the third set of //equations
//	int			nTDSCQDM_CAP_Fequu;								// number of SCQDM CAP model quadratic terms for controls in the third set of //equations
//	int			nTDSCQDM_CAP_Fequx;								// number of SCQDM CAP model quadratic terms for the product of state and /control/ variable in the third set of equations
//
//	// ---- SCAQCF single valve ON and OFF model
//
//	void		Init_SCAQCF_ValveON();							// SCAQCF ValveON model Initialization
//	void		Init_SCAQCF_ValveOFF();							// SCAQCF ValveOFF model Initialization
//	void		Clear_SCAQCF_ValveON();							// SCAQCF ValveON model clear
//	void		Clear_SCAQCF_ValveOFF();						// SCAQCF ValveOFF model clear
//
//	double**	pTDSCAQCFModel_ValveON_Yeqx;					// pointer to SCAQCF ValveON model linear state matrix
//	double**	pTDSCAQCFModel_ValveON_Yequ;					// pointer to SCAQCF ValveON model linear control matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveON_Feqxx;					// pointer to SCAQCF ValveON model quadratic state matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveON_Fequu;					// pointer to SCAQCF ValveON model quadratic control matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveON_Fequx;					// pointer to SCAQCF ValveON model quadratic product of state and control matrix 
//	double**	pTDSCAQCFModel_ValveON_Neqx;					// pointer to SCAQCF ValveON model linear past history state matrix 
//	double**	pTDSCAQCFModel_ValveON_Nequ;					// pointer to SCAQCF ValveON model linear past history control matrix 
//	double**	pTDSCAQCFModel_ValveON_Meq;						// pointer to SCAQCF ValveON model linear past history through viariable matrix
//	double*		pTDSCAQCFModel_ValveON_Keq;						// pointer to SCAQCF ValveON model constant part
//	CString*	pTDSCAQCFModel_ValveON_NodeName;				// pointer to SCAQCF ValveON model connectivity vectors
//	double*		pTDSCAQCFModel_ValveON_StateNormFactor;			// pointer to SCAQCF ValveON model state normalization factors
//	double*		pTDSCAQCFModel_ValveON_ThroughNormFactor;		// pointer to SCAQCF ValveON model through variable normalization factors
//	double*		pTDSCAQCFModel_ValveON_ControlNormFactor;		// pointer to SCAQCF ValveON model control normalization factors
//
//	int			nTDSCAQCFModel_ValveON_Equ;						// number of SCAQCF ValveON model equations (time t and tm)
//	int			nTDSCAQCFModel_ValveON_EquOver2;				// number of SCAQCF ValveON model equations (time t only)
//	int			nTDSCAQCFModel_ValveON_State;					// number of SCAQCF ValveON model states (time t and tm)
//	int			nTDSCAQCFModel_ValveON_Control;					// number of SCAQCF ValveON model control variables (time t and tm)
//	int			nTDSCAQCFModel_ValveON_Feqxx;					// number of SCAQCF ValveON model quadratic terms for states (time t and tm)
//	int			nTDSCAQCFModel_ValveON_Fequu;					// number of SCAQCF ValveON model quadratic terms for controls (time t and tm)
//	int			nTDSCAQCFModel_ValveON_Fequx;					// number of SCAQCF ValveON model quadratic terms for the product of state and //control variable (time t and tm)
//
//	double**	pTDSCAQCFModel_ValveOFF_Yeqx;					// pointer to SCAQCF ValveOFF model linear state matrix
//	double**	pTDSCAQCFModel_ValveOFF_Yequ;					// pointer to SCAQCF ValveOFF model linear control matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveOFF_Feqxx;					// pointer to SCAQCF ValveOFF model quadratic state matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveOFF_Fequu;					// pointer to SCAQCF ValveOFF model quadratic control matrix
//	SP_CUBIX*	pTDSCAQCFModel_ValveOFF_Fequx;					// pointer to SCAQCF ValveOFF model quadratic product of state and control /matrix/ 
//	double**	pTDSCAQCFModel_ValveOFF_Neqx;					// pointer to SCAQCF ValveOFF model linear past history state matrix 
//	double**	pTDSCAQCFModel_ValveOFF_Nequ;					// pointer to SCAQCF ValveOFF model linear past history control matrix 
//	double**	pTDSCAQCFModel_ValveOFF_Meq;					// pointer to SCAQCF ValveOFF model linear past history through viariable matrix
//	double*		pTDSCAQCFModel_ValveOFF_Keq;					// pointer to SCAQCF ValveOFF model constant part
//	CString*	pTDSCAQCFModel_ValveOFF_NodeName;				// pointer to SCAQCF ValveOFF model connectivity vectors
//	double*		pTDSCAQCFModel_ValveOFF_StateNormFactor;		// pointer to SCAQCF ValveOFF model state normalization factors
//	double*		pTDSCAQCFModel_ValveOFF_ThroughNormFactor;		// pointer to SCAQCF ValveOFF model through variable normalization factors
//	double*		pTDSCAQCFModel_ValveOFF_ControlNormFactor;		// pointer to SCAQCF ValveOFF model control normalization factors
//
//	int			nTDSCAQCFModel_ValveOFF_Equ;					// number of SCAQCF ValveOFF model equations (time t and tm)
//	int			nTDSCAQCFModel_ValveOFF_EquOver2;				// number of SCAQCF ValveOFF model equations (time t only)
//	int			nTDSCAQCFModel_ValveOFF_State;					// number of SCAQCF ValveOFF model states (time t and tm)
//	int			nTDSCAQCFModel_ValveOFF_Control;				// number of SCAQCF ValveOFF model control variables (time t and tm)
//	int			nTDSCAQCFModel_ValveOFF_Feqxx;					// number of SCAQCF ValveOFF model quadratic terms for states (time t and tm)
//	int			nTDSCAQCFModel_ValveOFF_Fequu;					// number of SCAQCF ValveOFF model quadratic terms for controls (time t and tm)
//	int			nTDSCAQCFModel_ValveOFF_Fequx;					// number of SCAQCF ValveOFF model quadratic terms for the product of state and //control variable (time t and tm)
//
//	// ---- SCAQCF capacitor model
//
//	void		Init_SCAQCF_CAP();								// SCAQCF Capacitor model Initialization
//	void		Clear_SCAQCF_CAP();								// SCAQCF Capacitor model clear
//
//	double**	pTDSCAQCFModel_CAP_Yeqx;						// pointer to SCAQCF CAP model linear state matrix
//	double**	pTDSCAQCFModel_CAP_Yequ;						// pointer to SCAQCF CAP model linear control matrix
//	SP_CUBIX*	pTDSCAQCFModel_CAP_Feqxx;						// pointer to SCAQCF CAP model quadratic state matrix
//	SP_CUBIX*	pTDSCAQCFModel_CAP_Fequu;						// pointer to SCAQCF CAP model quadratic control matrix
//	SP_CUBIX*	pTDSCAQCFModel_CAP_Fequx;						// pointer to SCAQCF CAP model quadratic product of state and control matrix 
//	double**	pTDSCAQCFModel_CAP_Neqx;						// pointer to SCAQCF CAP model linear past history state matrix 
//	double**	pTDSCAQCFModel_CAP_Nequ;						// pointer to SCAQCF CAP model linear past history control matrix 
//	double**	pTDSCAQCFModel_CAP_Meq;							// pointer to SCAQCF CAP model linear past history through viariable matrix
//	double*		pTDSCAQCFModel_CAP_Keq;							// pointer to SCAQCF CAP model constant part
//	CString*	pTDSCAQCFModel_CAP_NodeName;					// pointer to SCAQCF CAP model connectivity vectors
//	double*		pTDSCAQCFModel_CAP_StateNormFactor;				// pointer to SCAQCF CAP model state normalization factors
//	double*		pTDSCAQCFModel_CAP_ThroughNormFactor;			// pointer to SCAQCF CAP model through variable normalization factors
//	double*		pTDSCAQCFModel_CAP_ControlNormFactor;			// pointer to SCAQCF CAP model control normalization factors
//
//	int			nTDSCAQCFModel_CAP_Equ;							// number of SCAQCF CAP model equations (time t and tm)
//	int			nTDSCAQCFModel_CAP_EquOver2;					// number of SCAQCF CAP model equations (time t only)
//	int			nTDSCAQCFModel_CAP_State;						// number of SCAQCF CAP model states (time t and tm)
//	int			nTDSCAQCFModel_CAP_Control;						// number of SCAQCF CAP model control variables (time t and tm)
//	int			nTDSCAQCFModel_CAP_Feqxx;						// number of SCAQCF CAP model quadratic terms for states (time t and tm)
//	int			nTDSCAQCFModel_CAP_Fequu;						// number of SCAQCF CAP model quadratic terms for controls (time t and tm)
//	int			nTDSCAQCFModel_CAP_Fequx;						// number of SCAQCF CAP model quadratic terms for the product of state and control variable (time t and tm)

	// ---- Converter Status

	int**		iValveStatusPerMode;							// valve ON or OFF status for each mode, 
																// iValvePresentTimeStatus[i][j]=k; i: ith valve; j: jth mode; k: ON (1) or OFF (0)

	double		dPreviousValveONTime;							// time when the last valve was turned on
	bool		bCommutation;									// wheter the converter is in the commutation period

	//----------------------------------------------------------
	// ---- Stoage Scheme for Converter AQCF models - 0-12 + 1 modes
	//----------------------------------------------------------

	vector<CPowerDevice_M801*>		pTDSCAQCFModel_M801;					// pointers to AQCF converter models
	CPowerDeviceT*					pTDSCAQCFModel_Cap;						// pointers to AQCF single cap model
	CPowerDeviceT*					pTDSCAQCFModel_ValveON;					// pointers to AQCF single valve ON model
	CPowerDeviceT*					pTDSCAQCFModel_ValveOFF;				// pointers to AQCF single valve OFF model


	int			nConverterEqu;
	int			nConverterState;


	//------member procedures of Trapezoidal Integration ------------

	BOOL	ComputeModel801Pointers();
	BOOL	ComputeValveAndCap_Y_PandZ_801();
	BOOL	ComputeConverter_Y_PandZ_801();
	BOOL	ComputeConverterSubStepACF_801();
	BOOL	DetermineValveStateAtSubStep_M801(int iSubStep);
	BOOL	ComputeConverterTwoStepACF_801(double dStep1,double dStep2);
	BOOL	ComputeConverterFirstStepACF_801();

	BOOL	UpdateConverterCurrents_M801();

	//------member procedures of Quadratic Integration ------------

	BOOL	TQN_UpdateConverterRunTimeRef();
	BOOL	TQN_ComputeModelPointers();
	BOOL	TQN_ComputeValveAndCap_Y_PandZ_801();
	BOOL	TQN_ComputeValveSwitchingTimes();
	BOOL	TQN_ComputeInverterMode();
	BOOL	TQN_ComputeConverter_Y_PandZ_801();
	BOOL	TQN_ComputeConverterFirstStepACF_801();

	//------member of Controller action

	BOOL    TQN_TimeUpdateValveStatus_801();
	BOOL	TQN_CalculateFiringAngle_801();
	BOOL	TQN_ComputeValveScheduleTime();
	BOOL	TQN_UpdateValveScheduleTime();

	BOOL	TQN_CalculateFiringAngleINV();
	BOOL	TQN_ComputeValveSwitchingTimesINV_801();

	//------Debug report of Trapezoidal integration------------------

	BOOL	DebugReportModel801();
	BOOL	DebugPartialReportModel801();

	//------Debug report of Quadratic integration--------------------

	BOOL	TQN_DebugReportModel801();
	BOOL	TQN_DebugMatrixReportModel801();

	//------Allocation/Deallocation arrays of Trapezoidal integration

	BOOL	NullDataModel_801();
	BOOL	AllocateDataModel_801();
	BOOL	DeleteDataModel_801();

	//----member variables for Quadratic and Trapezoidal integration-

	double		dBetaM801;				// stabilizer value

	int			pUserDevices801;		//selected DSP
	int			MeasureDeviceIDM801;
	int			nValves;				// total number of valves;
	int			nValTerm;
	int			nCapTerm;
	int			nValTerm_tq;
	int			nCapTerm_tq;

	int			iPfControl;				// a code: 1: firing angle, 2: power, 3: DC Voltage

	int			iCControlM801;			// a code: 1:equidistant, 2:real power, 3:voltage

	int			nSubSteps;				// number of substeps for variable time step scheme
	double		dHdtsecsSubM801;		// Half SubTimeStep - working
	double		ddtsecsM801;			// TimeStep
	int			nBlockedCycles;			// number of cycles to be blocked for converter at starting
	double		dBlockedTime;			// blocked time of the converter operation
	int			iAnySwitching;			// a code:	0 if no switching any valve in (t-h,t)
										//			1 if at least one valve switching in (t-h,t)
	double		dValveOffConductance;	// Valve conductance, OFF state
	double		dValveONConductance;	// Valve conductance, ON state
	double		dValveOffConductanceL;	// Valve conductance, OFF state times inductance
	double		dValveONConductanceL;	// Valve conductance, ON state times inductance
	double		dCLInductance;			// Current limiting inductor incuctance (Henries)
	double		dCLConductance;			// Conductance of current limiting conductance in parallel with inductor
	double		dParasiticCapacitance;	// Thyristor Parasitic Capacitance (Farads)
	double		dSnubberConductance;	// Snubber resistor conductance (mhos)
	double		dSnubberCapacitance;	// Snubber Capacitance (Farads)
	double		dGvcConductance;		// Conductance of valve capacitor stabilizer 
	double		dGvcConductanceL;		// Conductance of valve capacitor stabilizer times inductance

	//------Single Valve matrices of Trapezoidal------------------

	int**		iValvePointerM801;		// pointer of one valve's index to state index
										// index in xa_real = iValvePointerM801[index in valve][valve#]
	double**	dSValveCurrentM801;		// single valve currents
	double**	dSValvePHCurrentM801;	// single valve paqst history currents

	double		dCpYeqReal;
	double		dCpPeqReal;
	double*		dCpCurrent;
	double*		dCpPHCurrent;
	double*		dSnubberCurrent;
	double*		dValveCurrentM801;
	double*		dValveCurrentPastM801;

	//------Single Valve matrices of Quadratic------------------

	int**		iValvePointer;		

	double**	dYmatrixSvalve;			// single valve Y matrix of a Compact form of ACF
	double**	dYmatrixSvalveON;		// single valve Y matrix of a Compact form of ACF at Switch ON
	double**	dYmatrixSvalveOFF;		// single valve Y matrix of a Compact form of ACF at Switch OFF
	double**	dPmatrixSvalve;			// single valve P matrix of a Compact form of ACF
	double**	dPmatrixSvalveON;		// single valve P matrix of a Compact form of ACF at Switch ON
	double**	dPmatrixSvalveOFF;		// single valve P matrix of a Compact form of ACF at Switch OFF
	double**	dZmatrixSvalve;			// single valve Z matrix of a Compact form of ACF

	//------Capacitor matrices of Trapezoidal------------------------

	double		dSmoothingCapacitance;	// capacitance(Farads)
	int*		iCapPointerM801;		// pointer of one capacitor's index to state index
										// index in xa_real = iCapPointerM801[index in cap]

	//------Capacitor matrices of Quadratic------------------------

	double**	dYmatrixCap;			// smoothing capacitor Y matrix of ACF
	double**	dPmatrixCap;			// smoothing capacitor P matrix of ACF
	double**	dZmatrixCap;			// smoothing capacitor Z matrix of ACF
	int*		iCapPointerM801_tq;		// pointer of one capacitor's index to state index

	//------Converter matrices of Trapezoidal------------------------

	double**	dMmatrixConverter;		// converter M matrix: Mc
	double**	dNmatrixConverter;		// converter N matrix: Nc
	double**	dYeqSubStepM801;		// defined as (Mc+2Nc/hs), hs is substep time step
	double**	dPeqSubStepM801;		// defined as (Mc-2Nc/hs), hs is substep time step

	double**	dY22M801;				// Xi- submatrix of yeq_real
	double**	dInvY22M801;			// Xi- inverse of submatrix of yeq_real
	double**	dY21M801;				// Xi- submatrix of yeq_real
	double**	dDM801;					// Xi- D matrix
	double*		b_vector;				// Xi-working vector
	double*		x_vector;				// Xi-working vector
	double*		y_vector;				// Xi-working vector

	//------Converter matrices of Quadratic--------------------------

	double**	dYmatrixConverter;		// converter Y matrix of ACF
	double**	dPmatrixConverter;		// converter P matrix of ACF
	double**	dZmatrixConverter;		// converter Z matrix of ACF

	//------control references---------------------------------------

	double		dVacZeroTime;			// zero crossing time of voltage phase A-phase C 
										//		normalized: i.e. nearest zero crossing
										//		(from neg to pos) to the time
										//		dRunTimeNormRef
										//		(always negative)
	double		dPosSeqVMagn;			// Positive Sequence voltage magnitude
	double		dRealPowerAC;			// Net real power flow
	double		dReactivePowerAC;		// Net reactive power flow
	double		dNoLoadDirectVoltage;   // Ideal no-load direct Votage
	double		dAvgDirectVoltage;		// average direct Votage
	double		dDirectCurrent;			// DC current for control action
	double		dDirectVoltage;			// DC Voltage for control action
	double		dFrequency;				// Frequency from DSP	

	//------update firing time signals-------------------------------

	int			iOperating_mode;			// Operating mode 1=rectifier, 2=inverter
	int			iInverterSStatusModel;		// index for inverter switching status model
	int			iLastTurnedONValve;			// the last valve that was turned on
//	bool		bInverterModeChange;		// True: mode change; False: no mode change
	int			iSwitchValveON;				// the index of the valve that will be turned ON
	int			iSwitchValveOFF;			// the index of the valve that will be turned OFF
	double		dCommutationCount;			// To count the time of commutation 
	
	double		dRealPowerDC;				// Rated Power of DC side
	double		dFiringAngle;				// firing delay for Equidistant control 
	double		dPre_FiringAngle;			// previous firing angle
	double		dExtinctionAngle;			// Extinction angle for the inverter mode 
	double		dTimeDelay;					// converter time delay for valve 1.
	double		dTimePeriod;				// fundamental period
	double		dConverterRelativeTime;		// Converter run relative time
	double		dConverterTimeRef;			// Relative run time reference -  
											// equals integer number of periods.
											// it is introduced to be used as a reference of 
											// one cycle operation, so that the variable
											//		dRunTimeNorm
											// is always less than one cycle
	double		dValve1TurnOnTime;			// valve 1 next turn ON time

	double*		dPastState;					// converter previous time step state
	int*		iValveStatusM801;			// valve status: 0: off, 1: ON
	int*		iValvePresentTimeStatus;	// valve status: 0: off, 1: ON

	double*		dValveConductanceM801;		// valve conductance matrix at each substep
	double*		dValveNextTurnOn;			// valve turn-ON time - next
											//	normalized time with respect to time
											//	dRunTimeNormRef
											//	it is defined for one period starting
											//	at time dRunTimeNormRef


	double*		dValveTurnONTimesOld;		//  valve turn-ON time from last time step
	double*		dValveTurnONTimesNew;		//  valve turn-ON time in the present time step
	int*		nValveDelayM801;			//  valve delay logic upon turn-on
	double*		dValveSwitchTime;			// valve switching time (seconds from t-h)




	double		dConverterStartTime;		// converter start time after simulation starts (seconds in actual time)
	bool		bConverterON;				// a code:	true if the converter is operational
											//			false if the converter is not operational

	double		dLogicStartTime;			// logic start time after simulation starts (seconds in actual time)
	bool		bLogicStart;				// true if logic starts
											// false if logic has not started

	bool		bCommutationON;				// a code:	true if converter in commutation
											//			false if iconverter not commutation does not start
	double		dCommutationStartTime;		// The start time of last commutation (second in actual-time)
	double		dCommutationTime;			// The time from firing angle to extinction angle 
	int			iNextCommutationValve;		// the index of the valve for next commutation
	double		dCurrentNextCommuationValve;// Current through the commutation valve


	double*		dValveTurnONTimes;			//  valve turn-ON time
											//	relative time with respect to time reference: dConverterTimeRef
											//	it is defined for one period starting at time dConverterTimeRef
	int			iNextTurnONValve;			// the next pulse that will be turned on

	double		dLastDebugTime;

};								
#endif							
