void Initialize_NEF(void)
{
	# This function initializes the Navigation Filter

	struct sNEFInitializationData Init_Data;
	struct s3dVector AHRS = AHRS_GetAttitude();
	struct s3dVector GPSAntennaLeverArm;

	long_float Pos_X, Pos_Y, Pos_Z;
	long_float Lat,   Long,  Height;
	long_float Heading;


	Init_Data.TimeOfValidity =
		((long_float)Read_Nav_Time(SYSTEM_TIME) * SINAV_TIME_TO_REAL_TIME);

	if (Input_Init_Message.Validity.Position_Valid)
	{
		# User Input Initialization

		Init_Data.Position.Element[GEODETIC_POSITION_LATITUDE] =
			Input_Init_Message.Latitude;
		Init_Data.Position.Element[GEODETIC_POSITION_LONGITUDE] =
			Input_Init_Message.Longitude;
		Init_Data.Position.Element[GEODETIC_POSITION_HEIGHT] =
			Input_Init_Message.Altitude;
	}
	else
	{
		# GPS Position Initialization

		# Convert ECEF Position to Geodetic Position
		Pos_X = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
			GPS_Nav_Solution.Position_ECEF_X);
		Pos_Y = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
			GPS_Nav_Solution.Position_ECEF_Y);
		Pos_Z = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
			GPS_Nav_Solution.Position_ECEF_Z);
		ECEF_To_Geodetic (
			Pos_X, Pos_Y, Pos_Z,
			&Lat,  &Long, &Height);

		Init_Data.Position.Element[GEODETIC_POSITION_LATITUDE] = Lat;
		Init_Data.Position.Element[GEODETIC_POSITION_LONGITUDE] = Long;
		Init_Data.Position.Element[GEODETIC_POSITION_HEIGHT] = Height;
	}

	if (Input_Init_Message.Validity.Velocity_Valid)
	{
		# User Input Initialization

		Init_Data.Velocity.Element[NAV_FRAME_NORTH] =
			Input_Init_Message.North_Velocity;
		Init_Data.Velocity.Element[NAV_FRAME_EAST] =
			Input_Init_Message.East_Velocity;
		Init_Data.Velocity.Element[NAV_FRAME_DOWN] =
			Input_Init_Message.Down_Velocity;
	}
	else
	{
		# GPS Velocity Initialization

		Init_Data.Velocity.Element[NAV_FRAME_NORTH] = GPS_Vel_North;
		Init_Data.Velocity.Element[NAV_FRAME_EAST] = GPS_Vel_East;
		Init_Data.Velocity.Element[NAV_FRAME_DOWN] = GPS_Vel_Down;

	}

	if (Input_Init_Message.Validity.Attitude_Valid)
	{
		# User Input Initialization

		Init_Data.Attitude.Element[NAV_FRAME_EULER_ROLL] =   #1.570796326794897;
			Input_Init_Message.Roll;
		Init_Data.Attitude.Element[NAV_FRAME_EULER_PITCH] =  #1.570796326794897;
			Input_Init_Message.Pitch;
	}
	else
	{

		# AHRS Attitude Initialization

		Init_Data.Attitude.Element[NAV_FRAME_EULER_ROLL] =
			#AHRS.Element[NAV_FRAME_EULER_ROLL];
			AHRS_PreviousEulerAttitude[4].Element[NAV_FRAME_EULER_ROLL];
		Init_Data.Attitude.Element[NAV_FRAME_EULER_PITCH] =
			#AHRS.Element[NAV_FRAME_EULER_PITCH];
			AHRS_PreviousEulerAttitude[4].Element[NAV_FRAME_EULER_PITCH];
	}

	if (Input_Init_Message.Validity.Heading_Valid)
	{
		# User Input Initialization (Only valid state for lab mode)

		Heading = Input_Init_Message.Heading;
		#Heading =  1.570796326794897; # pi/2, or straight east
	}
	else
	{
		# Heading initialized from GPS Velocities.

		Heading = lf_atan2(GPS_Vel_East, GPS_Vel_North);

	}

	Init_Data.Attitude.Element[NAV_FRAME_EULER_HEADING] = Heading;

	LCNApplication_SetParameters();

	# Setting the default parameters wipes the GPS antenna lever arm
	# data so we have to re-affirm it.
	GPSAntennaLeverArm.Element[0] = Input_Control_Message.Lever_Arm_X;
	GPSAntennaLeverArm.Element[1] = Input_Control_Message.Lever_Arm_Y;
	GPSAntennaLeverArm.Element[2] = Input_Control_Message.Lever_Arm_Z;

	LCNApplication_SetGPSAntennaLeverArm(GPSAntennaLeverArm);

	LCNApplication_Initialize(Init_Data);

}