//---------------------------------------------------------------------------------------------

void Update_NEF_And_GPS_State(void)
{
#define AHRS_SETTLING_TIME 10.0  // seconds

#define GPS_NAV_SOLUTION_QUALITY_THREHOLD 50.0  // metres

	// FEET TO METERS
#define FEET_TO_METERS 0.3048

	boolean New_GPS_Nav_Solution;
	boolean New_GPS_Time_Mark_Pulse_2;

	short_float GPS_Position_Quality = 0.0;

	// GPS Velocity definitions and variables
#define GPS_VELOCITY_THRESHOLD 3.0 // m/s

	long_float GPS_Horizontal_Velocity, GPS_Velocity;
	long_float GPS_X_Vel, GPS_Y_Vel, GPS_Z_Vel;
	long_float Pos_X, Pos_Y, Pos_Z;
	long_float Lat,   Long,  Height;
	uint16     Vertical_Error   = 999;
	uint16     Horizontal_Error = 999;
	float dum_float;


	if ((New_Init_Message == TRUE) &&
		(Input_Init_Message.Validity.Heading_Valid == TRUE) &&
		(Movement_Detector_Enabled == FALSE))
	{
		// Lab Mode

		// A heading has been received from the User. This could be a new heading.

		NEF_Status = NEF_WAITING_FOR_VALID_INIT_DATA;

		Heading_Init_Valid    = TRUE;
		Roll_Pitch_Init_Valid = FALSE;
		Velocity_Init_Valid   = FALSE;
		Position_Init_Valid   = FALSE;

	}

	// NEF state transitions and actions

	switch (NEF_Status)
	{
	case NEF_NOT_INItializED:

		Heading_Init_Valid    = FALSE;
		Roll_Pitch_Init_Valid = FALSE;
		Velocity_Init_Valid   = FALSE;
		Position_Init_Valid   = FALSE;
		//Movement_Detector_Enabled = TRUE;  //  Lab Use only with no SiIMU02

		if (Movement_Detector_Enabled == TRUE)
		{

			// Movement detection initialization is expected.
			NEF_Status = NEF_WAITING_FOR_VALID_INIT_DATA;

		}
		else
		{
			// Stay in state
		}

		break;

	case NEF_WAITING_FOR_VALID_INIT_DATA:

		// NEF_WAITING_FOR_VALID_INIT_DATA state can only be entered if
		// Movement_Detector_Enabled = TRUE or in Lab Mode.

		// Get latest GPS Nav Solution message from the GPS card.
		New_GPS_Nav_Solution =
			New_GPS_Time_Mark_Pulse_Message();
		New_GPS_Time_Mark_Pulse_2 = 
			New_GPS_Time_Mark_Pulse_2_Message();

		dum_float = (double)System_Time/15625.0;

         if (New_GPS_Nav_Solution)
		 {
		 	GPS_Nav_Solution = Get_GPS_Time_Mark_Pulse_Message();
                          //Get_GPS_Navigation_Solution_Message();
                          
            if (New_GPS_Time_Mark_Pulse_2)
            {                          
         	   Horizontal_Error = One_16_Bit_Complement_Word_To_Integer(
         	                       GPS_Nav_Solution.Est_Horizontal_Error);
          	   Vertical_Error   = One_16_Bit_Complement_Word_To_Integer(
           	                       GPS_Nav_Solution.Est_Vertical_Error);
            }
		 }

		GPS_Position_Quality = sqrt(Horizontal_Error * 
			Horizontal_Error + Vertical_Error * Vertical_Error) * 
			FEET_TO_METERS;

		GPS_Horizontal_Velocity = 0.0;
		GPS_Velocity = 0.0;

		if ((New_GPS_Nav_Solution == TRUE) &&
			(GPS_Nav_Solution.Data_Block_Valid == TRUE))// && 
			//(GPS_Nav_Solution.Nav_Data_Validity == TRUE) &&
			//(GPS_Position_Quality < GPS_NAV_SOLUTION_QUALITY_THREHOLD))
		{
            if ((GPS_Position_Quality < GPS_NAV_SOLUTION_QUALITY_THREHOLD) &&
                (GPS_Position_Quality > 0))
            {
				// GPS Horizontal Velocity always needs to be calculated if there
				// is a good GPS position solution, as it can be used to determine
				// the validity of the heading.

				// Calculate GPS Horizontal Velocity
				GPS_Vel_North =  One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Velocity_North);
				GPS_Vel_East  =  One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Velocity_East);
				GPS_Vel_Down  = -One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Velocity_Up);
				// Convert ECEF Position to Geodetic Position
				Pos_X = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Position_ECEF_X);
				Pos_Y = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Position_ECEF_Y);
				Pos_Z = One_32_Bit_DECSPFP_Word_To_IEEE_Double(
					GPS_Nav_Solution.Position_ECEF_Z);

				ECEF_To_Geodetic (
					Pos_X, Pos_Y, Pos_Z,
					&Lat,  &Long, &Height);

				NED_To_ECEF(
					Lat,   Long,
					GPS_Vel_North, GPS_Vel_East, GPS_Vel_Down,
					&GPS_X_Vel, &GPS_Y_Vel, &GPS_Z_Vel);

				GPS_Horizontal_Velocity =
					sqrt((GPS_Vel_North * GPS_Vel_North) +
					(GPS_Vel_East * GPS_Vel_East));

				if (GPS_Quality_Check > 3)
				{
					Velocity_Init_Valid = TRUE;
					Position_Init_Valid = TRUE;
				}
				else GPS_Quality_Check++;
			}
			else GPS_Quality_Check = 0;
		}
		else
		{
			// Stay in state unless the User has set Velocity and Position valid.
			Velocity_Init_Valid = FALSE;
			Position_Init_Valid = FALSE;
		}

		if ((Input_Init_Message.Validity.Position_Valid) &&
			(Input_Init_Message.Validity.Velocity_Valid))
		{
			// Overwrite the validities if requested by User.
			Velocity_Init_Valid = TRUE;
			Position_Init_Valid = TRUE;
		}

		if ((Input_Init_Message.Validity.Attitude_Valid == TRUE) ||
			(AHRS_GetElapsedTime() > AHRS_SETTLING_TIME))
		{
			Roll_Pitch_Init_Valid = TRUE;
		}
		else
		{
			// Stay in state
			Roll_Pitch_Init_Valid = FALSE;
		}

		if ((Input_Init_Message.Validity.Heading_Valid == TRUE) ||

			((Movement_Detector_Enabled == TRUE) &&
			(GPS_Horizontal_Velocity > GPS_VELOCITY_THRESHOLD) &&
			(GPS_Quality_Check > 3)))
		{
			Heading_Init_Valid = TRUE;
		}
		else
		{
			// Stay in state
			Heading_Init_Valid = FALSE;
		}

		if ((Position_Init_Valid == TRUE) &&
			(Velocity_Init_Valid == TRUE) &&
			(Roll_Pitch_Init_Valid == TRUE) &&
			(Heading_Init_Valid == TRUE) &&
			(Real_IMU_Data_Received == TRUE))
		{
			Initialize_NEF();
			printf("Movement Detector has initialized at time %f\n",dum_float);

			NEF_Status = NEF_NAVIGATING;
		}
//         if ((GPS_Position_Quality < GPS_NAV_SOLUTION_QUALITY_THREHOLD))// &&
//         //    (GPS_Position_Quality > 0))
//         {
//         	if (GPS_Quality_Check < 11)
//               GPS_Quality_Check++;  // check for consecutive good quality signals
//         }
//         else if (GPS_Position_Quality > 0.0) GPS_Quality_Check = 0;
//
//		 }
		break;

	case NEF_NAVIGATING:

		Navigate();
		break;
	}
}