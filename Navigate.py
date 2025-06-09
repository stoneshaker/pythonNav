void Navigate(void)
{

	#----------------------------------------------------------
	# The following define determines how frequently filter
	# prediction is carried out - NEF runs at 600Hz, so setting
	# this threshold value to 60 means that filter prediction
	# will run once for every 60 NEF cycles i.e. 10Hz.
	#----------------------------------------------------------
#define NEF_CYCLE_COUNT_THRESHOLD    60

	uint32 GPS_Measurement_Status;
	uint32 Time_When_Filter_Started;
	uint32 NEF_Cycles_Since_Prediction;


	if (SiNAV_Cycle_Count < SiNAV_Cycle_Count_At_Predict_Time)
	{
		# Handle wrap around of SiNAV_Cycle_Count
		NEF_Cycles_Since_Prediction = SiNAV_Cycle_Count +
			(MAX_SiNAV_CYCLE_COUNT - SiNAV_Cycle_Count_At_Predict_Time);
	}
	else
	{
		NEF_Cycles_Since_Prediction = 
			SiNAV_Cycle_Count - SiNAV_Cycle_Count_At_Predict_Time;
	}

	if (NEF_Cycles_Since_Prediction >= NEF_CYCLE_COUNT_THRESHOLD)
	{

		# filter predict/measurement update cycle required.
		SiNAV_Cycle_Count_At_Predict_Time = SiNAV_Cycle_Count;

		# do filter predict.
		Time_When_Filter_Started = Read_Nav_Time(SYSTEM_TIME);

		/*
		if (NEF_Status == NEF_NAVIGATING)
		{
		Start_Event_Time(1); # ???? Debug
		}
		*/

		LCNApplication_FilterPredict(
			(short_float)(Time_When_Filter_Started * SINAV_TIME_TO_REAL_TIME));

		/*
		# ???? Start Debug
		if (NEF_Status == NEF_NAVIGATING)
		{
		if (Event_Index < MAX_EVENT_INDEX)
		{
		Event_Time = Request_Event_Time(1);
		Event_Time_Array[Event_Index] = Event_Time;
		}
		else
		{
		Event_Time = 0;
		}
		Event_Index++;
		}
		# ???? End Debug
		*/

		if (Covariance_Data_Logging_Enabled == TRUE)
		{
			# Only create Covariance data if logging is enabled.
			Create_Covariance_Data_Log_Message();
		}

		#      GPS_Measurement_Status = Read_Integer_Pseudo_Range_Measurement_Status();
		GPS_Measurement_Status = Read_Integer_Modified_Line_Of_Sight_Measurement_Status();

		# prediction over with - see if we need to do measurement updates.
		if (GPS_Measurement_Status > 0)
		{

			# do measurement updates.

			Measurement_Update(GPS_Measurement_Status);

			#----------------------------------------------------------------
			# disable the IMU (NEF) interrupt while critical data is updated.
			#----------------------------------------------------------------
			#         sysreg_bit_clr(sysreg_IMASK,DAIHI); 

			# Apply NEF corrections and store position in nav
			# buffer to correct up to.
			LCNApplication_StopMeasurements();

			#-----------------------------------
			# re-enable the IMU (NEF) interrupt.
			#-----------------------------------
			#         sysreg_bit_set(sysreg_IMASK,DAIHI); 

			# Correct the nav buffer with the latest corrections.
			LCNApplication_CorrectNavBuffer();

		}

	}
}
