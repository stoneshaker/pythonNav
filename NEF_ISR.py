def NEF_ISR()
   MAX_SiNAV_CYCLE_COUNT=int(64799)
   # Note: The IMU data arrives from the FPGA as 24 32 bit words, each
   #       32 bit word contains 2 IMU message bytes, e.g if the 32 bit
   #       word is 0x0034005A the 0x5A represents byte 0 and 0x34
   #       represents byte 1.

   # Bytes in the IMU02 message as for the AMRAAM Auto Pilot Outputs.

   #      Description     Word
   #      -----------     ----
   #define AXIS_1_ACCEL_0  0 # Header
   #define AXIS_1_ACCEL_1  0 # LSB - Accel bits 16 - 23
   #define AXIS_1_ACCEL_2  1 # MSB - Accel bits  0 -  7
   #define AXIS_1_ACCEL_3  1 # Not Used
   #define AXIS_1_RATE_0   2 # Header - bits 0 - 3; bits 4 - 7 are Rate bits 16 - 19
   #define AXIS_1_RATE_1   2 # LSB - Rate bits 16 - 23
   #define AXIS_1_RATE_2   3 # MSB - Rate bits  0 -  7
   #define AXIS_1_RATE_3   3 # Not Used
   #define AXIS_3_ACCEL_0  4 # Header
   #define AXIS_3_ACCEL_1  4 # LSB - Accel bits 16 - 23
   #define AXIS_3_ACCEL_2  5 # MSB - Accel bits  0 -  7
   #define AXIS_3_ACCEL_3  5 # Not Used
   #define AXIS_2_RATE_0   6 # Header - bits 0 - 3; bits 4 - 8 are Rate bits 16 - 19
   #define AXIS_2_RATE_1   6 # LSB - Rate bits 16 - 23
   #define AXIS_2_RATE_2   7 # MSB - Rate bits  0 -  7
   #define AXIS_2_RATE_3   7 # Not Used
   #define AXIS_2_ACCEL_0  8 # Header
   #define AXIS_2_ACCEL_1  8 # LSB - Accel bits 16 - 23
   #define AXIS_2_ACCEL_2  9 # MSB - Accel bits  0 -  7
   #define AXIS_2_ACCEL_3  9 # Not Used
   #define AXIS_3_RATE_0  10 # Header - bits 0 - 3; bits 4 - 8 are Rate bits 16 - 19
   #define AXIS_3_RATE_1  10 # LSB - Rate bits 16 - 23
   #define AXIS_3_RATE_2  11 # MSB - Rate bits  0 -  7
   #define AXIS_3_RATE_3  11 # Not Used

   #define ACCEL_SCALING  0.019613300000 # m/s/s - based on gravity being 9.80665 in the IMU 
   #define GYRO_SCALING   0.001169998917 # rads/s

   AMRAAM_TIME_DELTA = (1.0/600.0)

#   #define AMRAAM_IMU_MESSAGE_SIZE_BYTES 24
   extern struct sNEFInertialInputData imu_message;
   struct sNEFInertialInputData Current_IMU_Data;
   struct sNEFInertialInputData *pIMU_Data = &imu_message;
   extern struct s3dVector AHRS_PreviousEulerAttitude[5];

   short_float  Current_IMU_Data_TOV;
   int32        Temp;
   uint32 Current_Time;

   uint32       IMU02_Status;
   extern uint32       IMU02_Data_Array[AMRAAM_IMU_MESSAGE_SIZE_BYTES];

   uint32       Interrupt_Register = 1;#*pDAI_IRPTL_H;
   uint16   j;
   #define IMU02_RX_SYNTHETIC_INTERRUPT_IN_USE 0x01
   #define IMU02_RX_BUFFER_FULL                0x80
   
   #-----------------------------------------------------------------
   # Check whether data in IMU Rx buffer is valid.  If it is, read it
   # and populate the 'Current_IMU_Data' data structure.
   #-----------------------------------------------------------------

   # Need to read the DAI interrupt latch register to clear latch.
   if ((Interrupt_Register) != 0)#& SRU_EXTMISCB1_INT) != 0)
   {
      IMU02_Status = 0x80;
      IMU02_Status = IMU02_Status & 0xFF;

      if (((IMU02_Status & IMU02_RX_BUFFER_FULL) == IMU02_RX_BUFFER_FULL) &&
          ((IMU02_Status & IMU02_RX_SYNTHETIC_INTERRUPT_IN_USE) !=
                                       IMU02_RX_SYNTHETIC_INTERRUPT_IN_USE))
      {

         # Note: There is no checksum calculation for the AMRAAM IMU.

         Real_IMU_Data_Received = TRUE;

         #---------------------------------------------------------------
         # TOV required in seconds, so get 32-bit SiNAV System Time value
         # and apply conversion factor N.B. LSB = 64us.
         #---------------------------------------------------------------
         Current_IMU_Data_TOV = ((long_float)Read_Nav_Time(SYSTEM_TIME)) *
                                                    SINAV_TIME_TO_REAL_TIME;
         Current_IMU_Data.TimeOfValidity = Current_IMU_Data_TOV;
         Current_IMU_Data.DataValid = TRUE;
   
         # Time delta is current IMU data TOV minus the previous TOV
         # value ...
         # Current_IMU_Data.TimeDelta = Current_IMU_Data_TOV -
         #                                        Previous_IMU_Data_TOV;
   
         Current_IMU_Data.TimeDelta = AMRAAM_TIME_DELTA;
   
         # update 'old' value ...
         Previous_IMU_Data_TOV = Current_IMU_Data_TOV;
   
         # Read in the IMU data frame
		 Create_IMU_signal(pIMU_Data);
         Temp =
            ((IMU02_Data_Array[AXIS_1_ACCEL_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_1_ACCEL_2] & 0xFF) << 8);
         if ((Temp & 0x8000) == 0x8000)
         {
            # Sign Extend 16 bit value
            Temp = Temp | 0xFFFF0000;
         }
         Current_IMU_Data.BodyAccelerations['BODY_FRAME_X'] =
                                                (float)Temp * ACCEL_SCALING;
   
         Temp =
            ((IMU02_Data_Array[AXIS_2_ACCEL_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_2_ACCEL_2] & 0xFF) << 8);
         if ((Temp & 0x8000) == 0x8000)
         {
            # Sign Extend 16 bit value
            Temp = Temp | 0xFFFF0000;
         }
         Current_IMU_Data.BodyAccelerations['BODY_FRAME_Y'] =
                                                (float)Temp * ACCEL_SCALING;
    
         Temp =
            ((IMU02_Data_Array[AXIS_3_ACCEL_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_3_ACCEL_2] & 0xFF) << 8);
         if ((Temp & 0x8000) == 0x8000)
         {
            # Sign Extend 16 bit value
            Temp = Temp | 0xFFFF0000;
         }
         Current_IMU_Data.BodyAccelerations['BODY_FRAME_Z'] =
                                                (float)Temp * ACCEL_SCALING;
   
         Temp =
            ((IMU02_Data_Array[AXIS_1_RATE_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_1_RATE_2] & 0xFF) << 8) |
            ((IMU02_Data_Array[AXIS_1_RATE_0] & 0xF0) << 12);
         if ((Temp & 0x80000) == 0x80000)
         {
            # Sign Extend 20 bit value
            Temp = Temp | 0xFFF00000;
         }
         Current_IMU_Data.BodyAngularRates['BODY_FRAME_X'] =
                                                (float)Temp * GYRO_SCALING;

         if ((Current_IMU_Data.BodyAngularRates['BODY_FRAME_X'] > 60.0) ||
             (Current_IMU_Data.BodyAngularRates['BODY_FRAME_X'] < -60.0))
         {
         	Current_IMU_Data.BodyAngularRates['BODY_FRAME_X'] = 
         	   Previous_IMU_Data.BodyAngularRates['BODY_FRAME_X'];
         }

         Temp =
            ((IMU02_Data_Array[AXIS_2_RATE_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_2_RATE_2] & 0xFF) << 8) |
            ((IMU02_Data_Array[AXIS_2_RATE_0] & 0xF0) << 12);
         if ((Temp & 0x80000) == 0x80000)
         {
            # Sign Extend 20 bit value
            Temp = Temp | 0xFFF00000;
         }
         Current_IMU_Data.BodyAngularRates['BODY_FRAME_Y'] =
                                                (float)Temp * GYRO_SCALING;

         if ((Current_IMU_Data.BodyAngularRates['BODY_FRAME_Y'] > 60.0) ||
             (Current_IMU_Data.BodyAngularRates['BODY_FRAME_Y'] < -60.0))
         {
         	Current_IMU_Data.BodyAngularRates['BODY_FRAME_Y'] = 
         	   Previous_IMU_Data.BodyAngularRates['BODY_FRAME_Y'];
         }
           
         Temp =
            ((IMU02_Data_Array[AXIS_3_RATE_1] & 0xFF0000) >> 16) |
            ((IMU02_Data_Array[AXIS_3_RATE_2] & 0xFF) << 8) |
            ((IMU02_Data_Array[AXIS_3_RATE_0] & 0xF0) << 12);
         if ((Temp & 0x80000) == 0x80000)
         {
            # Sign Extend 20 bit value
            Temp = Temp | 0xFFF00000;
         }
         Current_IMU_Data.BodyAngularRates['BODY_FRAME_Z'] =
                                                (float)Temp * GYRO_SCALING;

         if ((Current_IMU_Data.BodyAngularRates['BODY_FRAME_Z'] > 60.0) ||
             (Current_IMU_Data.BodyAngularRates['BODY_FRAME_Z'] < -60.0))
         {
         	Current_IMU_Data.BodyAngularRates['BODY_FRAME_Z'] = 
         	   Previous_IMU_Data.BodyAngularRates['BODY_FRAME_Z'];
         }
             
         IMU_Count++;
 
   
         # Resolve the IMU body frame data into SiNAV body frame.
         Current_IMU_Data.BodyAngularRates =
                        s3x3Matrix_s3dVector_Multiply(
                                       &IMUToBodyDCM,
                                       &Current_IMU_Data.BodyAngularRates);

         Current_IMU_Data.BodyAccelerations =
                        s3x3Matrix_s3dVector_Multiply(
                                       &IMUToBodyDCM,
                                       &Current_IMU_Data.BodyAccelerations);

      }
      else
      {
         #------------------------------------------------------------------
         # if no valid data, re-use the previous IMU data after updating the
         # TOV by adding the previous 'TimeDelta' value.
         #------------------------------------------------------------------
         Current_IMU_Data = Previous_IMU_Data;
         Current_IMU_Data.TimeOfValidity +=
                              (long_float)Current_IMU_Data.TimeDelta;

      }
   
      if (Real_IMU_Data_Received == TRUE)
      {
         # Only run filters if valid IMU data has been received
   
         # Update the 'old' IMU data
         Previous_IMU_Data = Current_IMU_Data;
      
         IMU_Data_Log_Message.TimeOfValidity =
               (uint32)(Current_IMU_Data.TimeOfValidity *
                                             REAL_TIME_TO_SINAV_TIME);
         IMU_Data_Log_Message.BodyXRate =
               Current_IMU_Data.BodyAngularRates['BODY_FRAME_X'];
         IMU_Data_Log_Message.BodyYRate =
               Current_IMU_Data.BodyAngularRates['BODY_FRAME_Y'];
         IMU_Data_Log_Message.BodyZRate =
               Current_IMU_Data.BodyAngularRates['BODY_FRAME_Z'];
         IMU_Data_Log_Message.AccX =
               Current_IMU_Data.BodyAccelerations['BODY_FRAME_X'];
         IMU_Data_Log_Message.AccY =
               Current_IMU_Data.BodyAccelerations['BODY_FRAME_Y'];
         IMU_Data_Log_Message.AccZ =
               Current_IMU_Data.BodyAccelerations['BODY_FRAME_Z'];
      
	  
         # Call NEF
         if (NEF_Status == NEF_NAVIGATING)
         {
            LCNApplication_UpdateNEF(Current_IMU_Data);
         }
         else
         {
            AHRS_Run(&Current_IMU_Data);
            if (SiNAV_Cycle_Count % AHRS_UPDATE_RATE == 0)
            {
            	AHRS_PreviousEulerAttitude[4] = AHRS_PreviousEulerAttitude[3];
            	AHRS_PreviousEulerAttitude[3] = AHRS_PreviousEulerAttitude[2];
            	AHRS_PreviousEulerAttitude[2] = AHRS_PreviousEulerAttitude[1];
            	AHRS_PreviousEulerAttitude[1] = AHRS_PreviousEulerAttitude[0];
            	AHRS_PreviousEulerAttitude[0] = AHRS_GetAttitude(); 
            }           	
         }
      }
      
         # ------------------------------
         # Schedule the following events.
         # ------------------------------
      
      if ((Real_IMU_Data_Received == TRUE) ||
          ((IMU02_Status & IMU02_RX_SYNTHETIC_INTERRUPT_IN_USE) ==
                                       IMU02_RX_SYNTHETIC_INTERRUPT_IN_USE))
      {
         # Allow GPS messages to be received in operational mode or
         # synthetic interrupt mode.
         if (GPS_SERIAL_RX_SCHEDULE)
         {
            # Check for received GPS messages.
#			Reset_SV_Status();
#			if (GPS_SV_COUNT_SCHEDULE)
#				No_Of_SVs = 0;
            GPS_Serial_Rx_Handler();
         }
         if (GPS_SV_COUNT_SCHEDULE)
         {
 #        	Send_String_to_SPORT1("oo ");
         	No_Of_SVs = 0;
         	Current_Time = Read_Nav_Time(SYSTEM_TIME);
         	for (j = 0; j < 32; j++)
         	{
         		if (Current_Time  - Get_Pseudo_Range_ToV_Data(j) < (REAL_TIME_TO_SINAV_TIME * 1.25))
          		   No_Of_SVs++;
         	}
 #        	if (Current_Time > 3450 * REAL_TIME_TO_SINAV_TIME)
 #        	Current_Time = Current_Time;
         }

         # Allow logging in operational mode or synthetic interrupt mode.
         if (OUTPUT_DATA_LOGGING_SCHEDULE)
         {
            if (Initialization_Complete == TRUE)
            {
               Output_Data_Logging(Current_IMU_Data.TimeOfValidity);
            }
         }
      }
	  
      # Count the NEF cycles...
      if (SiNAV_Cycle_Count == MAX_SiNAV_CYCLE_COUNT)
      {
         SiNAV_Cycle_Count = 0;
      }
      else
      {
         SiNAV_Cycle_Count++;
      }
   }

