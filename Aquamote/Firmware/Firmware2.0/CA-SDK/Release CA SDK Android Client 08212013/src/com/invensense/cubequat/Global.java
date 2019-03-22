package com.invensense.cubequat;

import java.util.LinkedList;

import android.os.Handler;

/**
 *
 * This is a Global class that is primarily intended to provide a bridge between the sensor values
 * that are shared between different classes and activities. A better alternative of class could be
 * Listener pattern.
 * 
 */
public class Global {
	
	public static final int PRINT_ACCEL     = (0x01);
	public static final int PRINT_GYRO      = (0x02);
	public static final int PRINT_QUAT      = (0x04);
	public static final int PRINT_COMPASS   = (0x08);
	public static final int PRINT_EULER     = (0x10);
	public static final int PRINT_ROT_MAT   = (0x20);
	public static final int PRINT_HEADING   = (0x40);
	public static final int PRINT_PEDO      = (0x80);
	public static final int PRINT_OTHER_SENSORS	= (0x100);

	/**
	 * Six-axis quaternion values that are helpful in displaying a rolling dice.
	 */
	    public static float[] q = new float[4];
	    static {
	    	q[0] = (float) 1.0;
	    	q[1] = (float) 0.0;
	    	q[2] = (float) 0.0;
	    	q[3] = (float) 0.0;
	    }
	    
	    /**
	     * Euler angle values
	     */
	    public static float[] eular = new float[3];
	    static{
	    	eular[0]= 0f;
			eular[1]= 0f;
			eular[2]= 0f;
	    }
	    /**
	     * Current activity information
	     */
		public static String currActivity="Starting...";
		/**
		 * Confidence level of current calculated activity in percentage.
		 */
		public static int confidenceLevel = 0;
		/**
		 * Gyro data
		 */
		public static float[] Gyro= new float[3];
		/**
		 * Accelerometer data
		 */
		public static float[] Accel= new float[3];
		/**
		 * Compass data if available
		 */
		public static float[] Compass= new float[3];
		/**
		 * Pressure Data
		 */
		public static float Pressure=0;
		/**
		 * Reference pressure value that will be used to calculate the elevation.
		 */
		public static float Pressure_Reference= -1;
		/**
		 * Elevation value.
		 */
		public static float Elevation= 0;
		/**
		 * Humidity value.
		 */
		public static float Humidity=0;
		/**
		 * Temperature value.
		 */
		public static float Tempreture=0;
		/**
		 * Light value.
		 */
		public static float Light=0;
		/**
		 * UV Index value.
		 */
		public static float UV=0;
		
		/**
		 * A start and stop flag for data logging.
		 */
		public static boolean startLogging=false;
		
		public static float Heading = 0f;
		
		public static float[] RM = new float[9];
				
		public static Handler CASDKUtilityActivityHandler;

		public static Handler MainActivityHandler;
		
		public static int SensorLogStatus = 0;
		public static double status= 20.0;
		
}