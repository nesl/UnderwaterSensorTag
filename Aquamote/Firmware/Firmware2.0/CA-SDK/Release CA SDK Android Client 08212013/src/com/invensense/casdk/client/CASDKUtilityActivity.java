package com.invensense.casdk.client;
/***
 * 
 * Uses OpenCSV open source code: http://opencsv.sourceforge.net/
 * 
 * 
 */
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import android.app.Activity;
import android.app.Fragment;
import android.app.FragmentManager;
import android.app.FragmentTransaction;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.view.ViewGroup;
import android.widget.AbsListView;
import android.widget.AbsListView.OnScrollListener;
import android.widget.ArrayAdapter;
import android.widget.BaseAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.ListView;
import android.widget.Switch;
import android.widget.TextView;
import au.com.bytecode.opencsv.CSVWriter;

import com.invensense.cubequat.Global;

/**
 * This Activity represents second page of the application. It includes the
 * fragmented layout to display rolling dice and values of different sensors
 * that are present on CA-Board along with set of commands to test different
 * features.
 * 
 * @author Invensense
 * 
 */

public class CASDKUtilityActivity extends Activity {
	/**List View of Commands for testing different features**/
	private ListView lvCommands;
	/**List View for displaying console messages**/
	private ListView lvMessages;
	/**List View for displaying different sensor values**/
	private ListView lvSensorsDisplay;
	/**This is another list view for swithces for sensor
	 * this needs because in case of updating sensor data 
	 * at fast rate, list view will be updating at fast rate
	 * will be making it difficult to intercept the touch events
	 *  for switches **/
	private ListView lv_sensor_Switch;
	private View clickSource;
	private View touchSource;
	int offset =0;
	
	/**Commands key map to send a command when clicked**/
	private LinkedHashMap<String, String> commandsMap;
	/**Commands key map to send sensor display related commands when clicked**/
	private LinkedHashMap<String, String> commandsMap2;
	
	/**Commands keys to send commands when clicked**/
	private ArrayList<String> commandsKeys;
	/**Commands key map to send sensor display related commands when clicked**/
	private ArrayList<String> commandsKeys2;
	/**Console messages list view adapter**/
	private ArrayAdapter<String> msgsArrayAdapter;
	/**Console messages list view adapter data source**/
	private LinkedList<String> msgs;

	/**Data source for sensor dispaly list.**/
	private ArrayList<String> displaySensorText;
	private ArrayList<String> displaySensorcommand;

	/**Sensors display switches list view adapter**/
	private DisplayListViewAdapterswitch displaySensorsAdapter_switch;
	/**Sensors display list view adapter**/
	private DisplayListViewAdapter displaySensorsAdapter;
	/**Commands list view adapter**/
	private CommandListViewAdapter lvCommandAdapter;
	

	/**Directory path where log files will be saved**/
	protected final static String LOG_FILES_DIR_PATH = "/mnt/sdcard/CASDKClient";

	/**Buffered writer to log data**/
	private BufferedWriter debugLogBufferedWriter;
	
	private CSVWriter writer = null;
	private CSVWriter external_writer = null;

	private String Device_Model=null;
	
	/**A array to maintain switches status as list view get redraw every time it scrolls**/
	private boolean[] checkedSatusOfSwitchCommandList;
	private boolean[] checkedSatusOfSwitchDisplayList;

	/**
	 * This function is called when the Activity is created
	 * 
	 * **/
	@Override
	protected void onCreate(Bundle savedInstanceState) {

		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_casdk_utility);
		Device_Model = android.os.Build.MODEL;
		
		if (savedInstanceState == null) {
			// During initial setup, plug in the details fragment.
			FragmentLayoutCube detailscube = new FragmentLayoutCube();
			FragmentManager frag = getFragmentManager();
			Fragment fragment_1= null;
			
				
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
			{
				fragment_1 = frag.findFragmentById(R.id.fragment_content_nexus);
			}
			else
			{
				fragment_1 = frag.findFragmentById(R.id.fragment_content2);
			}
			if (fragment_1 == null) 
			{
				final FragmentTransaction frt = frag.beginTransaction();
				if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				{
					frt.add(R.id.fragment_content_nexus, detailscube);
				}
				else
				{
					frt.add(R.id.fragment_content2, detailscube);
				}
				
				frt.commit();
			}
		}

		
		initCommandListView();
		initSensorLogView();

		lvMessages = (ListView) findViewById(R.id.lv_msgs);
		msgs = new LinkedList<String>();
		
		msgsArrayAdapter = new ArrayAdapter<String>(this,
				R.layout.msg_list_row, msgs);
		lvMessages.setAdapter(msgsArrayAdapter);

		File dirPath = new File(LOG_FILES_DIR_PATH);

		if (!dirPath.exists()) {
			dirPath.mkdir();
		}

		Global.CASDKUtilityActivityHandler = this.myHandler;

		setDefaultStatusFlagsForSwitchesOfCommandsList();
		setDefaultStatusFlagsForSwitchesOfDisplatList();
	}

	/**
	 * This function will be called when activity is resumed, need to reset the MSP430 flags
	 * by sending reset command.
	 */
	@Override
	public void onResume() {
		super.onResume();
		Global.CASDKUtilityActivityHandler .sendEmptyMessageDelayed(0, 100);

	}

	@Override
	protected void onPause() {
		super.onPause();
	}

	@Override
	public void onDestroy() {
		
		super.onDestroy();
	}

	/**
	 * Initialize commands list view with default values
	 */
	private void initCommandListView() {
		lvCommands = (ListView) findViewById(R.id.lv_commands);
		commandsMap = new LinkedHashMap<String, String>();

		commandsMap.put("01. Accelerometer Sensor", "inv8");
		commandsMap.put("02. Gyro Sensor", "inv9");
		commandsMap.put("03. Compass Sensor", "inv0");
		commandsMap.put("04. Magnetic Disturbance", "invm");
		commandsMap.put("05. Low Power Quaternions", "invv");
		commandsMap.put("06. DMP", "invf");
		commandsMap.put("07. Dump Gyro Register Values", "invd");
		commandsMap.put("08. Test Low Power Accel Mode", "invp");
		commandsMap.put("09. Load Calib Data", "invl");
		commandsMap.put("11. Store Calib Data", "invs");
		commandsMap.put("12. Self Test", "invt");
		commandsMap.put("13. Set Sensor Data Rate to 10 Hz", "inv1");
		commandsMap.put("14. Set Sensor Data Rate to 20 Hz", "inv2");
		commandsMap.put("15. Set Sensor Data Rate to 40 Hz", "inv3");
		commandsMap.put("16. Set Sensor Data Rate to 50 Hz", "inv4");
		commandsMap.put("17. Set Interrupt On Gesture Event Only", "inv,");
		commandsMap.put("18. Set Interrupt Periodically", "inv.");
		commandsMap.put("19. Reset Pedometer", "inv7");
		commandsMap.put("20. Reset Board", "invx");
		commandsMap.put("21. Set Motion Interrupt Mode", "invi");
		commandsMap.put("22. Get eMPL Version", "invn");
		commandsMap.put("23. Get Compass Status", "invw");
		commandsMap.put("24. Get Battery Status", "invy");

		commandsKeys = new ArrayList<String>(commandsMap.keySet());

		lvCommandAdapter = new CommandListViewAdapter();
		lvCommands.setAdapter(lvCommandAdapter);
	}

	/**
	 * Toggle the status of the flags of sensors for displaying data
	 * @param position position of sensor in display sensor list view
	 */
	private void toggleFlagsForDisplaySensorCommands(int position) {
		switch (position) {
		case 0:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_ACCEL;
			Log.i("CASDK", "Global.SensorLogStatus: "+Global.SensorLogStatus);
			break;

		case 1:
			Global.SensorLogStatus = Global.SensorLogStatus ^ Global.PRINT_GYRO;
			break;

		case 2:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_COMPASS;
			break;

		case 3:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_EULER;
			break;

		case 4:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_ROT_MAT;
			break;

		case 5:
			Global.SensorLogStatus = Global.SensorLogStatus ^ Global.PRINT_QUAT;
			break;

		case 6:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_HEADING;
			break;

		case 7:
			Global.SensorLogStatus = Global.SensorLogStatus
					^ Global.PRINT_OTHER_SENSORS;
			break;
		}
	}

	private static final int ACCEL_INDEX_IN_DISPLAY_LISTVIEW = 0;
	private static final int GYRO_INDEX_IN_DISPLAY_LISTVIEW = 1;
	private static final int COMPASS_INDEX_IN_DISPLAY_LISTVIEW = 2;
	private static final int EULER_INDEX_IN_DISPLAY_LISTVIEW = 3;
	private static final int QUAT_INDEX_IN_DISPLAY_LISTVIEW = 4;
	private static final int RM_INDEX_IN_DISPLAY_LISTVIEW = 5;
	private static final int HEADING_INDEX_IN_DISPLAY_LISTVIEW = 6;
	private static final int EXTERNAL_SENSORS_INDEX_IN_DISPLAY_LISTVIEW = 7;
	
	/**
	 * Initialize the display sensors data section of Activity.
	 */
	private void initSensorLogView() {

		lv_sensor_Switch= (ListView) findViewById(R.id.lv_sensor_switch);
		lvSensorsDisplay =(ListView) findViewById(R.id.lv_sensor_log);
		
		lvSensorsDisplay.setOnTouchListener(new OnTouchListener() {
		    @Override
		    public boolean onTouch(View v, MotionEvent event) {
		        if(touchSource == null)
		            touchSource = v;

		        if(v == touchSource) {
		        	lv_sensor_Switch.dispatchTouchEvent(event);
		            if(event.getAction() == MotionEvent.ACTION_UP) {
		                clickSource = v;
		                touchSource = null;
		            }
		        }

		        return false;
		    }
		});
		
		lv_sensor_Switch.setOnTouchListener(new OnTouchListener() {
		    @Override
		    public boolean onTouch(View v, MotionEvent event) {
		        if(touchSource == null)
		            touchSource = v;

		        if(v == touchSource) {
		        	lvSensorsDisplay.dispatchTouchEvent(event);
		            if(event.getAction() == MotionEvent.ACTION_UP) {
		                clickSource = v;
		                touchSource = null;
		            }
		        }

		        return false;
		    }
		});
		
		

		lvSensorsDisplay.setOnScrollListener(new OnScrollListener() {
		    @Override
		    public void onScroll(AbsListView view, int firstVisibleItem, int visibleItemCount, int totalItemCount) {
		        if(view == clickSource) 
		        	lv_sensor_Switch.setSelectionFromTop(firstVisibleItem, view.getChildAt(0).getTop() - offset);
		        
		        
		    }

		    @Override
		    public void onScrollStateChanged(AbsListView view, int scrollState) {}
		});
		
		
		lv_sensor_Switch.setOnScrollListener(new OnScrollListener() {
		    @Override
		    public void onScroll(AbsListView view, int firstVisibleItem, int visibleItemCount, int totalItemCount) {
		        if(view == clickSource) 
		        	lvSensorsDisplay.setSelectionFromTop(firstVisibleItem, view.getChildAt(0).getTop() + offset);
		    }

		    @Override
		    public void onScrollStateChanged(AbsListView view, int scrollState) {}
		});
		
		
		
		commandsMap2 = new LinkedHashMap<String, String>();
		
		commandsMap2.put("Accel(g): ", "inva");
		
		commandsMap2.put("Gyro(dpg): ", "invg");
		commandsMap2.put("Compass(uT):", "invc");
		commandsMap2.put("Euler(rad): ", "inve");
		commandsMap2.put("Quat: ", "invq");
		commandsMap2.put("Rot.Mat: ", "invr");
		commandsMap2.put("Heading(deg): ", "invh");
		commandsMap2.put("External Sensors: ", "invk");
		displaySensorcommand  = new ArrayList<String>(commandsMap2.keySet());
		
		displaySensorText  = new ArrayList<String>(commandsMap2.keySet());

		displaySensorsAdapter = new DisplayListViewAdapter();
		lvSensorsDisplay.setAdapter(displaySensorsAdapter);
		
		displaySensorsAdapter_switch = new DisplayListViewAdapterswitch();
		lv_sensor_Switch.setAdapter(displaySensorsAdapter_switch);
		

	}

	/**
	 * Stop logging of sensor data
	 */
	private void stopLogging() {
		try {
			debugLogBufferedWriter.close();
			debugLogBufferedWriter = null;
			writer.close();
			writer = null;
			
			if(external_writer!= null)
			{
				external_writer.close();
				external_writer= null;
			}
			
			Global.startLogging = false;
			

		} catch (IOException e) {
			Log.i("eMPLClient", "Unable to Stop Logging");
		}
	}


	private DateFormat df = new SimpleDateFormat("MM_dd_yyyy_HH_mm_ss");

	/**
	 * Start logging of the sensors which are currently on.
	 */
	private void startLogging() {
		Date current = Calendar.getInstance().getTime();

		String filename_debug = "debug_" + df.format(current) + ".txt";
		String filename_sensor = "sensors_" + df.format(current) + ".csv";

		try {
			File root = new File(LOG_FILES_DIR_PATH);
			if (root.canWrite()) {
				File debugLogFile = new File(root, filename_debug);
				FileWriter debugLogWriter = new FileWriter(debugLogFile);
				debugLogBufferedWriter = new BufferedWriter(debugLogWriter);

				File sensorLogFile = new File(root, filename_sensor);
				writer = new CSVWriter(new FileWriter(sensorLogFile), ',');
				
				
				String sensorLogFileHeader = new String();

				if ((Global.SensorLogStatus & Global.PRINT_ACCEL) > 0)
					sensorLogFileHeader=("Accel[X]," + "Accel[Y],"
							+ "Accel[Z],");
				if ((Global.SensorLogStatus & Global.PRINT_GYRO) > 0)
					sensorLogFileHeader+=("Gyro[X]," + "Gyro[Y],"
							+ "Gyro[Z],");
				if ((Global.SensorLogStatus & Global.PRINT_COMPASS) > 0)
					sensorLogFileHeader+=("Compass[X]," + "Compass[Y],"
							+ "Compass[Z],");

				if ((Global.SensorLogStatus & Global.PRINT_QUAT) > 0)
					sensorLogFileHeader+=("Quat[W]," + "Quat[X],"
						+ "Quat[Y]," + "Quat[Z],");
				if ((Global.SensorLogStatus & Global.PRINT_EULER) > 0)
					sensorLogFileHeader+=("Euler[X]," + "Euler[Y],"
						+ "Euler[Z],");
				if ((Global.SensorLogStatus & Global.PRINT_ROT_MAT) > 0)
					sensorLogFileHeader+=("RM[0]," + "RM[1],"
						+ "RM[2]," + "RM[3]," + "RM[4]," + "RM[5],"
						+ "RM[6]," + "RM[7]," + "RM[8],");
				if ((Global.SensorLogStatus & Global.PRINT_HEADING) > 0)
					sensorLogFileHeader+=("Heading,");
				String[] entries= sensorLogFileHeader.split(",");
				if(entries !=null)
				writer.writeNext(entries); 
				

				if ((Global.SensorLogStatus & Global.PRINT_OTHER_SENSORS) > 0)
				{
					if (external_writer == null) {
						Date current_1 = Calendar.getInstance().getTime();
						String filename_sensor_1 = "sensors_" + df.format(current_1)
								+ "_external.csv";
	
						
							File root_1 = new File(LOG_FILES_DIR_PATH);
							if (root_1.canWrite()) {
								File sensorLogFile_1 = new File(root_1, filename_sensor_1);
								external_writer= new CSVWriter(new FileWriter(sensorLogFile_1), ',');
								String sensor_external = new String();
	
	
								
									sensor_external+=("Pressure," + "Temperature,"
											+ "UV Index," + "Humidity, Light");
								
	
								String[] entries_1= sensor_external.split(",");
								external_writer.writeNext(entries_1); 
							}
					}
				}
				Global.startLogging = true;
				 
				new LoggerThread().start();
			}
		} catch (IOException e) {
			Log.e("eMPL Client", "Could not write file " + e.getMessage());
		}
	}

	/**
	 * Write console messages into the file.
	 * @param msg
	 */
	private void writeDebugLog(String msg) {
		Date current = Calendar.getInstance().getTime();
		try {
			if (debugLogBufferedWriter != null) {
				debugLogBufferedWriter.write(df.format(current) + "\t" + msg
						+ "\r\n");
			}
		} catch (IOException e) {
			Log.i("CASDK", "Unable to write Debug log");
		}
	}

	/**
	 * 
	 * Write sensor data log to the file.
	 */
	private void writeSensorsDataLog() {

		if ( writer != null) {
			String sensorLogStr = new String();
			
	
			
				
			if ((Global.SensorLogStatus & Global.PRINT_ACCEL) > 0)
				sensorLogStr += Global.Accel[0] + "," + Global.Accel[1] + ","
						+ Global.Accel[2] + ",";
			if ((Global.SensorLogStatus & Global.PRINT_GYRO) > 0)
				sensorLogStr += Global.Gyro[0] + "," + Global.Gyro[1] + ","
						+ Global.Gyro[2] + ",";
			if ((Global.SensorLogStatus & Global.PRINT_COMPASS) > 0)
				sensorLogStr += Global.Compass[0] + "," + Global.Compass[1]
						+ "," + Global.Compass[2] + ",";
			
			if ((Global.SensorLogStatus & Global.PRINT_QUAT) > 0)
				sensorLogStr += Global.q[0] + "," + Global.q[1] + "," + Global.q[2]
					+ "," + Global.q[3] + ",";
			
			if ((Global.SensorLogStatus & Global.PRINT_EULER) > 0)
				sensorLogStr += Global.eular[0] + "," + Global.eular[1] + ","
					+ Global.eular[2] + ",";
			
			if ((Global.SensorLogStatus & Global.PRINT_ROT_MAT) > 0)
				sensorLogStr += Global.RM[0] + "," + Global.RM[1] + ","
					+ Global.RM[2] + "," + Global.RM[3] + "," + Global.RM[4]
					+ "," + Global.RM[5] + "," + Global.RM[6] + ","
					+ Global.RM[7] + "," + Global.RM[8] + ",";
			
			if ((Global.SensorLogStatus & Global.PRINT_HEADING) > 0)
				sensorLogStr += Global.Heading + ",";
			
			

			if (sensorLogStr.length() > 0)
				if (sensorLogStr.charAt(sensorLogStr.length() - 1) == ',')
					sensorLogStr = sensorLogStr.substring(0,
							sensorLogStr.length() - 1);

			String[] entries= sensorLogStr.split(",");
			if (entries != null)
			writer.writeNext(entries); 
		}
			
			if ((Global.SensorLogStatus & Global.PRINT_OTHER_SENSORS) > 0)
				{
					if (external_writer != null) 
					{
						{
							String sensorLogStr_e = new String();
							
						
							sensorLogStr_e += Global.Pressure + "," + Global.Tempreture + ","
								+ Global.UV + "," + Global.Humidity + ","+Global.Light+",";
							String[] entries_e= sensorLogStr_e.split(",");
							if (entries_e != null)
							external_writer.writeNext(entries_e); 
						
						}
					}
				}
	}



	private LinkedList<Message> loggingMsgs = new LinkedList<Message>();
	private LinkedList<Integer> quatLoggingMsgs = new LinkedList<Integer>();
	private static int LOG_MESSAGE_WRITE_DEBUG_LOG = 0;
	private static int LOG_MESSAGE_WRITE_SENSOR_LOG = 1;

	/**
	 * @author Jibran Ahmed
	 * A separate sensor logger thread to write sensor data into the file.
	 *
	 */
	private class LoggerThread extends Thread {
		@Override
		public void run() {
			while (Global.startLogging) {
				if (loggingMsgs.size() > 1) {
					Message msg = loggingMsgs.removeFirst();
					if (msg.arg1 == LOG_MESSAGE_WRITE_DEBUG_LOG) {
						writeDebugLog((String) msg.obj);
					} else if (msg.arg1 == LOG_MESSAGE_WRITE_SENSOR_LOG) {
						writeSensorsDataLog();
					}
				} else {
					try {
						sleep(40);
					} catch (InterruptedException e) {
						Log.i("CA-SDK", "Logging thread interrupted exception");
					}
				}
			}
			super.run();
		}
	};


	/**
	 * An activity handler class to update the data and UI from background thread.
	 * Please refer to handleMessage definition.
	 */
	public Handler myHandler = new Handler() {
		public void handleMessage(android.os.Message msg) {
			
			if (msg.arg1 == 0) {
				String str = (String) msg.obj;
				str = str.replaceAll("[\n\r]", "");

				if (msgs.size() < 1000) {
					msgs.addLast(str);
				} else {
					msgs.removeFirst();
					msgs.addLast(str);
				}
				msgsArrayAdapter.notifyDataSetChanged();
			
				if (Global.startLogging) {
					// writeDebugLog(str);
					Message logMsg = new Message();
					logMsg.arg1 = LOG_MESSAGE_WRITE_DEBUG_LOG;
					logMsg.obj = str;
					loggingMsgs.addLast(logMsg);
				}

			} 
			else if (msg.arg1 == 1)
			{ 
				
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_OTHER_SENSORS) {
						displaySensorText.set(EXTERNAL_SENSORS_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("External Sensors:\nPressure(hPa): %.2f \nTemperature(C): %.2f\n"+
										"UV Index: %.2f\nHumidity(%%): %.2f\nLight: %.2f", 
										Global.Pressure, Global.Tempreture,Global.UV,
										Global.Humidity, Global.Light));
	
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_ACCEL) {
						displaySensorText.set(ACCEL_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("Accel(g):\n%.2f  %.2f  %.2f",
										Global.Accel[0], Global.Accel[1],
										Global.Accel[2]));
					
						
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_GYRO) {
						displaySensorText.set(GYRO_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("Gyro(dps):\n%.2f  %.2f  %.2f",
										Global.Gyro[0], Global.Gyro[1],
										Global.Gyro[2]));
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_COMPASS) {
						displaySensorText.set(COMPASS_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("Compass(uT):\n%.2f  %.2f  %.2f",
										Global.Compass[0], Global.Compass[1],
										Global.Compass[2]));
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_EULER) {
						displaySensorText.set(EULER_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("Euler(rad):\n%.2f  %.2f  %.2f",
										Global.eular[0], Global.eular[1],
										Global.eular[2]));
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_HEADING) {
						displaySensorText
								.set(HEADING_INDEX_IN_DISPLAY_LISTVIEW, String
										.format("Heading(deg): %.2f",
												Global.Heading));
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_ROT) {
						displaySensorText.set(RM_INDEX_IN_DISPLAY_LISTVIEW, String
								.format("Rot. Mat:\n " 
										+"  %.2f  %.2f  %.2f\n"
										+"	%.2f  %.2f  %.2f\n"
										+"	%.2f  %.2f  %.2f", Global.RM[0],
										Global.RM[1], Global.RM[2], Global.RM[3],
										Global.RM[4], Global.RM[5], Global.RM[6],
										Global.RM[7], Global.RM[8]));
					}
	
					if (msg.arg2 == BluetoothDataReader.PACKET_DATA_QUAT) {
						displaySensorText.set(QUAT_INDEX_IN_DISPLAY_LISTVIEW,
								String.format("Quat:\n%.2f  %.2f  %.2f  %.2f",
										Global.q[0], Global.q[1], Global.q[2],
										Global.q[3]));
					}

					displaySensorsAdapter.notifyDataSetChanged();

				if (Global.startLogging) {
					// writeSensorsDataLog();
					Message logMsg = new Message();
					logMsg.arg1 = LOG_MESSAGE_WRITE_SENSOR_LOG;
					loggingMsgs.addLast(logMsg);
					
				}

			} else if (msg.arg1 == 2) {
				String command = (String) msg.obj;
				if (command.equalsIgnoreCase("StopLogging")) {
					stopLogging();
				}
				else if (command.equalsIgnoreCase("StartLogging")) 
				{
					
					startLogging();
					
				}

				//lvCommands.invalidateViews();
				
				lvSensorsDisplay.invalidateViews();
				lv_sensor_Switch.invalidateViews();
			} else if (msg.arg1 == 3) {
				quatLoggingMsgs.addLast(0);
			}
		};

	};

	OnClickListener listsViewClickListener = new OnClickListener() {

		@Override
		public void onClick(View v) {
			v.getTag(R.id.lv_msgs);

		}
	};

	/**
	 * 
	 * Cache class for holding CommandListView to make rendering and other things faster.
	 *
	 */
	private class CommandListViewHolder {
		private TextView tvCommandDesc;
		private Button btnCommandSend;
		private Switch swToggleCommands;
	};

	private class CommandListViewAdapter extends BaseAdapter {

		@Override
		public int getCount() {
			return commandsKeys.size();
		}

		@Override
		public Object getItem(int position) {
			return null;
		}

		@Override
		public long getItemId(int position) {
			return 0;
		}

		@Override
		public int getItemViewType(int position) {
			if (position >= 0 && position <= 5) {
				return 0;
			} else {
				return 1;
			}
		}

		@Override
		public int getViewTypeCount() {
			return 2;
		}

		@Override
		public View getView(int position, View convertView, ViewGroup parent) {

			CommandListViewHolder viewHolder;
			if (convertView == null) {
				LayoutInflater inflater = (LayoutInflater) CASDKUtilityActivity.this
						.getSystemService(Context.LAYOUT_INFLATER_SERVICE);

				if (position >= 0 && position <= 5) {

					convertView = inflater.inflate(R.layout.command_list_row_command,
							null);
				} else {
					convertView = inflater.inflate(R.layout.command_list_row2,
							null);
				}

				viewHolder = new CommandListViewHolder();

				if (position >= 0 && position <= 5) {
					viewHolder.swToggleCommands = (Switch) convertView
							.findViewById(R.id.sw_command);
					if (viewHolder.swToggleCommands != null) {
						viewHolder.swToggleCommands.setId(position);

						viewHolder.swToggleCommands
								.setOnCheckedChangeListener(null);
						viewHolder.swToggleCommands
								.setChecked(checkedSatusOfSwitchCommandList[position]);
						viewHolder.swToggleCommands
								.setOnCheckedChangeListener(commandListViewItemOnCheckedChangeListener);
					}

				} else {
					viewHolder.btnCommandSend = (Button) convertView
							.findViewById(R.id.btn_command);
					if (viewHolder.btnCommandSend != null) {
						viewHolder.btnCommandSend.setId(position);
						viewHolder.btnCommandSend
								.setOnClickListener(commandListViewItemClickListener);
					}
				}

				viewHolder.tvCommandDesc = (TextView) convertView
						.findViewById(R.id.tv_command);
				if (viewHolder.tvCommandDesc != null)
					viewHolder.tvCommandDesc
							.setText(commandsKeys.get(position));

				convertView.setTag(viewHolder);

			}

			viewHolder = (CommandListViewHolder) convertView.getTag();
			viewHolder.tvCommandDesc.setText(commandsKeys.get(position));
			if (position >= 0 && position <= 5) {
				viewHolder.swToggleCommands.setId(position);

				viewHolder.swToggleCommands.setOnCheckedChangeListener(null);
				viewHolder.swToggleCommands
						.setChecked(checkedSatusOfSwitchCommandList[position]);
				viewHolder.swToggleCommands
						.setOnCheckedChangeListener(commandListViewItemOnCheckedChangeListener);

			} else {
				viewHolder.btnCommandSend.setId(position);
				viewHolder.btnCommandSend
						.setOnClickListener(commandListViewItemClickListener);
			}
			return convertView;
		}
	};
	/**
	 * 
	 * Display List view holder class to cache the views of display list view for fast rendering.
	 *
	 */
	private class DisplayListViewHolder {
		private TextView tvDisplay;
		private Switch swDisplay;
	}

	private class DisplayListViewAdapterswitch extends BaseAdapter {
		@Override
		public int getCount() {
			return displaySensorText.size();
		}

		@Override
		public Object getItem(int position) {
			return null;
		}

		@Override
		public long getItemId(int position) {
			return 0;
		}
		
		@Override
		public View getView(int position, View convertView, ViewGroup parent) {

			DisplayListViewHolder viewHolder_display;
			if (convertView == null) {
				LayoutInflater inflater = (LayoutInflater) CASDKUtilityActivity.this
						.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
				convertView = inflater.inflate(R.layout.command_list_row, null);

				viewHolder_display = new DisplayListViewHolder();

				 int[] loc = new int[2];
				 lv_sensor_Switch.getLocationInWindow(loc);
				
				 


			        // Save listView's y and get listView2's coordinates
			        int firstY = loc[1];
			        lvSensorsDisplay.getLocationInWindow(loc);
			        
			        offset = firstY - loc[1];
			 		
				
			        viewHolder_display.swDisplay = (Switch) convertView.findViewById(R.id.sw_command);
				if (viewHolder_display.swDisplay != null) {
					viewHolder_display.swDisplay.setId(position);

					viewHolder_display.swDisplay.setOnCheckedChangeListener(null);
					viewHolder_display.swDisplay.setChecked(checkedSatusOfSwitchDisplayList[position]);
					viewHolder_display.swDisplay.setOnCheckedChangeListener(displayListViewItemCheckedChangeListener);
						 
				}				
				convertView.setTag(viewHolder_display);
			}

			viewHolder_display = (DisplayListViewHolder) convertView.getTag();
			viewHolder_display.swDisplay.setId(position);
			viewHolder_display.swDisplay.setOnCheckedChangeListener(null);
			viewHolder_display.swDisplay.setChecked(checkedSatusOfSwitchDisplayList[position]);
			viewHolder_display.swDisplay.setOnCheckedChangeListener(displayListViewItemCheckedChangeListener);

			if(Global.startLogging== true)
				viewHolder_display.swDisplay.setEnabled(false);
			else
				viewHolder_display.swDisplay.setEnabled(true);

			return convertView;
		}
	};
	
	private class DisplayListViewAdapter extends BaseAdapter {
		@Override
		public int getCount() {
			return displaySensorText.size();
		}

		@Override
		public Object getItem(int position) {
			return null;
		}

		@Override
		public long getItemId(int position) {
			return 0;
		}
		
		@Override
		public View getView(int position, View convertView, ViewGroup parent) {

			DisplayListViewHolder viewHolder_display;
			if (convertView == null) {
				LayoutInflater inflater = (LayoutInflater) CASDKUtilityActivity.this
						.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
				convertView = inflater.inflate(R.layout.command_list_row_text, null);

				viewHolder_display = new DisplayListViewHolder();
		
				 int[] loc = new int[2];
				 lvSensorsDisplay.getLocationInWindow(loc);

			        // Save listView's y and get listView2's coordinates
			        int firstY = loc[1];
			        lv_sensor_Switch.getLocationInWindow(loc);

			        offset = firstY - loc[1];
				

				viewHolder_display.tvDisplay = (TextView) convertView.findViewById(R.id.tv_command_text);
				if (viewHolder_display.tvDisplay != null  )
					viewHolder_display.tvDisplay.setText(displaySensorText.get(position));
					
					
				
				convertView.setTag(viewHolder_display);
			}

		viewHolder_display = (DisplayListViewHolder) convertView.getTag();
 		viewHolder_display.tvDisplay.setText(displaySensorText.get(position));

			return convertView;
		}
	};

	/**
	 * This command list view OnClickListener is intended for those list view items
	 * in which a simple button is present instead of compound button like run self test etc.
	 */
	private OnClickListener commandListViewItemClickListener = new OnClickListener() {

		@Override
		public void onClick(View v) {
			int id = v.getId();
			String key = commandsKeys.get(id);
			String command = commandsMap.get(key);
			Message msg = new Message();
			msg.obj = command;
			Global.MainActivityHandler.sendMessage(msg);
		}
	};

	/**
	 * Command list switches OnCheckedChangelistener, for more information please
	 * refer to android official documentation
	 */
	private OnCheckedChangeListener commandListViewItemOnCheckedChangeListener = new OnCheckedChangeListener() {

		@Override
		public void onCheckedChanged(CompoundButton buttonView,
				boolean isChecked) {

			int id = buttonView.getId();
			//toggleFlagsForDisplaySensorCommands(id);
			String key = commandsKeys.get(id);
			String command = commandsMap.get(key);
			Log.i("CA-SDK", "Command : " + command);
			Message msg = new Message();
			msg.obj = command;
			Global.MainActivityHandler.sendMessage(msg);

			checkedSatusOfSwitchCommandList[id] = buttonView.isChecked();

		}
	};

	/**
	 * Display list switches OnCheckedChangelistener, for more information please
	 * refer to android official documentation
	 */
	private OnCheckedChangeListener displayListViewItemCheckedChangeListener = new OnCheckedChangeListener() {

		@Override
		public void onCheckedChanged(CompoundButton buttonView,
				boolean isChecked) {
			
			int id = buttonView.getId();
			Log.i("CA-SDK", "id : " + id);
			toggleFlagsForDisplaySensorCommands(id);
		
			String key = displaySensorcommand.get(id);
			String command = commandsMap2.get(key);
			Log.i("CA-SDK", "Command : " + command);
			Message msg = new Message();
			msg.obj = command;
			Log.i("CA-SDK", "msg : " + msg);
			Global.MainActivityHandler.sendMessage(msg);
			
			
			checkedSatusOfSwitchDisplayList[id] = buttonView.isChecked();
			

		}
	};

	/**
	 * set default status flags of switches for command list
	 */
	private void setDefaultStatusFlagsForSwitchesOfCommandsList() {
		checkedSatusOfSwitchCommandList = new boolean[commandsKeys.size()];
		for (int i = 0; i < commandsKeys.size(); i++) {
			switch (i) {
			case 0:
			case 1:
			case 2:
			case 14:
				checkedSatusOfSwitchCommandList[i] = true;
				break;

			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
			case 11:
			case 12:
			case 13:
				checkedSatusOfSwitchCommandList[i] = false;

				break;
			}
		}
	}

	/**
	 * Set the switch status to default for display sensor list
	 */
	
	private void setDefaultStatusFlagsForSwitchesOfDisplatList() {
		checkedSatusOfSwitchDisplayList = new boolean[displaySensorText.size()];
		for (int i = 0; i < displaySensorText.size(); i++) {
			checkedSatusOfSwitchDisplayList[i] = false;
			
		}
	}
	

}
