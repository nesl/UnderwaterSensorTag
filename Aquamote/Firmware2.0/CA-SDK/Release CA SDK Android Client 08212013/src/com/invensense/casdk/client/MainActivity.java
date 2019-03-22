package com.invensense.casdk.client;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.DecimalFormat;
import java.util.UUID;

import org.achartengine.ChartFactory;
import org.achartengine.GraphicalView;
import org.achartengine.chart.PointStyle;
import org.achartengine.model.XYMultipleSeriesDataset;
import org.achartengine.model.XYSeries;
import org.achartengine.renderer.XYMultipleSeriesRenderer;
import org.achartengine.renderer.XYSeriesRenderer;

import android.app.Activity;
import android.app.Fragment;
import android.app.FragmentManager;
import android.app.FragmentTransaction;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Paint.Align;
import android.graphics.drawable.BitmapDrawable;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.view.ViewGroup.LayoutParams;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.invensense.cubequat.Global;

/**
 * This Activity represents first page of the application. It includes the
 * fragmented layout to display rolling dice and values of different sensors
 * that are present on wearable device board.
 * 
 * @author Invensense
 * 
 */
public class MainActivity extends Activity implements OnClickListener {
	/** Called when the activity is first created. */

	/** Bluetooth Adapter of the device **/
	BluetoothAdapter mBluetoothAdapter = null;
	/** Bluetooth Device that is intended to get the information */
	BluetoothDevice mBluetoothDevice = null;
	/** A Bluetooth socket that connects the device to stream the data */
	BluetoothSocket mBluetoothSocket = null;
	/** A flag indicating the connectivity of the socket */
	Boolean socketConnected = false;
	/** Bluetooth socket output stream */
	OutputStream mBluetoothOS = null;

	protected static final int Total_values = 100;

	/** Compass history for graph view */
	protected float compass_historyValue[] = new float[Total_values];
	int compass_historyValue_count = 0;

	/** UV history for graph view */
	protected float UV_historyValue[] = new float[Total_values];
	int UV_historyValue_count = 0;
	/** humidity history for graph view */
	protected float humidity_historyValue[] = new float[Total_values];
	int humidity_historyValue_count = 0;

	/** light history for graph view */
	protected float light_historyValue[] = new float[Total_values];
	int light_historyValue_count = 0;

	/** temperature history for graph view */
	protected float tempreture_historyValue[] = new float[Total_values];
	int tempreture_historyValue_count = 0;

	/** pressure history for graph view */
	protected float pressure_historyValue[] = new float[Total_values];
	int pressure_historyValue_count = 0;

	/**
	 * UI variables to display the several UI elements while the program is
	 * running
	 */
	RelativeLayout heading_sensor;
	RelativeLayout uv_sensor;
	RelativeLayout tempreture_sensor;
	RelativeLayout pressure_sensor;
	RelativeLayout light_sensor;
	RelativeLayout humidity_sensor;

	LinearLayout heading_background;
	LinearLayout uv_background;
	LinearLayout temperature_background;
	LinearLayout pressure_background;
	LinearLayout light_background;
	LinearLayout humidity_background;

	TextView sensor_name;
	TextView sensor_value;

	TextView heading;
	ImageView iv_heading;
	ImageView iv_compass_needle;
	TextView UVindex;
	ImageView iv_uvindex;
	TextView hummidity;
	ImageView iv_humidity;
	TextView light;
	ImageView iv_light;
	TextView tempreture;
	ImageView iv_tempreture;
	TextView pressure;
	ImageView iv_pressure;

	TextView heading_title;
	TextView uv_title;
	TextView humidity_title;
	TextView light_title;
	TextView temperature_title;
	TextView pressure_title;

	static ImageView iv_pressure_needle;

	/** Bluetooth input stream to read the incoming data from the device */
	private InputStream mBluetoothIS = null;
	/**
	 * A utility class instance that provides the easy read operation from
	 * bluetooth data stream.
	 */
	private BluetoothDataReader mBluetoothDataReader;
	public static byte buffer[] = new byte[23];
	protected static final UUID MY_UUID = UUID
			.fromString("00001101-0000-1000-8000-00805f9b34fb");

	protected static final int REQUEST_CONNECT_DEVICE = 1;
	protected static final int REQUEST_ENABLE_BT = 2;

	protected RelativeLayout whole_page;
	protected long startTime;

	/**
	 * A class extension of Thread to perform the sensor update.
	 * 
	 * */
	private SensorsUpdateThread mSensorUpdateThread;
	
	private String Device_Model;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		whole_page = (RelativeLayout) findViewById(R.id.whole_page);

		if (savedInstanceState == null) {
			// During initial setup, plug in the details fragment.
			final FragmentLayout details = new FragmentLayout();
			final FragmentManager fm = 	getFragmentManager();
			
			final Fragment fragment = fm.findFragmentById(R.id.fragment_content);
			
			if (fragment == null) {
				final FragmentTransaction ft = fm.beginTransaction();
				
				ft.add(R.id.fragment_content, details);

				ft.commit();
				this.getFragmentManager().executePendingTransactions(); 
			}
		}
 
		Device_Model = android.os.Build.MODEL;					 

		heading_sensor = (RelativeLayout) findViewById(R.id.heading_sensor);
		uv_sensor = (RelativeLayout) findViewById(R.id.uv_sensor);
		tempreture_sensor = (RelativeLayout) findViewById(R.id.tempreture_sensor);
		pressure_sensor = (RelativeLayout) findViewById(R.id.pressure_sensor);
		light_sensor = (RelativeLayout) findViewById(R.id.light_sensor);
		humidity_sensor = (RelativeLayout) findViewById(R.id.humidity_sensor);

		heading_background = (LinearLayout) findViewById(R.id.heading_background);
		uv_background = (LinearLayout) findViewById(R.id.uv_background);
		temperature_background = (LinearLayout) findViewById(R.id.temperature_background);
		pressure_background = (LinearLayout) findViewById(R.id.pressure_background);
		light_background = (LinearLayout) findViewById(R.id.light_background);
		humidity_background = (LinearLayout) findViewById(R.id.humidity_background);

		heading_title = (TextView) findViewById(R.id.tv_heading);
		uv_title = (TextView) findViewById(R.id.tv_uvindex);
		temperature_title = (TextView) findViewById(R.id.tv_temperature);
		pressure_title = (TextView) findViewById(R.id.tv_pressure);
		light_title = (TextView) findViewById(R.id.tv_light);
		humidity_title = (TextView) findViewById(R.id.tv_humidity);

		heading = (TextView) findViewById(R.id.tv_heading_value);
		UVindex = (TextView) findViewById(R.id.tv_uvindex_value);
		hummidity = (TextView) findViewById(R.id.tv_humidity_value);
		light = (TextView) findViewById(R.id.tv_light_value);
		tempreture = (TextView) findViewById(R.id.tv_temperature_value);
		pressure = (TextView) findViewById(R.id.tv_pressure_value);
		iv_compass_needle = (ImageView) findViewById(R.id.iv_compass_needle);
		iv_pressure_needle = (ImageView) findViewById(R.id.iv_pressure_needle);
		iv_heading = (ImageView) findViewById(R.id.iv_heading);
		iv_uvindex = (ImageView) findViewById(R.id.iv_uvindex);
		iv_humidity = (ImageView) findViewById(R.id.iv_humidity);
		iv_light = (ImageView) findViewById(R.id.iv_light);
		iv_tempreture = (ImageView) findViewById(R.id.iv_temperature);
		iv_pressure = (ImageView) findViewById(R.id.iv_pressure);

		heading_sensor.setTag("iv_heading");
		uv_sensor.setTag("iv_uvindex");
		humidity_sensor.setTag("iv_humidity");
		light_sensor.setTag("iv_light");
		tempreture_sensor.setTag("iv_tempreture");
		pressure_sensor.setTag("iv_pressure");

		putZero();
		heading_sensor.setOnClickListener(this);
		humidity_sensor.setOnClickListener(this);
		light_sensor.setOnClickListener(this);
		pressure_sensor.setOnClickListener(this);
		tempreture_sensor.setOnClickListener(this);
		uv_sensor.setOnClickListener(this);

		Global.MainActivityHandler = myHandler;
		
		/** Bluetooth setup **/
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
			return;
		}
		socketConnected = false;

		startBluetooth();
	}

	/**
	 * resets the history values
	 */
	protected void putZero() {
		for (int i = 0; i < Total_values; i++) {
			compass_historyValue[i] = 0;
			UV_historyValue[i] = 0;
			humidity_historyValue[i] = 0;
			light_historyValue[i] = 0;
			tempreture_historyValue[i] = 0;
			pressure_historyValue[i] = 0;
		}

		/*
		 * new Thread(new Runnable() {
		 * 
		 * @Override public void run() { myWunderGroundClient = new
		 * WundergroundClient(MainActivity.this);
		 * myWunderGroundClient.getWeatherData();
		 * 
		 * } }).start();
		 */
	}

	@Override
	public void onResume() {
		super.onResume();
	}

	private boolean isDestroyed = false;

	@Override
	public void onDestroy() {
		super.onDestroy();

		isDestroyed = true;
		/*
		 * if(!(mBluetoothDataReader.isCancelled()) &&
		 * mBluetoothDataReader.running){ mBluetoothDataReader.running = false;
		 * mBluetoothDataReader.cancel(true); // mBluetoothDataReader = null; }
		 */
		if (socketConnected) {
			sendCommand("invx");
			if (!(mBluetoothDataReader.isCancelled())
					&& mBluetoothDataReader.running) {
				mBluetoothDataReader.running = false;
				mBluetoothDataReader.cancel(true);
				// mBluetoothDataReader = null;
			}
			try {
				mBluetoothOS.close();
				mBluetoothIS.close();
				mBluetoothSocket.close();
			} catch (IOException e) {
				Log.i("MainActivity", "exception thrown on close");
			}
			mBluetoothSocket = null;
			socketConnected = false;
			mSensorUpdateThread.stopSensorThread();
		}

	}

	/**
	 * starts the bluetooth connection
	 */
	public void startBluetooth() {
		if (!mBluetoothAdapter.isEnabled() && !socketConnected) {
			Intent enableIntent = new Intent(
					BluetoothAdapter.ACTION_REQUEST_ENABLE);
			startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
		} else {
			showDevicesForSelection();
		}
	}

	/**
	 * shows a list of bluetooth devices to select from
	 */
	public void showDevicesForSelection() {

		Intent serverIntent = new Intent(this, DeviceListActivity.class);
		this.startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);

	}

	/**
	 * This will be called once the DeviceListActivity is finished to make a
	 * connection with selected device if user has selected a device while
	 * DeviceListActivity.java was running.
	 * */
	public void onActivityResult(int requestCode, int resultCode, Intent data) {
		Log.i("MainActivity", "RequestCode " + requestCode + " ResultCode "
				+ resultCode);
		switch (requestCode) {
		case REQUEST_CONNECT_DEVICE:
			if (resultCode == Activity.RESULT_OK) {

				String address = data.getExtras().getString(
						DeviceListActivity.EXTRA_DEVICE_ADDRESS);
				mBluetoothDevice = mBluetoothAdapter.getRemoteDevice(address);
				if (mBluetoothDevice == null) {
					Log.i("MainActivity", "BLUETOOTH DEVICE IS NULL");

				} else {

					Log.i("MainActivity", "Setting up connection....");
					// SetupConnection();

					
						try {
							mBluetoothSocket = mBluetoothDevice.createInsecureRfcommSocketToServiceRecord(MY_UUID);
						} catch (IOException e4) {
							// TODO Auto-generated catch block
							e4.printStackTrace();
						}
						Log.i("MainActivity", "got socket");
						mBluetoothAdapter.cancelDiscovery();
						
					try {
						mBluetoothSocket.connect();
						Log.i("MainActivity", "socket connected");
						Log.i("MainActivity", "connecting streams");
						mBluetoothOS = mBluetoothSocket.getOutputStream();
						mBluetoothIS = mBluetoothSocket.getInputStream();
						Log.i("MainActivity", "streams connected!");
					} catch (IOException e) {
						Log.i("MainActivity", "failed to connect socket");
						try {
							if (mBluetoothIS != null)
								mBluetoothIS.close();
							if (mBluetoothOS != null)
								mBluetoothOS.close();
							if (mBluetoothSocket != null)
								mBluetoothSocket.close();
						} catch (IOException e2) {
						}

						startBluetooth();
						return;
					}

					Toast notify = Toast.makeText(this,
							"Connected to bluetooth!", Toast.LENGTH_LONG);
					notify.setGravity(Gravity.CENTER, 0, 0);
					notify.show();
					socketConnected = true;

					mBluetoothDataReader = new BluetoothDataReader();
					mBluetoothDataReader.execute(mBluetoothIS);
					mSensorUpdateThread = new SensorsUpdateThread();
					mSensorUpdateThread.start();
				}
			}
			break;
		case REQUEST_ENABLE_BT:
			if (resultCode != Activity.RESULT_OK) {
				Toast.makeText(this, "Bluetooth needs to be enabled",
						Toast.LENGTH_SHORT).show();
			} else {
				showDevicesForSelection();
			}
			break;
		}
	}

	/**
	 * This thread class is responsible to send the messages to the Handler with
	 * an interval in case if the bluetooth is connected successfully and able
	 * to receive data stream.
	 * 
	 */
	class SensorsUpdateThread extends Thread implements Runnable {

		@Override
		public void run() {
			super.run();
			while (mBluetoothDataReader.running) {
				double currTime = (System.currentTimeMillis() - startTime) / 1000.0;
				if (currTime > 0.2) {
					sensorUpdateThreadHandler.sendEmptyMessage(0);
					currTime = 0;
					startTime = System.currentTimeMillis();
				}
			}
			try {
				if(mBluetoothIS != null) mBluetoothIS.close();
				if(mBluetoothOS != null) mBluetoothOS.close();
				if(mBluetoothSocket != null) mBluetoothSocket.close();
			} catch (IOException e) {
			} finally {
				if (!isDestroyed)
					startBluetooth();
			}
		}

		public void stopSensorThread() {
			mBluetoothDataReader.running = false;
		}
	};

	private long PrevTime=0;
	/** A companion handler to update the sensors UI */
	private Handler sensorUpdateThreadHandler = new Handler() {
		public void handleMessage(android.os.Message msg) {

			FragmentLayout.UpdateText();

			DecimalFormat df = new DecimalFormat("#.#");
			UVindex.setText(df.format(Global.UV) + "  ");
			hummidity.setText(df.format(Global.Humidity) + "%");
			light.setText(df.format(Global.Light) + " lx");
			tempreture.setText(df.format(Global.Tempreture) + (char) 0x00B0
					+ "C");

			DecimalFormat d_o = new DecimalFormat("#");
			pressure.setText(d_o.format(Global.Pressure) + " hPa");

			setHeading();
			setUV();
			setHumidity();
			setLight();
			setTempreture();
			setPressure();
			TabOrganizer.setBatteryStatus();
			long currTime= System.currentTimeMillis();
			if(currTime - PrevTime >120000)
			{
				Message msgbattery = new Message();
				msgbattery.obj = "invy";
				Log.i("CA-SDK", "getting battery status : " + msgbattery);
				Global.MainActivityHandler.sendMessage(msgbattery);
				PrevTime= currTime;
			}
			
		}
	};

	/**
	 * Updates the pressure graph on the UI
	 */
	protected void setPressure() {
		iv_pressure.setScaleX(0.4f);
		iv_pressure.setScaleY(0.4f);
		// iv_pressure_needle.setRotation(Global.Pressure);

		if (pressure_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				pressure_historyValue[i] = pressure_historyValue[i + 1];
			}
			pressure_historyValue_count = Total_values - 1;
		}
		pressure_historyValue[pressure_historyValue_count] = Global.Pressure;

		if (popup_flag_pressure == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(pressure_historyValue, pressure_historyValue_count,
					"Pressure", "hPa");
		}

		pressure_historyValue_count++;

		setElevation();
	}

	static float altitude = 0;

	/**
	 * sets the Global elevation variable
	 */
	protected static void setElevation() {
		float pressureReference = 0;
		if (Global.Pressure_Reference != -1)
			pressureReference = Global.Pressure_Reference;
		else
			pressureReference = SensorManager.PRESSURE_STANDARD_ATMOSPHERE;

		Global.Elevation = (float) ((1 - Math.pow(Global.Pressure
				/ pressureReference, 0.190284)) * 145366.45);
		iv_pressure_needle.setRotation(Global.Elevation);
		Global.Elevation *= 0.3048f;
		altitude = Global.Elevation;
		iv_pressure_needle.setRotation(altitude);
		/*
		 * Log.i("DebugApp", "Elevation " + Global.Elevation + " ref pressure "
		 * + Global.Pressure_Reference);
		 */
	}

	/**
	 * Updates the temperature graphic on the UI and the graph
	 */
	protected void setTempreture() {
		iv_tempreture.setScaleX(0.5f);
		iv_tempreture.setScaleY(0.5f);
		if (Global.Tempreture <= 10) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_0_nexus);
			else
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_0);
		}
		else if (Global.Tempreture <= 20) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_1_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_1);
		}
		else if (Global.Tempreture <= 30) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_2_nexus);
			else
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_2);
		} 
		else if (Global.Tempreture <= 40) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_3_nexus);
			else
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_3);
		}
		else if (Global.Tempreture <= 50) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_4_nexus);
			else
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_4);
		}
		else if (Global.Tempreture <= 60) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_5_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_5);
		}
		else if (Global.Tempreture <= 70) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_6_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_6);
		} 
		else if (Global.Tempreture <= 80) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_7_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_7);
		} 
		else if (Global.Tempreture <= 90) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_8_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_8);
		}
		else
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_tempreture.setImageResource(R.drawable.sensor_thermometer_9_nexus);
			else
			iv_tempreture.setImageResource(R.drawable.sensor_thermometer_9);
		}
		if (tempreture_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				tempreture_historyValue[i] = tempreture_historyValue[i + 1];
			}
			tempreture_historyValue_count = Total_values - 1;
		}
		tempreture_historyValue[tempreture_historyValue_count] = Global.Tempreture;

		if (popup_flag_temperature == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(tempreture_historyValue, tempreture_historyValue_count,
					"Temperature", (char) 0x00B0 + "C");
		}

		tempreture_historyValue_count++;

	}

	/**
	 * Updates the light graphic on the UI. Also updates the light graph on the
	 * UI.
	 */
	protected void setLight() {
		iv_light.setScaleX(0.5f);
		iv_light.setScaleY(0.5f);
		if (Global.Light < 10) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_light.setImageResource(R.drawable.sensor_ambient_light_0_nexus);
			else
				iv_light.setImageResource(R.drawable.sensor_ambient_light_0);
		} else if (Global.Light < 200) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_light.setImageResource(R.drawable.sensor_ambient_light_1_nexus);
			else
			iv_light.setImageResource(R.drawable.sensor_ambient_light_1);
		} else if (Global.Light <= 500) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_light.setImageResource(R.drawable.sensor_ambient_light_2_nexus);
			else
			iv_light.setImageResource(R.drawable.sensor_ambient_light_2);
		}
		else 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_light.setImageResource(R.drawable.sensor_ambient_light_3_nexus);
			else
			iv_light.setImageResource(R.drawable.sensor_ambient_light_3);
		}
		if (light_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				light_historyValue[i] = light_historyValue[i + 1];
			}
			light_historyValue_count = Total_values - 1;
		}
		light_historyValue[light_historyValue_count] = Global.Light;

		if (popup_flag_light == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(light_historyValue, light_historyValue_count, "Light",
					"lx");
		}

		light_historyValue_count++;
	}

	/**
	 * Updates the humidty graph and graphic on the UI.
	 */
	protected void setHumidity() {
		iv_humidity.setScaleX(0.5f);
		iv_humidity.setScaleY(0.5f);
		if (Global.Humidity < 20) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_1_nexus);
			else
				iv_humidity.setImageResource(R.drawable.humidity_1);
		} 
		else if (Global.Humidity < 40) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_2_nexus);
			else
			iv_humidity.setImageResource(R.drawable.humidity_2);
		} 
		else if (Global.Humidity < 60) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_3_nexus);
			else
			iv_humidity.setImageResource(R.drawable.humidity_3);
		}
		else if (Global.Humidity < 80) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_4_nexus);
			else
			iv_humidity.setImageResource(R.drawable.humidity_4);
		} 
		else if (Global.Humidity < 90) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_5_nexus);
			else
			iv_humidity.setImageResource(R.drawable.humidity_5);
		} 
		else 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_humidity.setImageResource(R.drawable.humidity_6_nexus);
			else
			iv_humidity.setImageResource(R.drawable.humidity_6);
		}

		if (humidity_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				humidity_historyValue[i] = humidity_historyValue[i + 1];
			}
			humidity_historyValue_count = Total_values - 1;
		}
		humidity_historyValue[humidity_historyValue_count] = Global.Humidity;

		if (popup_flag_humidity == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(humidity_historyValue, humidity_historyValue_count,
					"Humidity", "%");
		}

		humidity_historyValue_count++;

	}

	/**
	 * Updates the UV graphic and graph on the UI.
	 */
	protected void setUV() {
		iv_uvindex.setScaleX(0.5f);
		iv_uvindex.setScaleY(0.5f);
		if (Global.UV < 3) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_0_nexus);
			else
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_0);
		} 
		else if (Global.UV < 6) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_1_nexus);
			else
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_1);
		}
		else if (Global.UV <= 8) 
		{	
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_2_nexus);
			else
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_2);
		} 
		else
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_3_nexus);
			else
				iv_uvindex.setImageResource(R.drawable.sensor_uv_index_3);
		}
		if (UV_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				UV_historyValue[i] = UV_historyValue[i + 1];
			}
			UV_historyValue_count = Total_values - 1;
		}
		UV_historyValue[UV_historyValue_count] = Global.UV;

		if (popup_flag_UV == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(UV_historyValue, UV_historyValue_count, "UV Index", " ");
		}
		UV_historyValue_count++;
	}

	/**
	 * Updates the Heading graphic and graph on the UI.
	 */
	protected void setHeading() {

		String text = new String();
		iv_compass_needle.setRotation(Global.eular[2]);

		if (Global.eular[2] >= -22.5 && Global.eular[2] <= 22.5) {
			text = "N";
		} else if (Global.eular[2] > 22.5 && Global.eular[2] < 72.5) {
			text = "NE";
		} else if (Global.eular[2] >= 72.5 && Global.eular[2] <= 112.5) {
			text = "E";
			
		} else if (Global.eular[2] > 112.5 && Global.eular[2] < 157.5) {
			text = "SE";
			
		} else if (Global.eular[2] >= 157.5 || Global.eular[2] <= -157.5) {
			text = "S";
			
		} else if (Global.eular[2] > -157.5 && Global.eular[2] < -112.5) {
			text = "SW";
			
		} else if (Global.eular[2] > -72.5 && Global.eular[2] < -22.5) {
			text = "NW";
			
		} else if (Global.eular[2] <= -72.5 && Global.eular[2] >= -112.5) {
			text = "W";
			
		}
		heading.setText(text);

		if (compass_historyValue_count >= Total_values) {
			for (int i = 0; i < Total_values - 1; i++) {
				compass_historyValue[i] = compass_historyValue[i + 1];
			}
			compass_historyValue_count = Total_values - 1;
		}
		if (Global.eular[2] > 0)
			compass_historyValue[compass_historyValue_count] = Global.eular[2];
		else
			compass_historyValue[compass_historyValue_count] = (360 + Global.eular[2]);

		if (popup_flag_heading == true) {
			lineDataSeries.clear();
			lineView.repaint();// repainting
			makeGraph(compass_historyValue, compass_historyValue_count,
					"Compass", (char) 0x00B0 + "");
		}
		compass_historyValue_count++;

	}

	@Override
	protected void onPause() {
		super.onPause();
		
	}

	/** variables for displaying graphs */
	protected static GraphicalView lineView;
	protected static XYSeries lineDataSeries;
	protected static XYMultipleSeriesRenderer lineRenderer = new XYMultipleSeriesRenderer();
	protected static double range[] = new double[] { 0, 10, 0, 4 };
	protected static PopupWindow popupWindow;
	protected static View popupView;
	protected static LayoutInflater layoutInflater;

	/**
	 * This function handles the on click for all the sensor boxes to display a
	 * pop-up with graphical representation showing the trend of change in
	 * values with time
	 * 
	 * */
	@Override
	public void onClick(View v) {
		if (v.getTag() == "iv_heading" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(compass_historyValue, compass_historyValue_count,
					"Compass", (char) 0x00B0 + "");
			popup_flag_heading = true;
			Log.i("test", "setting background");

			heading_title.setBackgroundColor(0xFFEF4023);

		}

		if (v.getTag() == "iv_humidity" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(humidity_historyValue, humidity_historyValue_count,
					"Humidity", "%");
			popup_flag_humidity = true;

			humidity_title.setBackgroundColor(0xFFEF4023);
		}

		if (v.getTag() == "iv_light" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(light_historyValue, light_historyValue_count, "Light",
					"lx");
			popup_flag_light = true;

			light_title.setBackgroundColor(0xFFEF4023);
		}

		if (v.getTag() == "iv_tempreture" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(tempreture_historyValue, tempreture_historyValue_count,
					"Temperature", (char) 0x00B0 + "C");
			popup_flag_temperature = true;
			temperature_title.setBackgroundColor(0xFFEF4023);
		}
		if (v.getTag() == "iv_pressure" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(pressure_historyValue, pressure_historyValue_count,
					"Pressure", "hPa");
			popup_flag_pressure = true;
			pressure_title.setBackgroundColor(0xFFEF4023);
		}
		if (v.getTag() == "iv_uvindex" && flag_dismiss == true) {
			flag_dismiss = false;
			makepopUpWindow();
			makeGraph(UV_historyValue, UV_historyValue_count, "UV Index", " ");
			popup_flag_UV = true;
			uv_title.setBackgroundColor(0xFFEF4023);
		}
	}

	/**
	 * Makes a popup windows that displays the sensor graph
	 */
	protected void makepopUpWindow() {
		layoutInflater = (LayoutInflater) getBaseContext().getSystemService(
				LAYOUT_INFLATER_SERVICE);
		popupView = layoutInflater.inflate(R.layout.sensor_popup, null);
		popupWindow = new PopupWindow(popupView, LayoutParams.WRAP_CONTENT,
				LayoutParams.WRAP_CONTENT);
		popupWindow.setOutsideTouchable(true);
		popupWindow.setBackgroundDrawable(new BitmapDrawable());
		// when a touch even happens outside of the window
		// make the window go away
		popupWindow.setTouchInterceptor(new View.OnTouchListener() {
			@Override
			public boolean onTouch(View arg0, MotionEvent event) {
				if (event.getAction() == MotionEvent.ACTION_OUTSIDE) {
					popupWindow.dismiss();
					flag_dismiss = true;
					popup_flag_heading = false;
					popup_flag_UV = false;
					popup_flag_humidity = false;
					popup_flag_light = false;
					popup_flag_temperature = false;
					popup_flag_pressure = false;

					heading_title.setBackgroundColor(0xFF0099cc);
					uv_title.setBackgroundColor(0xFF0099cc);
					humidity_title.setBackgroundColor(0xFF0099cc);
					light_title.setBackgroundColor(0xFF0099cc);
					temperature_title.setBackgroundColor(0xFF0099cc);
					pressure_title.setBackgroundColor(0xFF0099cc);
					return true;
				}
				return false;
			}
		});

		popupWindow.showAtLocation(whole_page, Gravity.CENTER, 0, 0);
	}

	/** variables for the graph popup */
	protected boolean popup_flag_heading = false;
	protected boolean popup_flag_UV = false;
	protected boolean popup_flag_humidity = false;
	protected boolean popup_flag_light = false;
	protected boolean popup_flag_temperature = false;
	protected boolean popup_flag_pressure = false;
	protected boolean flag_dismiss = true;

	/**
	 * Initializes and makes the graph.
	 * 
	 * @param values
	 *            graph values
	 * @param count
	 *            size of values
	 * @param name
	 *            name of the grpah
	 * @param measure
	 *            Unitys of the graph
	 */
	protected void makeGraph(float[] values, int count, String name,
			String measure) {
		DecimalFormat df = new DecimalFormat("#.##");
		float max = find_largest(values, count);
		float min = find_smallest(values, count);
		float avg = find_average(values, count);

		XYSeries series = new XYSeries("");
		lineDataSeries = series;
		lineRenderer = getLineRenderer();

		lineView = ChartFactory.getLineChartView(this, getLineDataSet(),
				lineRenderer);
		FrameLayout setlayout = (FrameLayout) popupView
				.findViewById(R.id.sensor_chart_view);
		sensor_name = (TextView) popupView.findViewById(R.id.sensor_name);
		sensor_value = (TextView) popupView.findViewById(R.id.sensor_value);

		sensor_value.setText(name);
		sensor_name.setText("Average:  " + df.format(avg) + "" + measure);

		setlayout.addView(lineView, ViewGroup.LayoutParams.MATCH_PARENT,
				ViewGroup.LayoutParams.MATCH_PARENT);

		range = new double[] { 0, (count * 0.2), min - 2, max + 2 };
		lineRenderer.setRange(range);
		lineDataSeries.add(0, values[0]);
		int data_count = 0;
		for (float c = 0.2f; c < (count * 0.2); c = c + 0.2f) {
			if (data_count >= 100)
				break;
			lineDataSeries.add(c, values[data_count]);
			data_count++;

			if (Build.VERSION.SDK_INT >= 14)
				lineView.repaint();
		}

	}

	/**
	 * finds the largest value in the graph
	 * 
	 * @param values
	 *            array of values
	 * @param count
	 *            size of values
	 * @return the largest value found
	 */
	protected float find_largest(float[] values, int count) {
		float max = values[0];
		for (int i = 1; i < count; i++) {
			if (max < values[i])
				max = values[i];
		}
		return max;
	}

	/**
	 * finds the smallest value in an array
	 * 
	 * @param values
	 *            array of values
	 * @param count
	 *            size of values
	 * @return the smallest value found
	 */
	protected float find_smallest(float[] values, int count) {
		float min = values[0];
		for (int i = 1; i < count; i++) {
			if (min > values[i])
				min = values[i];
		}
		return min;
	}

	/**
	 * solves the averages of an array of values
	 * 
	 * @param values
	 *            array of values
	 * @param count
	 *            size of values
	 * @return the average calculated
	 */
	protected float find_average(float[] values, int count) {
		float avg = values[0];
		for (int i = 1; i < count; i++) {
			avg += values[i];
		}
		return (avg / count);
	}

	/**
	 * used for the graph
	 * 
	 * @return data set for grpah
	 */
	protected XYMultipleSeriesDataset getLineDataSet() {
		XYMultipleSeriesDataset dataset = new XYMultipleSeriesDataset();

		dataset.addSeries(lineDataSeries);

		return dataset;
	}

	/**
	 * used for the graph
	 * 
	 * @return renderer needed for the graph
	 */
	protected XYMultipleSeriesRenderer getLineRenderer() {
		XYMultipleSeriesRenderer renderer = new XYMultipleSeriesRenderer();
		renderer.setAxisTitleTextSize(16);
		renderer.setChartTitleTextSize(20);
		renderer.setLabelsTextSize(15);
		renderer.setLegendTextSize(20);

		renderer.setMarginsColor(Color.WHITE);
		renderer.setBackgroundColor(Color.WHITE);

		XYSeriesRenderer r = new XYSeriesRenderer();
		r.setColor(Color.rgb(37, 178, 231));
		r.setPointStyle(PointStyle.POINT);
		r.setFillBelowLine(true);
		r.setFillBelowLineColor(Color.rgb(37, 178, 231));
		r.setFillPoints(true);

		renderer.addSeriesRenderer(r);

		renderer.setYLabelsAlign(Align.RIGHT);
		renderer.setAxesColor(Color.rgb(37, 178, 231));
		renderer.setLabelsColor(Color.rgb(37, 178, 231));

		return renderer;
	}

	private void sendCommand(String command) {
		try {
			if (socketConnected) {
				byte[] cmd = command.getBytes();
				for (int ii = 0; ii < cmd.length; ii++) {
					mBluetoothOS.write(cmd[ii]);
					long currentTime = System.currentTimeMillis();
					while(System.currentTimeMillis() - currentTime < 50){ ; }
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private Handler myHandler = new Handler(){
		public void handleMessage(android.os.Message msg) {
			String command = (String)msg.obj;
			sendCommand(command);
			Log.i("CA-SDK", "in command : " + command);
		};
	};
}