package com.invensense.casdk.client;

import android.app.TabActivity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.os.Message;
import android.os.PowerManager;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.view.Window;
import android.webkit.WebView;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.RelativeLayout;
import android.widget.TabHost;
import android.widget.TabHost.TabSpec;
import android.widget.TextView;

import com.invensense.cubequat.Global;

/**
 * Main activity for organizing the tabs in application.
 * 
 * @author Invensense
 * 
 */
@SuppressWarnings("deprecation")
public class TabOrganizer extends TabActivity {
	PowerManager pm;
	PowerManager.WakeLock wl;

	protected LinearLayout optionsWindow;
	private static ImageView iv_battery;
	/**
	 * Button to perform remote calibration of wearable SDK from the application
	 */
	protected Button btnCalibMPU;
	/** Button to start and stop the logging */
	protected Button Logging;

	/** A popup window that display options menu when clicked */
	protected PopupWindow optionsPopupWindow;

	private static String Device_Model=null;
	private BroadcastReceiver screenoffReceiver;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().requestFeature(Window.FEATURE_NO_TITLE);
		setContentView(R.layout.view);

		pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
		wl = pm.newWakeLock(PowerManager.FULL_WAKE_LOCK, "My Tag");
		wl.acquire();


		Device_Model = android.os.Build.MODEL;
		iv_battery= (ImageView)findViewById(R.id.batteryManagement);
		
		
		final LinearLayout help = (LinearLayout) findViewById(R.id.help);
		help.setOnClickListener(tabOrganizerViewOnClickListener);

		final LinearLayout options = (LinearLayout) findViewById(R.id.options);
		options.setOnClickListener(tabOrganizerViewOnClickListener);

		optionsWindow = (LinearLayout) findViewById(R.id.options);
		LayoutInflater layoutInflater = (LayoutInflater) getBaseContext()
				.getSystemService(LAYOUT_INFLATER_SERVICE);
		View popupView = layoutInflater.inflate(R.layout.popup_options, null);
		optionsPopupWindow = new PopupWindow(popupView,
				LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT);
		optionsPopupWindow.setBackgroundDrawable(new BitmapDrawable());
		optionsPopupWindow.setOutsideTouchable(true);

		btnCalibMPU = (Button) popupView.findViewById(R.id.btn_calib_mpu);
		btnCalibMPU.setOnClickListener(tabOrganizerViewOnClickListener);
		Logging = (Button) popupView.findViewById(R.id.Logging);
		Logging.setOnClickListener(tabOrganizerViewOnClickListener);

		TabHost tabHost = getTabHost();
		TabSpec sensorspec = tabHost.newTabSpec("SENSORS");
		sensorspec.setIndicator("SENSORS", null/*
												 * res.getDrawable(R.color.White)
												 */);
		Intent sensorsIntent = new Intent(this, MainActivity.class);
		sensorspec.setContent(sensorsIntent);

		TabSpec contextspec = tabHost.newTabSpec("DETAILS");
		contextspec.setIndicator("DETAILS", null);
		Intent contextIntent = new Intent(this, CASDKUtilityActivity.class);
		contextspec.setContent(contextIntent);

		// TabSpec helpspec = tabHost.newTabSpec("HELP");
		// helpspec.setIndicator("HELP", null);
		// Intent helpIntent = new Intent(this, HelpActivity.class);
		// helpspec.setContent(helpIntent);

		tabHost.addTab(sensorspec);
		tabHost.addTab(contextspec);
		// tabHost.addTab(helpspec);

		// tabHost.getTabWidget().getChildAt(0).setBackgroundColor(Color.rgb(0xAF,
		// 0xAF, 0xAF));
		// tabHost.getTabWidget().getChildAt(1).setBackgroundColor(Color.rgb(0xAF,
		// 0xAF, 0xAF));
		// tabHost.getTabWidget().getChildAt(2).setBackgroundColor(Color.rgb(0xAF,
		// 0xAF, 0xAF));

		TextView tv = (TextView) tabHost.getTabWidget().getChildAt(0)
				.findViewById(android.R.id.title);
		tv.setTextColor(this.getResources().getColorStateList(
				R.color.text_tab_indicator));

		tv = (TextView) tabHost.getTabWidget().getChildAt(1)
				.findViewById(android.R.id.title);
		tv.setTextColor(this.getResources().getColorStateList(
				R.color.text_tab_indicator));

		// tv = (TextView)
		// tabHost.getTabWidget().getChildAt(2).findViewById(android.R.id.title);
		// tv.setTextColor(this.getResources().getColorStateList(R.color.text_tab_indicator));

		// IntentFilter filter = new IntentFilter(Intent.ACTION_SCREEN_ON);
		// filter.addAction(Intent.ACTION_SCREEN_OFF);
		// if (screenoffReceiver == null) {
		// screenoffReceiver = new BroadcastReceiver() {
		// @Override
		// public void onReceive(Context context, Intent intent) {
		// if (intent.getAction().equals(Intent.ACTION_SCREEN_OFF)) {
		// Log.v("screenoffReceiver", "SCREEN OFF");
		// wl.release();
		// Log.i("DebugApp", " Distroy");
		// if (screenoffReceiver != null)
		// unregisterReceiver(screenoffReceiver);
		// System.exit(0);
		// } else if (intent.getAction().equals(
		// Intent.ACTION_SCREEN_ON)) {
		// Log.v("screenoffReceiver", "SCREEN ON");
		// }
		// return;
		// }
		// };
		// registerReceiver(screenoffReceiver, filter);
		// }
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
		if (screenoffReceiver != null)
			unregisterReceiver(screenoffReceiver);
		Log.i("DebugApp", " Pause");
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onPause();
		if (screenoffReceiver != null) {
			IntentFilter filter = new IntentFilter(Intent.ACTION_SCREEN_ON);
			filter.addAction(Intent.ACTION_SCREEN_OFF);
			registerReceiver(screenoffReceiver, filter);
		}
		Log.i("DebugApp", " Pause");
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		wl.release();
		Log.i("DebugApp", " Distroy");
		if (screenoffReceiver != null)
			unregisterReceiver(screenoffReceiver);
		System.exit(0);
	}

	/**
	 * This method will be call if user clicks a Help icon that is present on
	 * the first page of the application on the top-right corner of the screen.
	 * */
	protected void onHelpClick() {
		final RelativeLayout helpwindow = (RelativeLayout) findViewById(R.id.helpwindow);
		LayoutInflater layoutInflater = (LayoutInflater) getBaseContext()
				.getSystemService(LAYOUT_INFLATER_SERVICE);
		View popupView = layoutInflater.inflate(R.layout.popup_help, null);
		WebView wv = (WebView) popupView.findViewById(R.id.wv_help);
		wv.loadUrl("file:///android_asset/casdk_help.html");
		
		final PopupWindow popupWindow = new PopupWindow(popupView,
				LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT);
		popupWindow.setBackgroundDrawable(new BitmapDrawable());
		popupWindow.setOutsideTouchable(true);
		popupWindow.showAtLocation(helpwindow, Gravity.CENTER, 0, 0);
	}

	/**
	 * This method will be called if user clicks the options icon, that is
	 * present on top-bar of application.
	 * */
	protected void onOptionsClick() {
		optionsPopupWindow.showAtLocation(optionsWindow, Gravity.CENTER, 0, 0);
	}

	/**
	 * OnClick method to get OnClick events set on several UI elements and then
	 * to trigger a particular function with respect to a UI element that is
	 * being clicked.
	 * */
	OnClickListener tabOrganizerViewOnClickListener = new OnClickListener() {

		@Override
		public void onClick(View v) {
			int id = v.getId();

			switch (id) {
			case R.id.help:
				onHelpClick();
				break;

			case R.id.options:
				onOptionsClick();
				break;

			case R.id.btn_calib_mpu:
				if (Global.MainActivityHandler != null) {
					Message msg = new Message();
					msg.obj = "invt";
					Global.MainActivityHandler.sendMessage(msg);
				}
				optionsPopupWindow.dismiss();
				break;

			case R.id.Logging:
				if (Global.startLogging == false) {
					Logging.setText("Stop Logging");
					optionsPopupWindow.dismiss();
					if (Global.CASDKUtilityActivityHandler != null) {
						Message msg = new Message();
						msg.arg1 = 2;
						msg.obj = "StartLogging";
						Global.CASDKUtilityActivityHandler.sendMessage(msg);
					}
					// MainActivity.startLog();
				} else {
					Logging.setText("Start Logging");
					optionsPopupWindow.dismiss();

					if (Global.CASDKUtilityActivityHandler != null) {
						Message msg = new Message();
						msg.arg1 = 2;
						msg.obj = "StopLogging";
						Global.CASDKUtilityActivityHandler.sendMessage(msg);
					}
					// MainActivity.stopLog();
				}

				break;

			}
		}
	};
	
	
	static void setBatteryStatus()
	{
		if (Global.status >=100) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery10);
			else
				iv_battery.setImageResource(R.drawable.battery10);
		}
		else if (Global.status >=90) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery9);
			else
				iv_battery.setImageResource(R.drawable.battery9);
		}
		else if (Global.status >=80) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery8);
			else
				iv_battery.setImageResource(R.drawable.battery8);
		}
		else if (Global.status >=70) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery7);
			else
				iv_battery.setImageResource(R.drawable.battery7);
		}
		else if (Global.status >=60) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery6);
			else
				iv_battery.setImageResource(R.drawable.battery6);
		}
		else if (Global.status >=50) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery5);
			else
				iv_battery.setImageResource(R.drawable.battery5);
		}
		else if (Global.status >=40) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery4);
			else
				iv_battery.setImageResource(R.drawable.battery4);
		}
		else if (Global.status >=30) {
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery3);
			else
				iv_battery.setImageResource(R.drawable.battery3);
		}
		
		else if (Global.status >=20) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery2);
			else
				iv_battery.setImageResource(R.drawable.battery2);
		}
		else if (Global.status >=10) 
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery1);
			else
				iv_battery.setImageResource(R.drawable.battery1);
		}
		else
		{
			if(Device_Model.compareToIgnoreCase("Galaxy Nexus")==0)
				iv_battery.setImageResource(R.drawable.battery);
			else
				iv_battery.setImageResource(R.drawable.battery);
		}	
	}
}