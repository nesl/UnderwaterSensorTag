package com.invensense.casdk.client;

import java.text.DecimalFormat;

import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.invensense.casdk.client.R;
import com.invensense.cubequat.Global;

/**
 * This is the small fragmented window that outputs the Roll
 * Pitch Yaw angles and displays a rotating dice to illustrate
 * the use of six-axis quaternions obtained from wearable SDK.
 * @author Invensense
 *
 */
public class FragmentLayout extends Fragment 
{
	/** TextView that displays
	 * eular angle values of the rotation vector */
	protected static TextView eular;

	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
			Bundle savedInstanceState) {
		View view = inflater.inflate(R.layout.fragmentedview, container, false);
		eular = (TextView) view.findViewById(R.id.eular);

		UpdateText();

		return view;
	}

	/**
	 * Updates the euler angles on the UI
	 */
	public static void UpdateText() {
		DecimalFormat df = new DecimalFormat("#");
		double data0 = Math.round(Global.eular[0]);	
		double data1 = Math.round(Global.eular[1]);	
		double data2 = Math.round(Global.eular[2]);	
		String sa[] ={df.format(data0),df.format(data1),df.format(data2)};
		double data[]={data0,data1,data2};
		/*
		 * Updated the code here to align it, hope there is a easier
		 * way to alight the format. May be create a text box for each Pitch, Roll 
		 * and yaw and align it using the xml itself. 
		 * This is a quick fix. 
		 */
		for(int i=0;i<3;i++){
			if(data[i]>=0){
				sa[i]="  "+sa[i];
			}
			
			if(Math.abs(data[i])<100){
				sa[i]="  "+sa[i];
				if(Math.abs(data[i])<10){
					sa[i]="  "+sa[i];	
				}
			}
		}
		eular.setText("  ROLL:  " + sa[0] +  (char) 0x00B0 + ""
			    + "\n\n  PITCH: " + sa[1] +  (char) 0x00B0 + ""
			    + "\n\n  YAW:    " + sa[2] +  (char) 0x00B0 + ""
			    );

	}

}
