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
 * This is the small fragmented window that displays a rotating
 *  dice to illustrate the use of six-axis quaternions obtained
 *  from wearable SDK.
 * @author Invensense
 *
 */
public class FragmentLayoutCube extends Fragment 
{
	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
			Bundle savedInstanceState) {
		View view = inflater.inflate(R.layout.fragmentedview_cube, container, false);
		
		return view;
	}
}
