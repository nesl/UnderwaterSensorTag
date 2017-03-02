package com.invensense.casdk.client;
import android.app.Activity;
import android.os.Bundle;

/**
 * This class is intended to display the help view
 * by rendering an HTML.
 * @author Jibran Ahmed
 *
 */
public class HelpActivity extends Activity {

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		setContentView(R.layout.activity_help);
	}
}
