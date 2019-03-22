package com.invensense.cubequat;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;

/**
 * CubeView is the class that extend GLSurfaceView and it will acts as a View class to 
 * display a rolling dice cube on the application.
 * 
 */

public class CubeView extends GLSurfaceView implements Callback {

	/**
	 * An openGL renderer object that will render the cube on GLSurfaceView.
	 */
	private OpenGLRenderer renderer;

	/**
	 * A view constructor that will initialize the view.
	 * @param context Desired activity context.
	 * @param attributeSet Layout Attribute set to layout configuration of the view.
	 */
	public CubeView(Context context, AttributeSet attributeSet) {
		super(context, attributeSet);

		renderer = new OpenGLRenderer(context);
		setRenderer(renderer);

		getHolder().addCallback(this);
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		// TODO Auto-generated method stub
		super.surfaceCreated(holder);

	}

	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
		// TODO Auto-generated method stub
		super.surfaceChanged(holder, format, w, h);

	}

}
