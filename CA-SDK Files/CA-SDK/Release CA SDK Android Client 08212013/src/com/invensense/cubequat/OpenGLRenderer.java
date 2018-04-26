package com.invensense.cubequat;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.content.Context;
import android.opengl.GLSurfaceView.Renderer;
import android.opengl.GLU;
import android.util.Log;

/**
 * This class is responsible for rendering a CubeView over GLSurfaceView.	
 *
 */
public class OpenGLRenderer implements Renderer  
{
	/**
	 * A model object to render rolling dice over surface view.
	 */
	private Cube cube;	
	private Context context;
	private int filter=0;

	public OpenGLRenderer(Context context2) 
	{
		this.context= context2;
	}
	

	@Override
	public void onDrawFrame(GL10 gl) {
	
		//Clear Screen And Depth Buffer
		gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);	
		gl.glLoadIdentity();					//Reset The Current Modelview Matrix
		
		//Drawing
		gl.glTranslatef(0.0f, 0.0f, -4);			//Move z units into the screen
		gl.glScalef(0.8f, 0.8f, 0.8f); 			//Scale the Cube to 80 percent, otherwise it would be too large for the screen
		
		//Log.i("test", "quat: " + Global.q[0] + "," + Global.q[1] + "," + Global.q[2] + "," + Global.q[3]);
		synchronized(Global.q) {
			
    	    gl.glRotatef((float)(2 * Math.acos(Global.q[0]) * 180 / 3.1415), Global.q[1], Global.q[2], Global.q[3]);
    	//   Log.i("i", " "+ Math.acos(Global.q[0])  * 180 / 3.1415+"  "+ Global.q[1]+"  "+Global.q[2]+"  "+ Global.q[3]);
    	}
		
		if (cube==null)
		{			
			cube= new Cube( gl,  context);
		}
		else{
			cube.draw(gl, filter);
		}		
	}

	@Override
	public void onSurfaceChanged(GL10 gl, int width, int height) {
		if(height == 0) { 						//Prevent A Divide By Zero By
			height = 1; 						//Making Height Equal One
		}

		gl.glViewport(0, 0, width, height); 	//Reset The Current Viewport
		gl.glMatrixMode(GL10.GL_PROJECTION); 	//Select The Projection Matrix
		gl.glLoadIdentity(); 					//Reset The Projection Matrix

		//Calculate The Aspect Ratio Of The Window
		GLU.gluPerspective(gl, 45.0f, (float)width / (float)height, 0.1f, 100.0f);

		gl.glMatrixMode(GL10.GL_MODELVIEW); 	//Select The Modelview Matrix
		gl.glLoadIdentity(); 	
	}

	@Override
	public void onSurfaceCreated(GL10 gl, EGLConfig config) {
		//Settings
		gl.glDisable(GL10.GL_DITHER);				//Disable dithering ( NEW )
		gl.glEnable(GL10.GL_TEXTURE_2D);			//Enable Texture Mapping
		gl.glShadeModel(GL10.GL_SMOOTH); 			//Enable Smooth Shading
		gl.glClearColor(0.0f, 0.5f, 0.6f, 0.7f); 	//Black Background	
		gl.glClearDepthf(1.0f); 					//Depth Buffer Setup
		gl.glEnable(GL10.GL_DEPTH_TEST); 			//Enables Depth Testing
		gl.glDepthFunc(GL10.GL_LEQUAL); 			//The Type Of Depth Testing To Do
		
		//Really Nice Perspective Calculations
		gl.glHint(GL10.GL_PERSPECTIVE_CORRECTION_HINT, GL10.GL_NICEST); 
	//	onDrawFrame(gl);
		
	}

}
