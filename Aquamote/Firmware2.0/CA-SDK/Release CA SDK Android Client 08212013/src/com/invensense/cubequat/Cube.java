package com.invensense.cubequat;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;   
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import com.invensense.casdk.client.R;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLUtils;
import android.util.Log;

/** This class is the model class to for the rolling dice view to 
 * provide the abstract data that is necessary to setup and display the 
 * cube*/
public class Cube {
	
	/** The buffer holding the vertices */
	private FloatBuffer vertexBuffer;
	/** The buffer holding the texture coordinates */
	private FloatBuffer textureBuffer;

	/** The buffer holding the normals */
	private FloatBuffer normalBuffer;
	
	private int[] textures = new int[6]; // i use 6 faces textures,, near and linear... 
	
	
	public static float CubeSizef = 0.7f;
	private float vertices[]={ 
			// FRONT
			-CubeSizef, -CubeSizef, CubeSizef,
			 CubeSizef, -CubeSizef, CubeSizef,
			-CubeSizef,  CubeSizef, CubeSizef,
			 CubeSizef,  CubeSizef, CubeSizef,

			// BACK
			-CubeSizef, -CubeSizef, -CubeSizef,
			-CubeSizef,  CubeSizef, -CubeSizef,
			 CubeSizef, -CubeSizef, -CubeSizef,
			 CubeSizef,  CubeSizef, -CubeSizef,

			// LEFT
			-CubeSizef, -CubeSizef,  CubeSizef,
			-CubeSizef,  CubeSizef,  CubeSizef,
			-CubeSizef, -CubeSizef, -CubeSizef,
			-CubeSizef,  CubeSizef, -CubeSizef,

			// RIGHT
			CubeSizef, -CubeSizef, -CubeSizef,
			CubeSizef,  CubeSizef, -CubeSizef,
			CubeSizef, -CubeSizef,  CubeSizef,
			CubeSizef,  CubeSizef,  CubeSizef,

			// TOP
			-CubeSizef,  CubeSizef,  CubeSizef,
			 CubeSizef,  CubeSizef,  CubeSizef,
			-CubeSizef,  CubeSizef, -CubeSizef,
			 CubeSizef,  CubeSizef, -CubeSizef,

			// BOTTOM
			-CubeSizef, -CubeSizef,  CubeSizef,
			-CubeSizef, -CubeSizef, -CubeSizef,
			 CubeSizef, -CubeSizef,  CubeSizef,
			 CubeSizef, -CubeSizef, -CubeSizef,
							};

	
	private float texture[]={
							
			// FRONT
			0.0f, 0.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,

			// BACK
			1.0f, 0.0f,
			1.0f, 1.0f,
			0.0f, 0.0f,
			0.0f, 1.0f,

			// LEFT
			1.0f, 0.0f,
			1.0f, 1.0f,
			0.0f, 0.0f,
			0.0f, 1.0f,

			// RIGHT
			1.0f, 0.0f,
			1.0f, 1.0f,
			0.0f, 0.0f,
			0.0f, 1.0f,

			// TOP
			0.0f, 0.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f,

			// BOTTOM
			1.0f, 0.0f,
			1.0f, 1.0f,
			0.0f, 0.0f,
			0.0f, 1.0f,

			// MIDDLE
			0.0f, 0.0f,
			1.0f, 0.0f,
			0.0f, 1.0f,
			1.0f, 1.0f
							};

/** 
	 * The initial normals for the lighting calculations 
	 * 
	 * The normals are not necessarily correct from a 
	 * real world perspective, as I am too lazy to write
	 * these all on my own. But you get the idea and see
	 * what I mean if you run the demo.
	 */	
	private float normals[] = {
						// Normals are copy pased :  *** To be tested ***
						0.0f, 0.0f, 1.0f, 						
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f, 
						
						0.0f, 0.0f, 1.0f, 
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f,
						
						0.0f, 0.0f, 1.0f, 
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f,
						
						0.0f, 0.0f, 1.0f, 
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f,
						
						0.0f, 0.0f, 1.0f, 
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f,
						
						0.0f, 0.0f, 1.0f, 
						0.0f, 0.0f, -1.0f, 
						0.0f, 1.0f, 0.0f, 
						0.0f, -1.0f, 0.0f,
											};
	
	public int speed=0;

	public int move=5;

	/** Constructor function to setup the load and initialize the model class of rolling dice
	 * @param gl opengl interface object
	 * @param context activity context necessary to load and initialize textures from resources.
	 * */
	public  Cube(GL10 gl, Context context)
	{
		ByteBuffer bytBuf= ByteBuffer.allocateDirect(vertices.length*4);
		bytBuf.order(ByteOrder.nativeOrder());
		vertexBuffer= bytBuf.asFloatBuffer();
		vertexBuffer.put(vertices);
		vertexBuffer.position(0);
		
		bytBuf= ByteBuffer.allocateDirect(texture.length*4);
		bytBuf.order(ByteOrder.nativeOrder());
		textureBuffer= bytBuf.asFloatBuffer();
		textureBuffer.put(texture);
		textureBuffer.position(0);
		
		bytBuf = ByteBuffer.allocateDirect(normals.length * 4);
		bytBuf.order(ByteOrder.nativeOrder());
		normalBuffer = bytBuf.asFloatBuffer();
		normalBuffer.put(normals);
		normalBuffer.position(0);

			
		/*indexBuffer= ByteBuffer.allocateDirect(indice.length);
		indexBuffer.put(indice);
		indexBuffer.position(0);
		*/
		
		loadGLTexture(gl, context);
	}
	
	/**
	 * This function lets you draw the cube with the provided opengl interface object.
	 * @param gl opengl interface object.
	 * @param filter an optional filter to bind the texture according to the set texture filter.
	 */
	
	public void draw(GL10 gl, int filter)
	{
		//Bind the texture according to the set texture filter
	//	gl.glBindTexture(GL10.GL_TEXTURE_2D, textures[filter]);
	   	gl.glPushMatrix();
		//Enable the vertex, texture and normal state
		gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
		gl.glEnableClientState(GL10.GL_TEXTURE_COORD_ARRAY);
		gl.glEnableClientState(GL10.GL_NORMAL_ARRAY);

		//Set the face rotation
		gl.glFrontFace(GL10.GL_CCW);
		
		//Point to our buffers
		gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffer);
		gl.glTexCoordPointer(2, GL10.GL_FLOAT, 0, textureBuffer);
		gl.glNormalPointer(GL10.GL_FLOAT, 0, normalBuffer);
		
		for (int i = 0; i < 6; ++i) 
		{
           gl.glBindTexture(GL10.GL_TEXTURE_2D, textures[i]);
			//gl.glColor4f(1f, 1.0f, 0.0f, 0.6f);
			
            gl.glDrawArrays(GL10.GL_TRIANGLE_STRIP, i * 4, 4);
        }
		// draw triangle based on index buffer values
		//gl.glDrawElements(GL10.GL_TRIANGLES, indice.length, GL10.GL_UNSIGNED_BYTE, indexBuffer);
	    gl.glPopMatrix();
		// disable vertex, normal and texture
		gl.glDisable(GL10.GL_VERTEX_ARRAY);
		gl.glDisable(GL10.GL_NORMAL_ARRAY);
		gl.glDisable(GL10.GL_TEXTURE_COORD_ARRAY);
		
	
	}
	
	/**
	 * This function will load and bind the textures with six faces of the cube.
	 * @param gl
	 * @param context
	 */
	
	public void loadGLTexture(GL10 gl, Context context)
	{//context.getResources().openRawResource(Global.dimensions(Global.SIDE_TOP));
		

		gl.glGenTextures(textures.length, textures, 0);
		
		for (int i = 0; i < textures.length; ++i) {
			// mTextureID = textures[i];
			gl.glBindTexture(GL10.GL_TEXTURE_2D, textures[i]);
			gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MAG_FILTER, GL10.GL_NEAREST);
			gl.glTexParameterf(GL10.GL_TEXTURE_2D, GL10.GL_TEXTURE_MIN_FILTER, GL10.GL_NEAREST);
			InputStream is = null; 
			switch (i) {
				case 0:
					is=context.getResources().openRawResource(R.drawable.dice1); 
					break;
				case 1:
					is=context.getResources().openRawResource(R.drawable.dice6);
					break;
				case 2:
					is=context.getResources().openRawResource(R.drawable.dice3);
					break;
				case 3:
					is=context.getResources().openRawResource(R.drawable.dice4);
					break;
				case 4:
					is=context.getResources().openRawResource(R.drawable.dice5);
					break;
				case 5:
					is=context.getResources().openRawResource(R.drawable.dice2);
					break;
					}
			
			 Bitmap bitmap ;
			  try {
	              bitmap = BitmapFactory.decodeStream(is);
	          } finally {
	              try {
	                  is.close();
	              } catch(IOException e) {
	                  Log.i("Context Awareness", "Cube bitmap failed with exception");
	              }
	          }
	          GLUtils.texImage2D(GL10.GL_TEXTURE_2D, 0, bitmap, 0);
	         bitmap.recycle();
          }      	  
	
		}

}
