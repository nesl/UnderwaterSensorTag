package com.invensense.casdk.client;

import java.io.IOException;
import java.util.UUID;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;
import android.util.Log;

/**
 * An Asynchronous task extension that sets up the initial connection 
 * of the bluetooth for the wearable sdk. For more information refer 
 * to the android documentation.
 * @author Invensense
 *
 */
public class ConnectBluetooth extends AsyncTask<BluetoothDevice, Integer, BluetoothSocket> {

   
	/** UUID of the wearable sdk */
    protected static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
    
    @Override
    protected BluetoothSocket doInBackground(BluetoothDevice... params) {
        BluetoothSocket mBluetoothSocket = null;
        try {
            mBluetoothSocket = params[0].createRfcommSocketToServiceRecord(MY_UUID);
         } catch (IOException e) {
            Log.i("MainActivity", "failed to create socket");
             return null;
         }
         Log.i("MainActivity", "got socket");
         try {
             mBluetoothSocket.connect();
         } catch (IOException e) {
             Log.i("MainActivity", "failed to connect socket");
             try {
                 mBluetoothSocket.close();
             } catch (IOException e2) {
                 
             }

             return null;
         }

        return mBluetoothSocket;
    }


}
