package com.cityu.blind_navigation.ui;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.cityu.blind_navigation.R;
import com.cityu.blind_navigation.RCApplication;
import com.cityu.blind_navigation.utils.showTips;
import com.jilk.ros.ROSClient;
import com.jilk.ros.rosbridge.ROSBridgeClient;

import java.util.Locale;

public class MainActivity extends AppCompatActivity implements TextToSpeech.OnInitListener{

    private static final String TAG = "MainActivity";

    private EditText etIP;
    private EditText etPort;

    private Button btn_connect;

    private String ip;
    private String port;

    private SharedPreferences preferences;

    private TextToSpeech mTts;

    ROSBridgeClient client;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mTts = new TextToSpeech(this,this);

        etIP = (EditText) findViewById(R.id.et_ip);
        etPort = (EditText) findViewById(R.id.et_port);

        btn_connect = (Button) findViewById(R.id.btn_connect);

        btn_connect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ip = etIP.getText().toString();
                port = etPort.getText().toString();
                Log.d(TAG,ip);
                Log.d(TAG,port);
                SharedPreferences.Editor editor = getSharedPreferences("network",MODE_PRIVATE).edit();
                editor.putString("ip", ip);
                editor.putString("port", port);
                editor.apply();
                connect(ip,port);
            }
        });

        preferences = getSharedPreferences("network", MODE_PRIVATE);
        reload();
    }

    private void reload() {
        ip = preferences.getString("ip",null);
        if (ip != null) {
            etIP.setText(ip);
        }

        port = preferences.getString("port", null);
        if (port != null) {
            etPort.setText(port);
        }
    }

    private void connect(String ip, String port) {
        client = new ROSBridgeClient("ws://" + ip + ":" + port);
        //client = new ROSBridgeClient("ws://192.168.1.100:9090");
        Log.d(TAG,"client successfully create");
        boolean is_connect = client.connect(new ROSClient.ConnectionStatusListener() {
            @Override
            public void onConnect() {
                //client.setDebug(true);
                ((RCApplication)getApplication()).setRosClient(client);
                showTips.showTip(MainActivity.this, "Connect ROS success");
                Log.d(TAG,"Connect ROS success");
                systemFeedback("Connected to navigation server successfully. Begin to navigate");
                Intent intent = new Intent(MainActivity.this, NavigationActivity.class);
                startActivity(intent);
            }

            @Override
            public void onDisconnect(boolean normal, String reason, int code) {
                showTips.showTip(MainActivity.this, "ROS disconnect");
                Log.d(TAG,"ROS disconnect");
                systemFeedback("Disconnected to navigation server. Please check out your ip address and port and try again");
            }

            @Override
            public void onError(Exception ex) {
                ex.printStackTrace();
                showTips.showTip(MainActivity.this, "ROS communication error");
                Log.d(TAG,"ROS communication error");
                systemFeedback("Unable to connect to navigation server. Connection error occurs");
            }
        });
    }

    private void systemFeedback(String info){
        mTts.speak(info, TextToSpeech.QUEUE_FLUSH, null, "connectFeedback");
    }

    // realize TextToSpeech.OnInitListener.
    @Override
    public void onInit(int status) {
        if (status == TextToSpeech.SUCCESS){

            int result = mTts.setLanguage(Locale.ENGLISH);

            if(result == TextToSpeech.LANG_MISSING_DATA ||
                    result == TextToSpeech.LANG_NOT_SUPPORTED){
                Log.e("speechInfo","language data missing or not supported");
            } else {
                btn_connect.setEnabled(true);
            }

            if (mTts != null){
                mTts.setPitch(1.0f);
                mTts.setSpeechRate(1.5f);
            }
        } else {
            Log.e("speechInfo","fail to initialize");
        }
    }

}