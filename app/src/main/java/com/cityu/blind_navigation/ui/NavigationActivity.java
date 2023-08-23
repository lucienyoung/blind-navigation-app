package com.cityu.blind_navigation.ui;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.content.ContentValues;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.text.Editable;
import android.text.TextUtils;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import com.cityu.blind_navigation.R;
import com.cityu.blind_navigation.RCApplication;
import com.cityu.blind_navigation.database.goalDBHelper;
import com.cityu.blind_navigation.entity.PublishEvent;
import com.cityu.blind_navigation.entity.navGoal;
import com.cityu.blind_navigation.utils.navVibrator;
import com.cityu.blind_navigation.utils.parseTopic;
import com.cityu.blind_navigation.utils.showTips;
import com.cityu.blind_navigation.utils.stringManipulate;
import com.jilk.ros.rosbridge.ROSBridgeClient;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import de.greenrobot.event.EventBus;

public class NavigationActivity extends AppCompatActivity implements View.OnClickListener, TextToSpeech.OnInitListener {

    private static final String TAG = "NavigationActivity";

    //global client parameter, initialized in MainActivity, stored in RCApplication
    ROSBridgeClient client;

    //related to navigate button, whether or not subscribe topic /cmd_vel from ROS
    private boolean is_subscribe = false;

    private boolean is_record = false;

    private boolean is_inquire = false;

    //linear X and angular Z received from topic /cmd_vel, parsed in /utils/parseTopic/parseNavCmdTopic.class
    public static double x_l, z_a = 0.0;

    private String speech_input;

    private TextToSpeech mTts;

    private SpeechRecognizer speechRecognizer;

    private EditText et_record;

    private TextView tv_hint;

    private Button btn_nav;
    private Button btn_record;
    private Button btn_goal;
    private Button btn_inquire;
    private Button btn_interact;
    private Button btn_database;

    //SQLite helper
    private goalDBHelper mHelper;
    private SQLiteDatabase mWDB;
    private SQLiteDatabase mRDB;

    private navGoal goal;

    /* database related parameter */
    // name of point of interest, input via edit text
    private String goal_name;
    // from which coordinate the poses are obtained, default world frame
    public static String frame_id = "world";
    // position of pose (x, y, z)
    public static double goal_p_x;
    public static double goal_p_y;
    // orientation of pose, represented in the form of quaternion (x, y, z, w); only yaw angle is considered, so x, y are set to be 0.0
    public static double goal_o_z;
    public static double goal_o_w;
    private String goal_name_input;

    //only used for inquiry
    private double tmp_p_x;
    private double tmp_p_y;
    private double tmp_o_z;
    private double tmp_o_w;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_navigation);

        //dynamic audio input permission required
        requestPermission();

        //eventbus register, declare to subscribe message from ros-websocket
        EventBus.getDefault().register(this);

        //get client from RCApplication
        client = ((RCApplication)getApplication()).getRosClient();

        //define edit text
        et_record = (EditText) findViewById(R.id.et_record);

        tv_hint = (TextView) findViewById(R.id.tv_hint);

        //define button and set onclick listener
        btn_nav = (Button) findViewById(R.id.btn_nav);
        btn_nav.setOnClickListener(this);

        btn_record = (Button) findViewById(R.id.btn_record);
        btn_record.setOnClickListener(this);

        btn_goal = (Button) findViewById(R.id.btn_goal);
        btn_goal.setOnClickListener(this);

        btn_inquire = (Button) findViewById(R.id.btn_inquire);
        btn_inquire.setOnClickListener(this);

        btn_interact = (Button) findViewById(R.id.btn_interact);
        btn_interact.setOnClickListener(this);

        btn_database = (Button) findViewById(R.id.btn_database);
        btn_database.setOnClickListener(this);

        //text to speech instance
        mTts = new TextToSpeech(this,this);

        //speech recognizer instance and listener to start the service
        speechRecognizer = SpeechRecognizer.createSpeechRecognizer(NavigationActivity.this);
        speechRecognizer.setRecognitionListener(new navRecognitionListener());

        //mHelper = new goalDBHelper(this);

        //record button listener, whether or not to enable record
        et_record.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
                if (TextUtils.isEmpty(goal_name)) {
                    showTips.showTip(NavigationActivity.this,"Please input the name of your goal of interest to record");
                }
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                if (TextUtils.isEmpty(et_record.getText())) {
                    showTips.showTip(NavigationActivity.this, "If in record, the record name should not be empty");
//                    btn_record.setEnabled(false);
                } else {
                    btn_record.setEnabled(true);
                    btn_nav.setEnabled(false);
                    btn_goal.setEnabled(false);
                    btn_inquire.setEnabled(false);
                    btn_interact.setEnabled(false);
                    //goal_name = et_record.getText().toString().trim();
                }
            }

            @Override
            public void afterTextChanged(Editable s) {
                goal_name = s.toString();
                Log.d("editable", goal_name);

                if (s.toString().equals("reset")) {
                    mWDB.delete(goalDBHelper.getTableName(), "1=1", null);
                    showTips.showTip(NavigationActivity.this,"Database reset successfully");
                }
            }
        });

    }

    //database initialization
    @Override
    protected void onStart() {
        super.onStart();
        //get database helper instance
        mHelper = goalDBHelper.getInstance(this);
        mRDB = mHelper.getReadableDatabase();
        mWDB = mHelper.getWritableDatabase();
    }


    //main functions are called here
    @Override
    public void onClick(View v) {
        if(v.getId() == R.id.btn_nav){

            //whether or not to subscribe
            isCallCmdTopic();
            et_record.setHint("Navigation Mode");
            et_record.setEnabled(false);
            tv_hint.setText("");

        } else if (v.getId() == R.id.btn_record) {

            //record name and coordinate of point of interest for navigation, run when building the map
            isRecordOdometry();

        } else if (v.getId() == R.id.btn_goal) {

            Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
            intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
            intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.ENGLISH.toString());
            intent.putExtra(RecognizerIntent.EXTRA_PROMPT,"begin to input goal");
            speechRecognizer.startListening(intent);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //navigation goals setting
                    setGoal();
                }
            }).start();

        } else if (v.getId() == R.id.btn_inquire) {

            Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
            intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
            intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.ENGLISH.toString());
            intent.putExtra(RecognizerIntent.EXTRA_PROMPT,"begin to input inquiry");
            speechRecognizer.startListening(intent);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        if (is_subscribe) {
                            client.send("{\"op\":\"unsubscribe\",\"topic\":\"/cmd_vel\"}");
                        }

                        client.send("{\"op\":\"subscribe\",\"topic\":\"/pose_graph/global_camera_odom_2d\"}");
                        Thread.sleep(5000);

                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //inquire function support, only support the input "where i am" in version 1.0
                    setInquiry();
                }
            }).start();

        } else if (v.getId() == R.id.btn_interact) {

            // main navigation function, control signal processing
            processNavCmdTopic();
            btn_interact.setText("Interacting");

        } else if (v.getId() == R.id.btn_database) {

            //upload the global static parameter to database
            insertRecordToDatabase();

        }
    }

    //Callback, receive event data from eventbus, update static global parameter. See parseTopic.class.
    public void onEvent(final PublishEvent event) {
        if("/cmd_vel".equals(event.name)) {
            parseTopic.parseNavCmdTopic(event);
            return;
            // /pose_graph/global_camera_odom_2d
        } else if ("/pose_graph/global_camera_odom_2d".equals(event.name)) {
            parseTopic.parseOdometryRectTopic(event);
            return;
        }
        Log.d(TAG, event.msg);
    }

    private void isRecordOdometry() {

        if (is_record) {

            client.send("{\"op\":\"unsubscribe\",\"topic\":\"/pose_graph/global_camera_odom_2d\"}");
            btn_record.setText("Record");
            btn_database.setEnabled(false);

        } else {

            client.send("{\"op\":\"subscribe\",\"topic\":\"/pose_graph/global_camera_odom_2d\"}");
            btn_record.setText("Recording");
            btn_database.setEnabled(true);

        }
        is_record = !is_record;
    }

    private void isCallCmdTopic() {

        if(is_subscribe){
            client.send("{\"op\":\"unsubscribe\",\"topic\":\"/cmd_vel\"}");
            btn_nav.setText("Navigate");
            systemFeedback("Unsubscribe to navigation control signal");
        } else {
            client.send("{\"op\":\"subscribe\",\"topic\":\"/cmd_vel\"}");
            btn_nav.setText("Navigating");
            systemFeedback("Begin to subscribe navigation control signal. Keep receiving navigation command");
        }
        is_subscribe = !is_subscribe;
    }

    private void setGoal() {

        speech_input = navRecognitionListener.speech_result;
        Log.d("set_goal",speech_input);
        btn_interact.setText(speech_input);

        if (speech_input.toLowerCase().contains("go to")){

            goal_name_input = stringManipulate.remove(speech_input, "go to the").trim();

            //pick up 3DOF coordinate related to goal name input, retrieve the database
            List<navGoal> list = new ArrayList<>();
            Cursor cursor = mRDB.query(goalDBHelper.getTableName(), null, "goal_name=?",new String[]{goal_name_input}, null, null, null);

            //whether or not the record has been retrieved
            if (cursor.getCount() == 0) {
                systemFeedback("no match result about " + goal_name_input);
                return;
            }

            //retrieve, put the result into the object list
            while (cursor.moveToNext()) {
                navGoal goal = new navGoal();
                goal.id = cursor.getInt(0);
                goal.name = cursor.getString(1);
                goal.frame_id = cursor.getString(2);
                goal.position_x = cursor.getFloat(3);
                goal.position_y = cursor.getFloat(4);
                goal.orientation_z = cursor.getFloat(5);
                goal.orientation_w = cursor.getFloat(6);
                list.add(goal);
            }
            cursor.close();

            //if there are more than one matches, only pick the last one
            for (navGoal g : list) {
                Log.d("database", g.toString());
                frame_id = g.frame_id;
                goal_p_x = g.position_x;
                goal_p_y = g.position_y;
                goal_o_z = g.orientation_z;
                goal_o_w = g.orientation_w;
            }

            //send goal from android client to ros-bridge server, in form of geometry pose stamped
            String advertise = "{\"op\":\"advertise\",\"topic\":\"/move_base_simple/goal\",\"type\":\"geometry_msgs/PoseStamped\"}";
            client.send(advertise);

            String publish = "{\"op\":\"publish\",\"topic\":\"/move_base_simple/goal\",\"msg\":{\"header\":{\"frame_id\":\""+ frame_id + "\"},\"pose\":{\"position\":{\"x\":" + goal_p_x + ",\"y\":" +
                    goal_p_y + ",\"z\":0},\"orientation\":{\"x\":0,\"y\":0,\"z\":" + goal_o_z + ",\"w\":" + goal_o_w + "}}}}";
            client.send(publish);

            String unadvertise = "{\"op\":\"unadvertise\",\"topic\":\"/move_base_simple/goal\"}";
            client.send(unadvertise);

            systemFeedback("You will navigate to " + goal_name_input + ". " + "Goal setting finished. Begin to navigate");


        } else {

            systemFeedback("Warning: illegal navigation goal setting");
            btn_interact.setText("Warning: illegal navigation goal setting");
        }

    }

    private void setInquiry() {

        speech_input = navRecognitionListener.speech_result;
        Log.d("set_inquiry", speech_input);
        btn_interact.setText(speech_input);

        if (speech_input.toLowerCase().contains("where i am")){

//            if (is_subscribe) {
//                client.send("{\"op\":\"unsubscribe\",\"topic\":\"/cmd_vel\"}");
//            }
//
//            client.send("{\"op\":\"subscribe\",\"topic\":\"/pose_graph/global_camera_odom_2d\"}");

            List<navGoal> list = new ArrayList<>();
            Cursor cursor = mRDB.query(goalDBHelper.getTableName(), null, "goal_name=?",new String[]{goal_name_input}, null, null, null);

            //whether or not the record has been retrieved
            if (cursor.getCount() == 0) {
                systemFeedback("no match result about " + goal_name_input);
                return;
            }

            //retrieve, put the result into the object list
            while (cursor.moveToNext()) {
                navGoal goal = new navGoal();
                goal.id = cursor.getInt(0);
                goal.name = cursor.getString(1);
                goal.frame_id = cursor.getString(2);
                goal.position_x = cursor.getFloat(3);
                goal.position_y = cursor.getFloat(4);
                goal.orientation_z = cursor.getFloat(5);
                goal.orientation_w = cursor.getFloat(6);
                list.add(goal);
            }
            cursor.close();

            for (navGoal g : list) {
                Log.d("database", g.toString());
                tmp_p_x = g.position_x;
                tmp_p_y = g.position_y;
                tmp_o_z = g.orientation_z;
                tmp_o_w = g.orientation_w;
            }

            //compute distance and orientation from current point to the goal, only reserve 2 digit behind the decimal point
            double distance = Math.sqrt(Math.pow(goal_p_x-tmp_p_x, 2) + Math.pow(goal_p_y - tmp_p_y, 2));
            distance = (double) Math.round((double) (distance * 10)) / 10;
            double delta_q_real = goal_o_w * tmp_o_w + goal_o_z * tmp_o_z;
            double delta_q_k = goal_o_w * tmp_o_z - goal_o_z * tmp_o_w;
            double delta_q_norm = Math.sqrt(Math.pow(delta_q_real, 2) + Math.pow(delta_q_k, 2));
            double delta_q_w_norm = delta_q_real / delta_q_norm;
            double orientation = 2 * Math.acos(delta_q_w_norm) * 180 / Math.PI;
            orientation = (double) Math.round((double) (orientation * 10)) / 10;

            if (orientation > 180) {
                orientation = orientation - 360;
            }

            if (distance < 0.25){
                systemFeedback("Your destination is arrived, the navigation is over.");
                navVibrator.vibrate(NavigationActivity.this, 2000);
            } else {
                systemFeedback("There are still " + distance + " meters away from your goal " + goal_name_input +
                        ". The orientation is around " + orientation + " degrees");
            }

        } else {

            systemFeedback("sorry, I don't know what you mean");
        }

        client.send("{\"op\":\"unsubscribe\",\"topic\":\"/pose_graph/global_camera_odom_2d\"}");
        client.send("{\"op\":\"subscribe\",\"topic\":\"/cmd_vel\"}");
        if (!is_subscribe) {
            is_subscribe = !is_subscribe;
        }

    }

    //navigation mode
    private void processNavCmdTopic(){
        int amp = 0;

        if (Math.abs(z_a) < 0.6) {
            amp = (int) Math.abs(z_a) * 255 + 102;

            if (z_a == 0){
                amp = 0;
            }

        } else {
            amp = 255;
        }

        if(x_l > 0.1) {
            navVibrator.vibrate(NavigationActivity.this, new long[]{300,300,300,300},
                    new int[]{amp,amp,amp,amp},-1);
            if (z_a < -0.25) {
                navPrompt("turn right and go");
            } else if (z_a > 0.25) {
                navPrompt("turn left and go");
            } else {
                navPrompt("go straight");
            }

        } else {

            if (Math.abs(z_a) == 1.0 && Math.abs(x_l) == 0){
                systemFeedback("Recovery mode, please slowly move around, be cautious");
            } else if (Math.abs(z_a) == 0 && Math.abs(x_l) == 0) {
                systemFeedback("Your destination may be arrived, please check out the inquiry function for reassurance, otherwise, please slowly move around, be cautious");
                navVibrator.vibrate(NavigationActivity.this, 2000);
            } else if (z_a > 0) {
                navPrompt("turn left");
                navVibrator.vibrate(NavigationActivity.this, new long[]{100,200,300,400},
                        new int[]{amp,amp,amp,amp},-1);
            } else if (z_a < 0) {
                navPrompt("turn right");
                navVibrator.vibrate(NavigationActivity.this, new long[]{100,200,300,400},
                        new int[]{amp,amp,amp,amp},-1);
            }
        }

    }

    private void insertRecordToDatabase() {

        ContentValues values = new ContentValues();
        values.put("goal_name", goal_name);
        values.put("frame_id", frame_id);
        values.put("position_x", goal_p_x);
        values.put("position_y", goal_p_y);
        values.put("orientation_z", goal_o_z);
        values.put("orientation_w", goal_o_w);

        if (mWDB.insert(goalDBHelper.getTableName(),null, values ) > 0) {
            showTips.showTip(NavigationActivity.this, "Navigation goals are recorded successfully");
            et_record.setText("");
            btn_record.setText("Recorded");
            values.clear();
        }

    }

    private void navPrompt(String orientation){
        mTts.speak(orientation, TextToSpeech.QUEUE_FLUSH, null, "navMessage");
    }

    private void systemFeedback(String info){
        mTts.speak(info, TextToSpeech.QUEUE_FLUSH, null, "systemFeedback");
    }

    @Override
    protected void onStop(){
        super.onStop();
        client.send("{\"op\":\"unsubscribe\",\"topic\":\"/cmd_vel\"}");
        btn_nav.setText("Navigate");
        speechRecognizer.stopListening();

        if (mRDB != null && mRDB.isOpen()) {
            mRDB.close();
            mRDB = null;
        }

        if (mWDB != null && mWDB.isOpen()) {
            mWDB.close();
            mWDB = null;
        }

    }

    @Override
    protected void onDestroy(){
        super.onDestroy();
        EventBus.getDefault().unregister(this);

        if (mTts != null) {
            mTts.stop();
            mTts.shutdown();
        }

        if(speechRecognizer != null){
            speechRecognizer.cancel();
            speechRecognizer.destroy();
        }
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
                btn_nav.setEnabled(true);
                btn_goal.setEnabled(true);
                btn_inquire.setEnabled(true);
                btn_interact.setEnabled(true);
            }

            if (mTts != null){
                mTts.setPitch(1.0f);
                mTts.setSpeechRate(1.5f);
            }
        } else {
            Log.e("speechInfo","fail to initialize");
        }
    }

    public class navRecognitionListener implements RecognitionListener {

        private static final String TAG = "navRecognitionListener";

        public static String speech_result = "";

        @Override
        public void onReadyForSpeech(Bundle bundle) {
            Log.d(TAG, "onReadyForSpeech Start");
            Log.d(TAG, "onReadyForSpeech End");
        }

        @Override
        public void onBeginningOfSpeech() {
            Log.d(TAG, "onBeginningOfSpeech Start");
            Log.d(TAG, "onBeginningOfSpeech End");
        }

        @Override
        public void onRmsChanged(float v) {
            Log.d(TAG, "onRmsChanged Start");
            Log.d(TAG, "onRmsChanged End");
        }

        @Override
        public void onBufferReceived(byte[] bytes) {
            Log.d(TAG, "onBufferReceived Start");
            Log.d(TAG, "onBufferReceived End");
        }

        @Override
        public void onEndOfSpeech() {
            Log.d(TAG, "onEndOfSpeech Start");
            Log.d(TAG, "onEndOfSpeech End");

        }

        //error processing
        @Override
        public void onError(int error) {
            Log.d(TAG, "onError Start");

            switch (error) {
                case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                    btn_interact.setText("Error: network timeout");
                    systemFeedback("Error: network timeout");
                    break;
                case SpeechRecognizer.ERROR_NETWORK:
                    btn_interact.setText("Error: network unauthorized");
                    systemFeedback("Error: network unauthorized");
                    break;
                case SpeechRecognizer.ERROR_AUDIO:
                    btn_interact.setText("Error: audio fails");
                    systemFeedback("Error: audio fails");
                    break;
                case SpeechRecognizer.ERROR_CLIENT:
                    btn_interact.setText("Error: client error");
                    systemFeedback("Error: client error");
                    break;
                case SpeechRecognizer.ERROR_SERVER:
                    btn_interact.setText("Error: server error");
                    systemFeedback("Error: server error");
                    break;
                case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                    btn_interact.setText("Error: speech time out");
                    systemFeedback("Error: speech time out");
                    break;
                case SpeechRecognizer.ERROR_NO_MATCH:
                    btn_interact.setText("Error: no match result");
                    systemFeedback("Error: no match result");
                    break;
                case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                    btn_interact.setText("Error: recognizer busy");
                    systemFeedback("Error: recognizer busy");
                    break;
                case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                    btn_interact.setText("Error: APP permission required");
                    systemFeedback("Error: APP permission required");
                    break;
                default:
                    break;

            }
            Log.d(TAG, "onError End");
        }

        //get the speech recognition result
        @Override
        public void onResults(Bundle results) {
            Log.d(TAG, "onResults Start");
            String key = SpeechRecognizer.RESULTS_RECOGNITION;
            ArrayList<String> mResult = results.getStringArrayList(key);

//            String[] result = new String[0];
//            if (mResult != null) {
//                result = new String[mResult.size()];
//            }
//            if (mResult != null) {
//                mResult.toArray(result);
//            }

            speech_result = "";
            for(int i = 0; i < mResult.size(); i++){
                speech_result += mResult.get(i);
            }

            Log.d(TAG, "Recognize Result:" + speech_result);
            //btn_interact.setText(speech_result); //resetText(result[0]);
            Log.d(TAG, "onResults End");
        }

        //partial result, not used
        @Override
        public void onPartialResults(Bundle bundle) {
            Log.d(TAG, "onPartialResults Start");
            Log.d(TAG, "onPartialResults End");

        }

        @Override
        public void onEvent(int i, Bundle bundle) {
            Log.d(TAG, "onEvent Start");
            Log.d(TAG, "onEvent End");
        }

    }

    //dynamic permission request, required in API 6.0 or above, otherwise it occurs network error
    private void requestPermission() {
        Log.d(TAG, "requestPermission");
        int PERMISSIONS_REQUEST_RECORD_AUDIO = 0;
        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.RECORD_AUDIO, Manifest.permission.INTERNET}, PERMISSIONS_REQUEST_RECORD_AUDIO);
    }

}