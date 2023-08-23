package com.cityu.blind_navigation.database;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.widget.Toast;

import com.cityu.blind_navigation.entity.navGoal;

import java.util.ArrayList;
import java.util.List;

public class goalDBHelper extends SQLiteOpenHelper {

    private static final String DB_NAME = "goal_record.db";
    private static final String TABLE_NAME = "goal_info";
    private static final int DB_VERSION = 1;
    private static goalDBHelper mHelper;

    private Context mContext;

    private goalDBHelper(Context context) {
        super(context, DB_NAME, null, DB_VERSION);
        mContext = context;
    }


    //single instance mode
    public static goalDBHelper getInstance(Context context) {
        if (mHelper == null) {
            mHelper = new goalDBHelper(context);
        }
        return mHelper;
    }

    public static String getTableName() {
        return TABLE_NAME;
    }

    //create database, exert table create sentences
    @Override
    public void onCreate(SQLiteDatabase db) {
        String CREATE_GOAL = "CREATE TABLE IF NOT EXISTS "+ TABLE_NAME + " (" +
                "_id INTEGER PRIMARY KEY AUTOINCREMENT," +
                " goal_name VARCHAR," +
                " frame_id VARCHAR," +
                " position_x FLOAT," +
                " position_y FLOAT," +
                " orientation_z FLOAT," +
                " orientation_w FLOAT);";
        db.execSQL(CREATE_GOAL);
        Toast.makeText(mContext, "Create database successfully", Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {

    }
}
