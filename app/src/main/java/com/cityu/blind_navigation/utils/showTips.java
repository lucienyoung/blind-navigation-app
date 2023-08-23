package com.cityu.blind_navigation.utils;

import android.app.Activity;
import android.widget.Toast;

public class showTips {

    public static void showTip(final Activity activity, final String tip) {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(activity, tip, Toast.LENGTH_SHORT).show();
            }
        });
    }
}
