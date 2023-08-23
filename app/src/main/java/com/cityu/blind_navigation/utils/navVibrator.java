package com.cityu.blind_navigation.utils;

import android.app.Activity;
import android.app.Service;
import android.os.VibrationEffect;
import android.os.Vibrator;

public class navVibrator {
    //vibrate for input milliseconds
    public static void vibrate(final Activity activity, long milliseconds){
        Vibrator mVibrator = (Vibrator) activity.getSystemService(Service.VIBRATOR_SERVICE);
        mVibrator.vibrate(VibrationEffect.createOneShot(milliseconds,VibrationEffect.DEFAULT_AMPLITUDE));
    }

    //vibrate for input milliseconds and amplitude
    public static void vibrate(final Activity activity, long milliseconds, int amplitude){
        Vibrator mVibrator = (Vibrator) activity.getSystemService(Service.VIBRATOR_SERVICE);
        //mVibrator.vibrate(milliseconds);
        mVibrator.vibrate(VibrationEffect.createOneShot(milliseconds, amplitude));
    }

    //set vibration pattern only
    public static void vibrate(final Activity activity, long[] pattern, int repeat){
        Vibrator mVibrator = (Vibrator) activity.getSystemService(Service.VIBRATOR_SERVICE);
        mVibrator.vibrate(VibrationEffect.createWaveform(pattern,repeat));
    }

    //set vibration pattern and amplitude
    public static void vibrate(final Activity activity, long[] pattern, int[] amplitude, int repeat){
        Vibrator mVibrator = (Vibrator) activity.getSystemService(Service.VIBRATOR_SERVICE);
        mVibrator.vibrate(VibrationEffect.createWaveform(pattern,amplitude,repeat));
    }

    //cancel vibration mode
    public static void vibrateCancel(final Activity activity){
        Vibrator mVibrator = (Vibrator) activity.getSystemService(Service.VIBRATOR_SERVICE);
        mVibrator.cancel();
    }
}
