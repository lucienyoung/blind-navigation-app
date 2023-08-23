package com.cityu.blind_navigation;

import android.app.Application;

import com.jilk.ros.rosbridge.ROSBridgeClient;

/**
 * Created by Lucien on 23-05-31.
 */

public class RCApplication extends Application {
    ROSBridgeClient client;

    @Override
    public void onCreate(){
        super.onCreate();
    }

    @Override
    public void onTerminate(){
        if (client != null){
            client.disconnect();
        }
        super.onTerminate();
    }

    public ROSBridgeClient getRosClient(){
        return client;
    }

    public void setRosClient(ROSBridgeClient client) {
        this.client = client;
    }
}
