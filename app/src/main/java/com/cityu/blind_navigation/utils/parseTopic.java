package com.cityu.blind_navigation.utils;

import com.cityu.blind_navigation.entity.PublishEvent;
import com.cityu.blind_navigation.ui.NavigationActivity;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class parseTopic {

    public static void parseNavCmdTopic(PublishEvent event){
        try{
            JSONParser parser = new JSONParser();
            JSONObject jsonCmd = (JSONObject) parser.parse(event.msg);

            String linear = jsonCmd.get("linear").toString();
            String angular = jsonCmd.get("angular").toString();

            JSONObject jsonLinear = (JSONObject) parser.parse(linear);
            JSONObject jsonAngular = (JSONObject) parser.parse(angular);

            String linearX = jsonLinear.get("x").toString();
            String angularZ = jsonAngular.get("z").toString();

            NavigationActivity.x_l = Double.parseDouble(linearX);
            NavigationActivity.z_a = Double.parseDouble(angularZ);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void parseOdometryRectTopic(PublishEvent event) {
        try{
            JSONParser parser = new JSONParser();
            JSONObject jsonOdom = (JSONObject) parser.parse(event.msg);

            String header = jsonOdom.get("header").toString();
            JSONObject jsonHeader = (JSONObject) parser.parse(header);
            NavigationActivity.frame_id = jsonHeader.get("frame_id").toString();

            String pose = jsonOdom.get("pose").toString();
            JSONObject jsonPose = (JSONObject) parser.parse(pose);
            String poseWithoutCov = jsonPose.get("pose").toString();
            JSONObject jsonPoseWithoutCov = (JSONObject) parser.parse(poseWithoutCov);

            String position = jsonPoseWithoutCov.get("position").toString();
            JSONObject jsonPosition = (JSONObject) parser.parse(position);
            String positionX = jsonPosition.get("x").toString();
            String positionY = jsonPosition.get("y").toString();

            String orientation = jsonPoseWithoutCov.get("orientation").toString();
            JSONObject jsonOrientation = (JSONObject) parser.parse(orientation);
            String orientationZ = jsonOrientation.get("z").toString();
            String orientationW = jsonOrientation.get("w").toString();

            NavigationActivity.goal_p_x = Double.parseDouble(positionX);
            NavigationActivity.goal_p_y = Double.parseDouble(positionY);
            NavigationActivity.goal_o_z = Double.parseDouble(orientationZ);
            NavigationActivity.goal_o_w = Double.parseDouble(orientationW);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

