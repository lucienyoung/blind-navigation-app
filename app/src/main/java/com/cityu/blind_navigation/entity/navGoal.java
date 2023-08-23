package com.cityu.blind_navigation.entity;

public class navGoal {

    public int id;
    public String name;
    public String frame_id;
    public double position_x;
    public double position_y;
    public double orientation_z;
    public double orientation_w;

    public navGoal() {}

    public navGoal(String name, String frame_id, double position_x, double position_y, double orientation_z, double orientation_w) {
        this.name = name;
        this.frame_id = frame_id;
        this.position_x = position_x;
        this.position_y = position_y;
        this.orientation_z = orientation_z;
        this.orientation_w = orientation_w;
    }
}
