package com.cityu.blind_navigation.utils;

public class stringManipulate {

    public static String remove(String str, String substr) {
        if (str == null) {
            return null;
        }
        str = str.toLowerCase();
        int index = str.indexOf(substr);
        if (index == -1) {
            return str;
        }
        return str.substring(index + substr.length(), str.length());
    }
}
