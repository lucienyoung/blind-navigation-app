package com.cityu.blind_navigation.entity;

import com.jilk.ros.rosbridge.operation.Operation;

/**EventBus event entity,describe ros server response info
 * Created by Lucien on 23-05-22.
 */

public class PublishEvent {
    public String msg;
    public String id;
    public String name;
    public String op;


    public PublishEvent(Operation operation, String name, String content) {
        if(operation != null) {
            id = operation.id;
            op = operation.op;
        }
        this.name = name;
        msg = content;
    }
}
