package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flopper {
    /* Declare OpMode members. */
    public Servo flopper = null;

    public Flopper(HardwareMap hardwareMap) {

        flopper = hardwareMap.servo.get("flopper");
    }

    public void flopperRest() {
        flopper.setPosition(0.305);
    }
    public void flopperDump() {
        flopper.setPosition(0.7);
    }


}
