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
    public void flopperDump()  {
        while(flopper.getPosition() < 0.58){
            flopper.setPosition(flopper.getPosition()+0.0005);

        }
    }


    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void flopperHold(){flopper.setPosition(0.5);}


}
