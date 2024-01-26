
package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class Slides {
    /* Declare OpMode members. */
    public DcMotorEx slides = null;

    public Slides(HardwareMap hardwareMap) {
        this.slides = hardwareMap.get(DcMotorEx.class,"slides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void slidesUp(){

        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(Constants.SLIDE_HIGH_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.5);

        while(slides.isBusy())
        {
            sleep(10);
        }
    }

    public void slidesDown(){
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(Constants.SLIDE_LOW_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.5);

        while(slides.isBusy())
        {
            sleep(10);
        }
    }
    public void slidesHold(){
        //slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(Constants.SLIDE_MID_POSITION);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.5);

        while(slides.isBusy())
        {
            sleep(10);
        }
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}


