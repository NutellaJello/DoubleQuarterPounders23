
package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides {
    /* Declare OpMode members. */
    public DcMotorEx slides = null;

    public Slides(HardwareMap hardwareMap) {
        this.slides = hardwareMap.get(DcMotorEx.class,"slides"); //0
    }

    public void slidesUp(){
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(slides.getCurrentPosition() + 50);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.5);
    }

    public void slidesDown(){
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setTargetPosition(slides.getCurrentPosition() - 50);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.5);
    }


}
