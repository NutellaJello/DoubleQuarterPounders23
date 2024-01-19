package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    /* Declare OpMode members. */
    public Servo rightarm = null;
    public Servo leftarm = null;


    public Arm(HardwareMap hardwareMap) {
        leftarm = hardwareMap.servo.get("leftarm");
        rightarm = hardwareMap.servo.get("rightarm");
        rightarm.setDirection(Servo.Direction.REVERSE);
    }

    public void armUp() {
        leftarm.setPosition(0.967);
        rightarm.setPosition(0.967);
    }

    public void armDown() {
        leftarm.setPosition(0.04);
        rightarm.setPosition(0.04);
    }
    public void initArm(){
        leftarm.setPosition(0.8);
        rightarm.setPosition(0.8);
    }

    public void armHover() {
        leftarm.setPosition(0.10);
        rightarm.setPosition(0.10);
    }

    public void putPixel() {
        leftarm.setPosition(0.15);
        rightarm.setPosition(0.15);
    }

}
