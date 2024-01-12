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
        leftarm.setPosition(0.95);
        rightarm.setPosition(0.95);
    }

    public void armDown() {
        leftarm.setPosition(0.023);
        rightarm.setPosition(0.023);
    }
    public void initArm(){
        leftarm.setPosition(0.8);
        rightarm.setPosition(0.8);
    }

}
