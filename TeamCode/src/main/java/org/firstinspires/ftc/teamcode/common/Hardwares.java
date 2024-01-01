package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardwares {
    /* Declare OpMode members. */

    public DcMotorEx         FrontLeft = null;
    public DcMotorEx         FrontRight = null;
    public DcMotorEx         BackLeft = null;
    public DcMotorEx         BackRight = null;
    public DcMotorEx[]       motorsDrive;
    public DcMotor slides = null;

    public Servo airplane = null;
    public Servo claw = null;
    public Servo flopper = null;
    public Servo rightarm = null;
    public Servo leftarm = null;

    public IMU imu         = null;      // Control/Expansion Hub IMU

    public void init(HardwareMap hardwareMap) {

        FrontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft"); //0
        BackLeft = hardwareMap.get(DcMotorEx.class,"BackLeft"); //1
        FrontRight = hardwareMap.get(DcMotorEx.class,"FrontRight"); //2
        BackRight = hardwareMap.get(DcMotorEx.class,"BackRight"); //3
        slides = hardwareMap.dcMotor.get("slides"); //3
        airplane = hardwareMap.servo.get("airplane");
        claw = hardwareMap.servo.get("claw");
        flopper = hardwareMap.servo.get("flopper");
        flopper.setDirection(Servo.Direction.REVERSE);
        leftarm = hardwareMap.servo.get("leftarm");
        rightarm = hardwareMap.servo.get("rightarm");
        rightarm.setDirection(Servo.Direction.REVERSE);
        motorsDrive = new DcMotorEx[] {FrontLeft, BackLeft, FrontRight, BackRight};

        // reversing our motors
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        airplane.setPosition(0.63); // locks airplane
    }

}
