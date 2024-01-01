package org.firstinspires.ftc.teamcode.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="zigzag")
public class Zigzag extends LinearOpMode {
    private static Integer POSITION_FORWARD_ONE_BLOCK = -1092;
    private static Integer POSITION_RIGHT_ONE_BLOCK = 1250;
    private static Integer POSITION_LEFT_ONE_BLOCK = 1250;
    private static Integer TURN_LEFT = 1770;
    private static Integer TURN_RIGHT = -1770;
    private static double VELOCITY = 900;

    public DcMotorEx FrontLeft;
    public DcMotorEx BackLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackRight;
    public DcMotorEx[] motorsDrive;
    public DcMotorEx[] frontLBackR;
    public DcMotorEx[] frontRBackL;

    @Override
    public void runOpMode() {
        waitForStart();
        FrontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft"); //0
        BackLeft = hardwareMap.get(DcMotorEx.class,"BackLeft"); //1
        FrontRight = hardwareMap.get(DcMotorEx.class,"FrontRight"); //2
        BackRight = hardwareMap.get(DcMotorEx.class,"BackRight"); //3
        motorsDrive = new DcMotorEx[]{FrontLeft, FrontRight, BackLeft, BackRight};
        frontRBackL = new DcMotorEx[]{FrontRight, BackLeft};
        frontLBackR = new DcMotorEx[] {FrontLeft, BackRight};

        this.FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.BackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        while (opModeIsActive()) {
            moveForward(POSITION_FORWARD_ONE_BLOCK, VELOCITY);
            sleep(2000);
            moveLeft(POSITION_LEFT_ONE_BLOCK, VELOCITY);
            sleep(2000);
            turnLeft(TURN_LEFT, TURN_RIGHT, VELOCITY);
            moveForward(POSITION_FORWARD_ONE_BLOCK, VELOCITY);
        }

    }

    private void moveForward(int cycle, double velocity) {
        for (DcMotorEx motor : motorsDrive) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }

    private void moveLeft(int cycle, double velocity) {
        for (DcMotorEx motor : new DcMotorEx[] {FrontLeft, BackRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : new DcMotorEx[] {FrontRight, BackLeft}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }

    private void turnLeft(int cycle, int acycle, double velocity) {
        for (DcMotorEx motor : new DcMotorEx[] {FrontLeft, BackLeft}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : new DcMotorEx[] {FrontRight, BackRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(acycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }

    }

}
