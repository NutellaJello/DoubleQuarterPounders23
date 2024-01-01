package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vehicle {

    public DcMotorEx FrontLeft = null;
    public DcMotorEx         FrontRight = null;
    public DcMotorEx         BackLeft = null;
    public DcMotorEx         BackRight = null;
    public IMU imu         = null;
    public DcMotorEx[]       motorsDrive;
    public Telemetry         telemetry;

    public void Vehicle(HardwareMap hardwareMap, Telemetry telemetry) {

        this.FrontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft"); //0
        this.BackLeft = hardwareMap.get(DcMotorEx.class,"BackLeft"); //1
        this.FrontRight = hardwareMap.get(DcMotorEx.class,"FrontRight"); //2
        this.BackRight = hardwareMap.get(DcMotorEx.class,"BackRight"); //3
        this.motorsDrive = new DcMotorEx[] {FrontLeft, BackLeft, FrontRight, BackRight};

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        this.telemetry = telemetry;
    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        
    }
    /*
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * Constants.COUNTS_PER_INCH);
//            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
//            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            flTarget = FrontLeft.getCurrentPosition() + moveCounts;
            frTarget = FrontRight.getCurrentPosition() + moveCounts;
            blTarget = BackLeft.getCurrentPosition() + moveCounts;
            brTarget = BackRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
//            leftDrive.setTargetPosition(leftTarget);
//            rightDrive.setTargetPosition(rightTarget);

            FrontLeft.setTargetPosition(flTarget);
            FrontRight.setTargetPosition(frTarget);
            BackLeft.setTargetPosition(blTarget);
            BackRight.setTargetPosition(brTarget);

//            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FrontLeft.isBusy() && BackRight.isBusy() && FrontRight.isBusy()&& BackLeft.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void moveForward() {

    }

    public void moveBackward() {

    }

    public void moveLeft() {

    }

    public void moveRight() {

    }

    public void turnLeft() {

    }


    public void moveOneFrontLeft(int cycle, double velocity) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setTargetPosition(cycle);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setVelocity(velocity);
    }

    public void moveOneFrontRight(int cycle, double velocity) {
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setTargetPosition(cycle);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setVelocity(velocity);
    }

    public void moveOneBackLeft(int cycle, double velocity) {
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setTargetPosition(cycle);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setVelocity(velocity);
    }

    public void moveBackRight(int cycle, double velocity) {
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setTargetPosition(cycle);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setVelocity(velocity);
    }

     */
}
