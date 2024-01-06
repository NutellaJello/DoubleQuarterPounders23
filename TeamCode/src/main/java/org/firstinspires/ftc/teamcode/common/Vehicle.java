package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.PIDControl;

public class Vehicle {

    public DcMotorEx         FrontLeft = null;
    public DcMotorEx         FrontRight = null;
    public DcMotorEx         BackLeft = null;
    public DcMotorEx         BackRight = null;
    public IMU imu         = null;
    public DcMotorEx[]       motorsDrive;
    public Telemetry         telemetry;

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    private int flTarget = 0;

    private int frTarget = 0;

    private int blTarget = 0;

    private int brTarget = 0;

    private double flspeed = 0;

    private double frspeed = 0;

    private double blspeed = 0;

    private double brspeed = 0;
    private PIDControl pidControl = new PIDControl();

    private double          headingError  = 0;

    public Vehicle(HardwareMap hardwareMap, Telemetry telemetry) {

        this.FrontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft"); //0
        this.BackLeft = hardwareMap.get(DcMotorEx.class,"BackLeft"); //1
        this.FrontRight = hardwareMap.get(DcMotorEx.class,"FrontRight"); //2
        this.BackRight = hardwareMap.get(DcMotorEx.class,"BackRight"); //3
        this.motorsDrive = new DcMotorEx[] {FrontLeft, BackLeft, FrontRight, BackRight};

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Ensure that the OpMode is still active

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
            while (FrontLeft.isBusy() && BackRight.isBusy() && FrontRight.isBusy()&& BackLeft.isBusy()) {

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, Constants.P_DRIVE_GAIN);

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

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      FrontLeft.getCurrentPosition(),
                    FrontRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", flspeed, frspeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double controlSig = pidControl.PIDValue(desiredHeading, getHeading());
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        //return Range.clip(headingError * proportionalGain, -1, 1);
        return Range.clip(controlSig, -1, 1);
    }

    public void turn(double angle, double speed) {
        wheelSetMode(2);
        Orientation angles = imu.getRobotOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initialAngle = angles.firstAngle;
        double motorPower;
        double minMotorPower = 0.3 ;
        double powerScaleFactor;
        double targetAngle;
        double currentAngle;
        double deltaAngle;
        double robotAngle = angles.firstAngle;
        double previousAngle = angles.firstAngle;

        targetAngle = initialAngle + angle;

        if (Math.abs(angle) < 8){
            minMotorPower = .2;
        }

        while (Math.abs(targetAngle - robotAngle)> .5)
        {

            angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = angles.firstAngle;

            //update speed dynamically to slow when approaching the target
            powerScaleFactor = Math.sqrt(Math.abs((targetAngle-robotAngle)/angle));
            if (powerScaleFactor > 1)
            {
                powerScaleFactor = 1;
            }
            motorPower = powerScaleFactor*speed;
            if (motorPower < minMotorPower)
            {
                motorPower = minMotorPower;
            }

            //determine which direction the robot should turn



            if ((targetAngle - robotAngle) > 0) {
                BackLeft.setPower(-motorPower);
                FrontLeft.setPower(-motorPower);
                BackRight.setPower(motorPower);
                FrontRight.setPower(motorPower);
            }
            if ((targetAngle - robotAngle) < 0){
                BackLeft.setPower(motorPower);
                FrontLeft.setPower(motorPower);
                BackRight.setPower(-motorPower);
                FrontRight.setPower(-motorPower);
            }


            //define how the angle is changing and deal with the stupid 180 -> -180 thing
            deltaAngle = currentAngle - previousAngle;
            if (deltaAngle > 180)
            {
                deltaAngle -= 360;
            }
            else if(deltaAngle < -180)
            {
                deltaAngle += 360;
            }

            robotAngle += deltaAngle;
            previousAngle = currentAngle;

            telemetry.addData("robotangle", robotAngle);
            telemetry.addData("deltaAngle", deltaAngle);
            telemetry.addData("currentAngle", currentAngle);

            telemetry.update();

        }
        stopRobot();
    }

    public void stopRobot() {
        wheelSetMode(2);

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }


    public void wheelSetMode(int mode){
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (mode == 1){
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (mode == 2){
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void moveRobot(double drive, double turn) {
        double driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        double turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

//        leftSpeed  = drive - turn;
//        rightSpeed = drive + turn;


        flspeed = drive - turn;
        frspeed = drive + turn;
        blspeed = drive - turn;
        brspeed = drive + turn;


        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(flspeed), Math.abs(frspeed));

        if (max > 1.0)
        {
            flspeed /= max;
            frspeed /= max;
            blspeed /= max;
            brspeed /= max;
        }

//        leftDrive.setPower(leftSpeed);
//        rightDrive.setPower(rightSpeed);
        FrontLeft.setPower(flspeed);
        FrontRight.setPower(frspeed);
        BackLeft.setPower(blspeed);
        BackRight.setPower(brspeed);

    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, Constants.P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (Math.abs(headingError) > Constants.HEADING_THRESHOLD) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (holdTimer.time() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

}
