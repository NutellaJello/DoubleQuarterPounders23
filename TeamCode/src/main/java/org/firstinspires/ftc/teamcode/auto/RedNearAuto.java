/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.Arm;
import org.firstinspires.ftc.teamcode.common.Claw;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Flopper;
import org.firstinspires.ftc.teamcode.common.Slides;
import org.firstinspires.ftc.teamcode.common.Vehicle;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *`
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AUTO_RED_NEAR")

public class RedNearAuto extends LinearOpMode {

    /* Declare OpMode members. */


    @Override
    public void runOpMode() {
        Vehicle vehicle = new Vehicle(hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap);
        Flopper flopper = new Flopper(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        HSVDetectionRed detectionRed = new HSVDetectionRed(webcam, telemetry);
        webcam.setPipeline(detectionRed);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });


        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f");
            telemetry.addData("","");
            telemetry.update();
        }

        waitForStart();


        if (detectionRed.getPosition() == HSVDetectionRed.ParkingPosition.CENTER) {
            webcam.stopStreaming();
            claw.clawClose();
            sleep(500);
            arm.armHover();
            sleep(500);
            // drives forward and then places on the center
            vehicle.driveStraight(Constants.DRIVE_SPEED, -24.5, 0.0);    // Claw forward 27"
            vehicle.holdHeading(Constants.TURN_SPEED, 0.0, 0.5);
            arm.armDown();
            sleep(500);
            claw.clawOpen();
            sleep(500);
            arm.armUp();
            sleep(500);
            vehicle.driveStraight(Constants.DRIVE_SPEED, -4.5, 0.0);
            sleep(500);
            // turns and then drives to the board then places pixel.
            vehicle.turnToHeading(Constants.TURN_SPEED,90);
            vehicle.holdHeading(Constants.TURN_SPEED, 90, 0.5);
            vehicle.driveStraight(Constants.DRIVE_SPEED, 40.7, 90.0);
            sleep(500);
            placePixel(vehicle,arm,flopper,slides);
            // this moves the robot to the left side of the board and parks.
            vehicle.strafeByInches(Constants.DRIVE_SPEED,35);
            sleep(1000);
             vehicle.driveStraight(Constants.DRIVE_SPEED,5,90);
        }

        else if (detectionRed.getPosition() == HSVDetectionRed.ParkingPosition.RIGHT) {
            webcam.stopStreaming();
            claw.clawClose();
            sleep(500);
            // drives forward, turns, moves forward to place the pixel on the right
            vehicle.driveStraight(Constants.DRIVE_SPEED, -29.0, 0.0);    // Claw forward 27"
            vehicle.holdHeading(Constants.TURN_SPEED, 0.0, 0.5);
            vehicle.turnToHeading(Constants.TURN_SPEED,90);
            vehicle.holdHeading(Constants.TURN_SPEED, 90, 0.5);
            vehicle.driveStraight(Constants.DRIVE_SPEED, 25.5, 90.0);
            arm.armDown();
            sleep(1000);
            claw.clawOpen();
            sleep(500);
            arm.armUp();
            sleep(500);
            //drive to the board
            vehicle.driveStraight(Constants.DRIVE_SPEED, 11.5, 90.0);
            vehicle.holdHeading(Constants.TURN_SPEED, -90.0, 0.5);
            //place on left side board
            vehicle.strafeByInches(Constants.DRIVE_SPEED, -8);
            sleep(500);
            placePixel(vehicle,arm,flopper,slides);
            // this moves the robot to the left side of the board and parks.
            vehicle.strafeByInches(Constants.DRIVE_SPEED,13);
            sleep(500);
            vehicle.driveStraight(Constants.DRIVE_SPEED,5,90);

        }

        else if (detectionRed.getPosition() == HSVDetectionRed.ParkingPosition.LEFT) {
            webcam.stopStreaming();
            claw.clawClose();
            sleep(500);
            // drives forward, turns, moves slightly forward to place the pixel on the left
            vehicle.driveStraight(Constants.DRIVE_SPEED, -29.0, 0.0);
            vehicle.holdHeading(Constants.TURN_SPEED, 0.0, 0.5);
            vehicle.turnToHeading(Constants.TURN_SPEED,90);
            vehicle.holdHeading(Constants.TURN_SPEED, 90, 0.5);
            vehicle.driveStraight(Constants.DRIVE_SPEED, 4.0, 90.0);
            sleep(500);
            vehicle.strafeByInches(Constants.DRIVE_SPEED,-1);
            arm.armDown();
            sleep(1000);
            claw.clawOpen();
            sleep(500);
            arm.armUp();
            sleep(500);
            // drives to the backdrop
            vehicle.driveStraight(Constants.DRIVE_SPEED, 39 , 90.0);
            //place on right side board
            vehicle.strafeByInches(Constants.DRIVE_SPEED, -7);
            sleep(500);
            placePixel(vehicle,arm,flopper,slides);
            // this moves the robot to the left side of the board and parks.
            vehicle.strafeByInches(Constants.DRIVE_SPEED,30);
            sleep(500);
            vehicle.driveStraight(Constants.DRIVE_SPEED,3,90);

        }


        //parking to the corner
       // parkToCorner(vehicle);

        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(100000);  // Pause to display last telemetry message.
    }

    private void parkToCorner(Vehicle vehicle) {
        vehicle.turnToHeading(Constants.TURN_SPEED,0);
        vehicle.holdHeading(Constants.TURN_SPEED, 0, 0.5);
        sleep(1000);
        vehicle.driveStraight(Constants.DRIVE_SPEED, 18, 0.0);
        sleep(1000000);
    }
    public void placePixel(Vehicle vehicle,Arm arm, Flopper flopper, Slides slides){
        arm.armDown();
        sleep(500);
        slides.slidesUp();
        sleep(500);
        flopper.flopperHold();
        sleep(500);
        slides.slidesHold();
        sleep(500);
        flopper.flopperDump();
        sleep(1000);
        vehicle.driveStraight(Constants.DRIVE_SPEED,-3,90);
        sleep(500);
        flopper.flopperRest();
        sleep(500);
        slides.slidesDown();
        sleep(500);
        arm.armUp();
        sleep(500);
    }
}
