package org.firstinspires.ftc.teamcode.auto;
/*
FTC team 23928
2023-2024 Autonoumous Java Code
Keep Confidential
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "autorunColor3")
@Disabled
public class AutoRunColor extends LinearOpMode {

    OpenCvWebcam webcam;
    String what = "";

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamPropDetectionPipeline centerColor = new TeamPropDetectionPipeline(webcam, telemetry);
        webcam.setPipeline(centerColor);
        //ColorPipeline right = new ColorPipeline(webcam, 0, 35, 50, 40);
        //webcam code
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        waitForStart();
        sleep(1000);



        // detect camera colornt

        while (opModeIsActive()) {
            sleep(1000);
            telemetry.addData("postion", centerColor.getPosition());
            telemetry.addData("postion", centerColor.avgLeft);
            telemetry.addData("postion", centerColor.avgCenter);
            telemetry.addData("postion", centerColor.avgRight);

            telemetry.addData("center", centerColor);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(500000);


            //robotMove(position);


        }
    }
}