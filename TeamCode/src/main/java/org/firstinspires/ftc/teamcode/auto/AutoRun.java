package org.firstinspires.ftc.teamcode.auto;
/*
FTC team 23928
2023-2024 Autonoumous Java Code
Keep Confidential
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "autorunL")
public class AutoRun extends LinearOpMode {
    /*
    private static double ServoLeft = 0.65;
    private static double ServoMiddle = 0.45;
    private static double ServoRight = 0.3;

    public DcMotorEx frontLeft;
    public DcMotorEx bottomLeft;
    public DcMotorEx bottomRight;
    public DcMotorEx frontRight;
    public DcMotorEx linearSlide1;
    public DcMotorEx linearSlide2;
    public Servo claw;
    public Servo clawSpin;
    public DcMotorEx[] motorsDrive;
    public DcMotorEx[] motorsLeft;
    public DcMotorEx[] motorsRight;
    public DcMotorEx[] frontLbottomR;
    public DcMotorEx[] frontRbottomL;
    public DcMotorEx[] linearSlides;

    private static Integer POSITION_FORWARD_ONE_BLOCK = 1092;
    private static Integer POSITION_RIGHT_ONE_BLOCK = 1250;

    //center of the claw to the bottom
    private static double clawToBottom = 7.5 / 24;
    private static double robotLength = 15 / 24;

    private static int LINEAR_SLIDE_HIGH = 2000;
    private static int LINEAR_SLIDE_MIDDLE = 800;
    private static int LINEAR_SLIDE_LOW = 600;

    private static int rotate = 1000;

    private static double VELOCITY_SLIDES = 1000;
    private static double VELOCITY = 900;


    private static double LINEAR_SLIDE_HIGH_POWER = 1.05;
    private static double LINEAR_SLIDE_MIDDLE_POWER = 0.6;
    private static double LINEAR_SLIDE_LOW_POWER = 0.6;

*/
    OpenCvWebcam webcam;
    String what = "";

//    private void linearSlidePosition(int cycle, double velocity) {
//        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide1.setTargetPosition(cycle);
//        linearSlide1.setVelocity(velocity);
//    }
/*
    private void robotMove(String packingPosition) {
        robotMoveMiddle();
        sleep(3000);

        moveForward1(POSITION_FORWARD_ONE_BLOCK, VELOCITY);
        sleep(3000);
        rotateLeftTest(rotate, VELOCITY);

        putExtraCone();
        putExtraCone();
        putExtraCone();
        putExtraCone();
        putExtraCone();

        moveBackward1(POSITION_FORWARD_ONE_BLOCK, VELOCITY);
        sleep(3000);
        rotateRightTest(rotate, VELOCITY);

        parking(packingPosition);
        sleep(5000000);
    }



    private void robotMoveMiddle() {
        // close claw and lift linear slide (to avoid stuck the color cone)
        clawClose();
        sleep(400);
        // linearSlide1.setPower(LINEAR_SLIDE_MIDDLE_POWER);
        // linearSlide2.setPower(-LINEAR_SLIDE_MIDDLE_POWER);
        linearSlideEncoder(LINEAR_SLIDE_MIDDLE, VELOCITY_SLIDES);

        sleep(2500);

        // move forward and left a little
        Integer forwardToMidPole = (int) (POSITION_FORWARD_ONE_BLOCK * (2 - clawToBottom));
        moveBackward1(forwardToMidPole, VELOCITY);
        sleep(5500);
        moveLeft1((int) (POSITION_RIGHT_ONE_BLOCK * 0.14), VELOCITY );
        sleep(1500);

        //put left cone
        clawSpin.setPosition(0.95);
        sleep(1000);
        clawOpen();
        sleep(1000);

        // claw back to normal position
        clawSpin.setPosition(0.46);
        linearSlideEncoder(LINEAR_SLIDE_LOW, VELOCITY_SLIDES);
        sleep(1000);
        clawClose();
        sleep(1000);
        //linearSlide.setPower(-0.3);
        //linearSlidePosition(-20, 200);
        sleep(1000);
        moveRight1((int) (POSITION_RIGHT_ONE_BLOCK * 0.14), VELOCITY);
        sleep(1000);
        moveBackward1((int) (POSITION_RIGHT_ONE_BLOCK * 0.5), VELOCITY);

    }



    private void parking(String position) {
        // moves left if pos = 1, does nothing if pos = 2, and moves right if pos = 3

        moveForward1(POSITION_FORWARD_ONE_BLOCK/2 + 80, VELOCITY);

        if (position == "red") {
            moveLeft1(POSITION_RIGHT_ONE_BLOCK, VELOCITY);
        }
        else if (position == "green") {
            // do nothing lmao
        }
        else if (position == "blue") {
            moveRight1(POSITION_RIGHT_ONE_BLOCK, VELOCITY);
        }
    }


    private void robotMoveToHighPole() {

        clawClose();
        sleep(300);
        linearSlide1.setPower(LINEAR_SLIDE_HIGH_POWER);
        sleep(1500);

        Integer forwardToHighPole = (int) (POSITION_FORWARD_ONE_BLOCK * (3 - clawToBottom));
        moveBackward1(forwardToHighPole, VELOCITY);
        sleep(4000);

        //put left cone
        clawSpin.setPosition(0.8);
        sleep(2000);
        clawOpen();
        sleep(1000);
        clawSpin.setPosition(0.42);
        sleep(1000);
        clawClose();
        sleep(1000);
        linearSlide1.setPower(-0.6);
        sleep(1000);

    }

    private void putExtraCone() {


        clawOpen();
        sleep(1000);

        linearSlideEncoder(LINEAR_SLIDE_LOW, VELOCITY_SLIDES);
        sleep(1000);


        moveForward1(POSITION_RIGHT_ONE_BLOCK, VELOCITY);
        sleep(3000);
        clawClose();


        sleep(300);
        linearSlideEncoder(LINEAR_SLIDE_MIDDLE, VELOCITY_SLIDES);

        moveBackward1(POSITION_RIGHT_ONE_BLOCK, VELOCITY);
        sleep(1000);

        putLeftCone();

        sleep(500);

    }

    private void putLeftCone() {
        clawSpin.setPosition(0);
        sleep(2000);
        clawOpen();
        sleep(1000);
        clawSpin.setPosition(0.5);
        sleep(1000);
        clawClose();
    }
*/
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ColorPipeline centerColor = new ColorPipeline(webcam, 120, 35, 50, 40, telemetry);
        //ColorPipeline right = new ColorPipeline(webcam, 0, 35, 50, 40);
        //webcam code
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        waitForStart();
        /*
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        linearSlide1 = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
        claw = hardwareMap.servo.get("claw");
        clawSpin = hardwareMap.servo.get("clawSpin");

        motorsDrive = new DcMotorEx[]{this.frontLeft, this.frontRight, this.bottomLeft, this.bottomRight};
        motorsLeft = new DcMotorEx[]{this.frontLeft, this.bottomLeft};
        motorsRight = new DcMotorEx[]{this.frontRight, this.bottomRight};
        frontRbottomL = new DcMotorEx[]{this.frontRight, this.bottomLeft};
        frontLbottomR = new DcMotorEx[]{this.frontLeft, this.bottomRight};
        linearSlides = new DcMotorEx[]{this.linearSlide1, this.linearSlide2};

        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE);
        this.bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        this.bottomRight.setDirection(DcMotor.Direction.REVERSE);
        this.linearSlide2.setDirection(DcMotor.Direction.REVERSE);

*/
        sleep(1000);
        int count = 0;
        String position = "l";
        if (centerColor.getRed()> 200) {
            position = "c";
            webcam.stopStreaming();
        } else {
            while (count < 30) {
                if (centerColor.getRed() > 200) {
                    position = "c";
                    webcam.stopStreaming();
                    break;
                }
                count++;
                sleep(100);
            }

        }



        // detect camera colornt

        while (opModeIsActive()) {
            telemetry.addData("postion", position);
            telemetry.addData("Count", count);
            telemetry.addData("center", centerColor.getRed());
            telemetry.addData("center", centerColor);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);


            //robotMove(position);


        }
    }
/*
    private void moveFrontLeft(int cycle, double velocity) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(cycle);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setVelocity(velocity);
    }

    private void moveFrontRight(int cycle, double velocity) {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(cycle);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setVelocity(velocity);
    }

    private void moveBottomLeft(int cycle, double velocity) {
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setTargetPosition(cycle);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setVelocity(velocity);
    }

    private void movebottomRight(int cycle, double velocity) {
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setTargetPosition(cycle);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setVelocity(velocity);
    }


    // this stuff defines the movement functions
    private void moveForward1(int cycle, double velocity) {
        // move forward
        for (DcMotorEx motor : motorsDrive) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }


    private void moveBackward1(int cycle, double velocity) {
        // move backwards
        for (DcMotorEx motor : motorsDrive) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }


    private void moveLeft1(int cycle, double velocity) {
        for (DcMotorEx motor : frontLbottomR) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotorEx motor : frontRbottomL) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } // move robot left
        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }

    private void moveRight1(int cycle, double velocity) {
        for (DcMotorEx motor : frontLbottomR) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotorEx motor : frontRbottomL) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } // move robot right
        for (DcMotorEx motor : motorsDrive) {
            motor.setVelocity(velocity);
        }
    }

    private void rotateLeftTest(int cycle, double velocity) {
        for (DcMotorEx motor : frontLbottomR) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotorEx motor : frontLbottomR) {
            motor.setVelocity(velocity);
        }
    }

    private void rotateRightTest(int cycle, double velocity) {
        for (DcMotorEx motor : frontRbottomL) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(-cycle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotorEx motor : frontRbottomL) {
            motor.setVelocity(velocity);
        }
    }



    private void printColor(double red, double green, double blue) {
        telemetry.addData("totalRed", red);
        telemetry.addData("totalGreen", green);
        telemetry.addData("totalBlue", blue);
        telemetry.update();
    }

    private void clawClose() {
        claw.setPosition(-0.4);
        sleep(1200);
    }

    private void clawOpen() {
        claw.setPosition(1.0);
    }





    private void robotMoveOld(String position) {

        // close claw
        claw.setPosition(-0.4);
        sleep(1200);

        // linearSlidePower(-1.1);
        sleep(100);

        // move forward
        moveForward(0.5, 2850);
        sleep(500);

        clawSpin.setPosition(0);

        sleep(2500);

        claw.setPosition(0.4);

        sleep(500);

        moveBackwards(0.2, 2500);

        clawSpin.setPosition(0.35);
        sleep(2500);

        // linearSlidePower(0.5);

        sleep(1000);

        // linearSlidePower(0);

        sleep(500);

    }

    private void parkingOld(String position) {
        // moves left if pos = 1, does nothing if pos = 2, and moves right if pos = 3
        if (position == "red") {
            moveLeft(0.35, 2400);
        }
        else if (position == "green") {
            // do nothing lmao
        }
        else if (position == "blue") {
            moveRight(0.35, 2100);
        }

        sleep(42069);
    }





    // this stuff defines the movement functions
    private void moveLeft(double power, int time) {
        // move left
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(power);
        }
        sleep(time);
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(0);
        }

    }
    private void moveRight(double power, int time) {
        // move right
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(-power);
        }
        sleep(time);
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(0);
        }

    }


    private void moveBackwards(double power, int time) {
        for (DcMotorEx motor : frontLbottomR) {
            motor.setPower(power);
        }
        for (DcMotorEx motor : frontRbottomL) {
            motor.setPower(-power);
        } // move robot backwards
        sleep(time);
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(0);
        }

    }

    private void moveForward(double power, int time) {
        for (DcMotorEx motor : frontLbottomR) {
            motor.setPower(-power);
        }
        for (DcMotorEx motor : frontRbottomL) {
            motor.setPower(power);
        } // move robot forwards
        sleep(time);
        for (DcMotorEx motor : motorsDrive) {
            motor.setPower(0);
        }
    }

    private void linearSlideEncoder(double position, double speed) {
        for (DcMotor motor : linearSlides) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setTargetPosition((int) position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        linearSlide1.setVelocity(speed);
        linearSlide2.setVelocity(speed);
    }

    private void wheelTest() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(2000);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setVelocity(1000);

        sleep(5000);

        bottomLeft.setTargetPosition(2000);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setVelocity(1000);

        sleep(5000);

        bottomRight.setTargetPosition(2000);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setVelocity(1000);

        sleep(5000);

        frontRight.setTargetPosition(2000);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setVelocity(1000);

        clawClose();
        // linearSlidePosition(LINEAR_SLIDE_HIGH, VELOCITY_SLIDES);
        Integer toMiddlePole = (int)  (POSITION_FORWARD_ONE_BLOCK * (2 - clawToBottom));
        moveForward1(toMiddlePole, VELOCITY);
        frontRight.setVelocity(1000);
    }
*/
}