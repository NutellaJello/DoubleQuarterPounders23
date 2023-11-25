//ftc package yay!
package org.firstinspires.ftc.teamcode;

//importing needed things
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
//beginning of class
public class TeleopRhino extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        //motors
        //test 10/29/22
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft"); //0
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft"); //1
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight"); //3
//        Servo cone = hardwareMap.servo.get("cone"); //0
//        Servo knocker = hardwareMap.servo.get("knocker"); //1
//        //expansion
//        Servo lswing = hardwareMap.servo.get("lswing"); //port 0 lswing
//        Servo rswing = hardwareMap.servo.get("rswing"); //port 1 rswing
//        Servo claw = hardwareMap.servo.get("claw"); //port 2 claw
       // DcMotor slides = hardwareMap.dcMotor.get("slides"); //0

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        //wait for button click
        waitForStart();
        // stop the program if button click
        if (isStopRequested()) return;
        boolean slowMode = false;
        while (opModeIsActive()) {
            //finally moving the motors
//            if (gamepad1.dpad_up){
//                slides.setPower(0.5);
//
//            }
//            if (gamepad1.dpad_down){
//                slides.setPower(-0.5);
//
//            }

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x ==0){
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            if (gamepad1.right_stick_y == 0 && gamepad1.right_stick_x ==0){
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            if (gamepad1.left_stick_y > 0.2){
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(0.5);
            }
            else if (gamepad1.left_stick_y < -0.2){
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(-0.5);
            }
            if (gamepad1.left_stick_x != 0 && gamepad1.left_stick_x < 0){
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(0.5);
            }
            else if (gamepad1.left_stick_x != 0 && gamepad1.left_stick_x > 0){
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(0.5);

            }
            if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_x != 0){
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(0.5);
            }
            if (gamepad1.right_stick_x < 0  && gamepad1.right_stick_x != 0){
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(-0.5);
            }

// not useful for now
            //            if (gamepad2.left_stick_y > 0){
//                slides.setPower(gamepad2.left_stick_y);
//            }
//            else if (gamepad2.left_stick_y < 0){
//                slides.setPower(gamepad2.left_stick_y);
//            }
//            slides.setPower(0);
//            if (gamepad2.dpad_up){
//                lswing.setPosition(lswing.getPosition()+0.1);
//                rswing.setPosition(lswing.getPosition()-0.1);
//            }
//            else if (gamepad2.dpad_down){
//                lswing.setPosition(lswing.getPosition()-20);
//                rswing.setPosition(lswing.getPosition()-20);
//            }
//            if (gamepad2.left_trigger > 0){
//                cone.setPosition(cone.getPosition()+0.1);
//            }
//            else{
//                cone.setPosition(cone.getPosition()-0.1);
//            }
//
//            if (gamepad2.left_bumper){
//                claw.setPosition(cone.getPosition()+1);
//            }
//            else if (gamepad2.right_bumper){
//                claw.setPosition(cone.getPosition()-1);
//            }
        }
    }
}