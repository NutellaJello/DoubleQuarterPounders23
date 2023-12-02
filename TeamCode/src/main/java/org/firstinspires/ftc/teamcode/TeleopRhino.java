//ftc package yay!
package org.firstinspires.ftc.teamcode;

//importing needed things
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
//@Disabled
//beginning of class
public class TeleopRhino extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        //motors
        final double dampSpeedRatio = 0.85;
        final double dampTurnRatio = 0.8;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft"); //0
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft"); //1
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight"); //3

        Servo flopper = hardwareMap.servo.get("flopper"); //0
        Servo claw = hardwareMap.servo.get("claw"); //1
        //flopper.setDirection(Servo.Direction.REVERSE);
//        //expansion
        Servo leftarm = hardwareMap.servo.get("leftarm"); //port 0 lswing
        Servo rightarm = hardwareMap.servo.get("rightarm"); //port 1 rswing
        //imma make a double that updates the servo posi
        rightarm.setDirection(Servo.Direction.REVERSE);
        double sPosiL = 0.0;

        DcMotor slides = hardwareMap.dcMotor.get("slides"); //0

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);





        //wait for button click
        waitForStart();
        // stop the program if button click
        if (isStopRequested()) return;
        boolean slowMode = false;

        while (opModeIsActive()) {
            telemetry.addData("leftarm", Double.toString(leftarm.getPosition()));
            telemetry.addData("rightarm", Double.toString(rightarm.getPosition()));
            telemetry.addData("claw", Double.toString(claw.getPosition()));
            telemetry.addData("flopper", Double.toString(flopper.getPosition()));

            leftarm.setPosition(sPosiL);
            rightarm.setPosition(sPosiL);

            if(sPosiL >1){

            }

            if (gamepad2.a){
                sPosiL = 0.99;

            }
            if (gamepad2.b){
                sPosiL = 0.02;

            }

            if (gamepad2.left_bumper){
                claw.setPosition(0.045);
            }
            else if (gamepad2.right_bumper) {
                claw.setPosition(0.07);
            }
            if (gamepad2.x){
                flopper.setPosition(1);
            }
            else if (gamepad2.y) {
                flopper.setPosition(0.8);
            }


            slides.setPower(gamepad2.left_stick_y* 0.5);

            double y = Range.clip(gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);


            double flPower = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            double frPower = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
            double blPower = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            double brPower = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxFront = Math.max(flPower, frPower);
            double maxBack = Math.max(blPower, brPower);
            double maxPower = Math.max(maxFront, maxBack);

            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }
            //finally moving the motors
            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);

            telemetry.update();
// not useful for now


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