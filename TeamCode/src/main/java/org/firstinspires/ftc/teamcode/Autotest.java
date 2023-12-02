//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "leftautonoloop") //bluered
//public class autoleft extends LinearOpMode {
////    private DcMotorEx FrontLeft;
////    private DcMotorEx BackLeft;
////    private DcMotorEx BackRight;
////    private DcMotorEx FrontRight;
////
////    private Servo claw;
////    private Servo lswing;
////    private Servo rswing;
////    private Servo cone;
////    private Servo knocker;
//
//    DcMotorEx FrontLeft;
//    DcMotorEx BackLeft;
//    DcMotorEx FrontRight;
//    DcMotorEx BackRight;
//    DcMotor slides;
//    DcMotor intake;
//
//    double speeddamper = 1;
//
//    OpenCvWebcam webcam;
//    String What = "";
//    Double width = 16.0; //inches
//    Integer cpr = 28; //counts per rotation
//    Integer gearratio = 40;
//    Double diameter = 4.125;
//    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
//    Double bias = 0.8;//default 0.8
//    Double meccyBias = 0.9;//change to adjust only strafing movement
//    //
//    Double conversion = cpi * bias;
//    Boolean exit = false;
//
//    //allmotors = new DcMotor[]{FrontRight, FrontLeft, BackRight, BackLeft};
//
//    //HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
//    private ElapsedTime runtime = new ElapsedTime();
//
//    static final double FORWARD_SPEED = 0.6;
//    static final double TURN_SPEED = 0.5;
//
//    private Orientation lastAngles = new Orientation();
//    private double currAngle = 0.0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        FrontLeft = hardwareMap.get(DcMotorEx.class, "frontleft"); //0
//        BackLeft = hardwareMap.get(DcMotorEx.class, "backleft"); //1
//        FrontRight = hardwareMap.get(DcMotorEx.class, "frontright"); //2
//        BackRight = hardwareMap.get(DcMotorEx.class, "backright"); //3
//        //robot.initIMU(hardwareMap);
//
//        //expansion
//        Servo claw = hardwareMap.servo.get("claw");
//        Servo arm = hardwareMap.servo.get("arm");
//        Servo dump = hardwareMap.servo.get("dump");
//        Boolean ranvar = true;
//
//
//        intake = hardwareMap.dcMotor.get("intake");
//        slides = hardwareMap.dcMotor.get("slides"); //0
//
//        //brakes
//        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // ColorRangeSensor conesensor = hardwareMap.get(ColorRangeSensor.class, "conesensor");
//
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
////
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
////        AOEColorPipeline aoepipeline = new AOEColorPipeline("2022-2023", webcam, 140, 75, 40, 70);
////        webcam.setPipeline(aoepipeline);
////        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
////            @Override
////            public void onOpened() {
////                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
////            }
////
////            @Override
////            public void onError(int errorCode) {
////            }
////        });
//
////        claw.setPosition(0.22);
//        //  telemetry.addData("Color", aoepipeline.ColorValue);
//
//        waitForStart();
//
//        // telemetry.update();
//
//
//        //stop if stop button clicked
//        if (isStopRequested()) return;
//
//        //int level = 2;
//        int conter =0;
//        if (opModeIsActive()) {
//
//
//            // telemetry.update();
//            //END CODE
//        }
//
//    }
//    public void moveToPosition(double inches, double speed){
//        //
//        int move = (int)(Math.round(inches*conversion));
//        //
//        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() + move);
//        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + move);
//        BackRight.setTargetPosition(BackRight.getCurrentPosition() + move);
//        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + move);
//        //
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FrontLeft.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        FrontRight.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        //
//        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            if (exit){
//                FrontRight.setPower(0);
//                FrontLeft.setPower(0);
//                BackRight.setPower(0);
//                BackLeft.setPower(0);
//                return;
//            }
//        }
//        FrontRight.setPower(0);
//        FrontLeft.setPower(0);
//        BackRight.setPower(0);
//        BackLeft.setPower(0);
//        return;
//    }
//    //functions
//    public void forward (int distance, int speed) {
//        double FLtarget = (FrontLeft.getCurrentPosition()+(distance/96*537.7));
//        double FRtarget = (FrontRight.getCurrentPosition()+(distance/96*537.7));
//        double BRtarget = (BackRight.getCurrentPosition()+(distance/96*537.7));
//        double BLtarget = (BackLeft.getCurrentPosition()+(distance/96*537.7));
//        //double target = ();
//        FrontLeft.setTargetPosition((int) FLtarget);
//        FrontRight.setTargetPosition((int) FRtarget);
//        BackRight.setTargetPosition((int) BRtarget);
//        BackLeft.setTargetPosition((int) BLtarget);
//        //Turn On RUN_TO_POSITION mode
//        //moves motor
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set power (Speed)
//        FrontRight.setVelocity(speed);
//        FrontLeft.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        //idle while moving
//        while(opModeIsActive() && FrontRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            //idle();
////            telemetry.addData("FrontLeft Driving distance", FrontLeft.getCurrentPosition());
////            telemetry.addData("FrontRight Driving distance", FrontRight.getCurrentPosition());
////            telemetry.addData("BackLeft Driving distance", BackLeft.getCurrentPosition());
////            telemetry.addData("BackRight Driving distance", BackRight.getCurrentPosition());
//            // telemetry.update();
//        }
//        FrontRight.setVelocity(0);
//        FrontLeft.setVelocity(0);
//        BackRight.setVelocity(0);
//        BackLeft.setVelocity(0);
//
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void backward (int distance, int speed) {
//        double FLtarget = (FrontLeft.getCurrentPosition()-(distance/96*537.7));
//        double FRtarget = (FrontRight.getCurrentPosition()-(distance/96*537.7));
//        double BRtarget = (BackRight.getCurrentPosition()-(distance/96*537.7));
//        double BLtarget = (BackLeft.getCurrentPosition()-(distance/96*537.7));
//        //double target = ();
//        FrontLeft.setTargetPosition((int) FLtarget);
//        FrontRight.setTargetPosition((int) FRtarget);
//        BackRight.setTargetPosition((int) BRtarget);
//        BackLeft.setTargetPosition((int) BLtarget);
//        //Turn On RUN_TO_POSITION mode
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set power (Speed)
//        FrontRight.setVelocity(speed);
//        FrontLeft.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        //idle while moving
//        while(opModeIsActive() && FrontRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            //idle();
////            telemetry.addData("FrontLeft Driving distance", FrontLeft.getCurrentPosition());
////            telemetry.addData("FrontRight Driving distance", FrontRight.getCurrentPosition());
////            telemetry.addData("BackLeft Driving distance", BackLeft.getCurrentPosition());
////            telemetry.addData("BackRight Driving distance", BackRight.getCurrentPosition());
//            // telemetry.update();
//        }
//        FrontRight.setVelocity(0);
//        FrontLeft.setVelocity(0);
//        BackRight.setVelocity(0);
//        BackLeft.setVelocity(0);
//
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void strafe (int inches, int speed) {
//        int move = (int)(Math.round(inches*cpi*meccyBias));
//        //
//        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() - move);
//        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + move);
//        BackRight.setTargetPosition(BackRight.getCurrentPosition() + move);
//        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() - move);
//        //
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FrontLeft.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        FrontRight.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        //
//        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            if (exit){
//                FrontRight.setPower(0);
//                FrontLeft.setPower(0);
//                BackRight.setPower(0);
//                BackLeft.setPower(0);
//                return;
//            }
//        }
//        FrontRight.setPower(0);
//        FrontLeft.setPower(0);
//        BackRight.setPower(0);
//        BackLeft.setPower(0);
//        return;
//    }
//    public void strafedub (int move, int speed) {
//        //
//        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() - move);
//        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + move));
//        BackRight.setTargetPosition((int) (BackRight.getCurrentPosition() + move));
//        FrontRight.setTargetPosition((int) (FrontRight.getCurrentPosition() - move));
//        //
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FrontLeft.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        FrontRight.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        //
//        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            if (exit){
//                FrontRight.setPower(0);
//                FrontLeft.setPower(0);
//                BackRight.setPower(0);
//                BackLeft.setPower(0);
//                return;
//            }
//        }
//        FrontRight.setPower(0);
//        FrontLeft.setPower(0);
//        BackRight.setPower(0);
//        BackLeft.setPower(0);
//        return;
//    }
//    public void slider(int distance, int speed) {
//        //sliders go up if -
//        //sliders go down if +
//        //it's reversed.
//        double target = (slides.getCurrentPosition()+(distance/112*145.1));
//        slides.setTargetPosition((int) target);
//        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slides.setPower(speed);
//        while(opModeIsActive() && slides.isBusy()){
//            //idle();
//            //   telemetry.addData("sliders position", slides.getCurrentPosition());
//            //  telemetry.update();
//        }
//    }
//    public void intakes(int distance, int speed) {
//        //sliders go up if -
//        //sliders go down if +
//        //it's reversed.
//        double target = (intake.getCurrentPosition()+(distance/112*145.1));
//        intake.setTargetPosition((int) target);
//        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intake.setPower(speed);
//        while(opModeIsActive() && slides.isBusy()){
//            //idle();
//            //    telemetry.addData("intakes position", intake.getCurrentPosition());
//            // telemetry.update();
//        }
//    }
//    public void turn (int inches, int speed) {
//        // + = left
//        // - = right
////        double FLtarget = (FrontLeft.getCurrentPosition()+(distance/96*537.7));
////        double FRtarget = (FrontRight.getCurrentPosition()-(distance/96*537.7));
////        double BRtarget = (BackRight.getCurrentPosition()-(distance/96*537.7));
////        double BLtarget = (BackLeft.getCurrentPosition()+(distance/96*537.7));
//        //double target = ();
//        int move = (int)(Math.round(inches*conversion));
//        //
//        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() + move);
//        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + move);
//        BackRight.setTargetPosition(BackRight.getCurrentPosition() - move);
//        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() - move);
////        FrontLeft.setTargetPosition((int) FLtarget);
////        FrontRight.setTargetPosition((int) FRtarget);
////        BackRight.setTargetPosition((int) BRtarget);
////        BackLeft.setTargetPosition((int) BLtarget);
//        //Turn On RUN_TO_POSITION mode
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set power (Speed)
//        FrontRight.setVelocity(speed);
//        FrontLeft.setVelocity(speed);
//        BackRight.setVelocity(speed);
//        BackLeft.setVelocity(speed);
//        //idle while moving
//        while(opModeIsActive() && FrontRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            //idle();
////            telemetry.addData("FrontLeft Driving distance", FrontLeft.getCurrentPosition());
////            telemetry.addData("FrontRight Driving distance", FrontRight.getCurrentPosition());
////            telemetry.addData("BackLeft Driving distance", BackLeft.getCurrentPosition());
////            telemetry.addData("BackRight Driving distance", BackRight.getCurrentPosition());
//            // telemetry.update();
//        }
//        FrontRight.setVelocity(0);
//        FrontLeft.setVelocity(0);
//        BackRight.setVelocity(0);
//        BackLeft.setVelocity(0);
//    }
//    public void leftturn (int distance, int speed) {
//        // + = left
//        // - = right
//        double BRtarget = (BackRight.getCurrentPosition()+(distance/96*537.7));
//        double FRtarget = (FrontRight.getCurrentPosition()+(distance/96*537.7));
//
//        //double target = ();
//        BackRight.setTargetPosition((int) BRtarget);
//        FrontRight.setTargetPosition((int) FRtarget);
//        //Turn On RUN_TO_POSITION mode
//        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Set power (Speed)
//        BackRight.setVelocity(speed);
//        FrontRight.setVelocity(speed);
//        //idle while moving
//        while(opModeIsActive() && FrontRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy() && BackRight.isBusy()){
//            //idle();
////            telemetry.addData("BackRight Driving distance", BackRight.getCurrentPosition());
////            telemetry.addData("FrontRight Driving distance", FrontRight.getCurrentPosition());
//            // telemetry.update();
//        }
//        BackRight.setVelocity(0);
//        FrontRight.setVelocity(0);
//
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void resetAngle() {
//       // lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        //lastAngles = new Orientation();
//        currAngle = 0;
//    }
//
//    public double getAngle() {
//
//        // Get current orientation
//        //Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        // Change in angle = current angle - previous angle
//        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
//
//        // Gyro only ranges from -179 to 180
//        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
//        if (deltaAngle < -180) {
//            deltaAngle += 360;
//        } else if (deltaAngle > 180) {
//            deltaAngle -= 360;
//        }
//
//        // Add change in angle to current angle to get current angle
//        currAngle += deltaAngle;
//        lastAngles = orientation;
//        // telemetry.addData("gyro", orientation.firstAngle);
//        return currAngle;
//    }
//
////    public void turn(double degrees){
////        //();
////
////        double error = degrees;
////
////        while (opModeIsActive() && Math.abs(error) > 1) {
////            double motorPower = (error < 0 ? -0.8 : 0.8);
////            FrontLeft.setVelocity(-motorPower);
////            BackLeft.setVelocity(motorPower);
////            FrontRight.setVelocity(-motorPower);
////            BackRight.setVelocity(motorPower);
////            error = degrees - getAngle();
////            telemetry.addData("error", error);
////            telemetry.update();
////        }
////
////        FrontLeft.setVelocity(0);
////        BackLeft.setVelocity(0);
////        FrontRight.setVelocity(0);
////        BackRight.setVelocity(0);
////    }
//
//    public void turnTo(double degrees){
//
//        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        System.out.println(orientation.firstAngle);
//        double error = degrees - orientation.firstAngle;
//
//        if (error > 180) {
//            error -= 360;
//        } else if (error < -180) {
//            error += 360;
//        }
//
////        turn(error);
//    }
//
//    public double getAbsoluteAngle() {
//        return robot.imu.getAngularOrientation(
//                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
//        ).firstAngle;
//    }
//
//}