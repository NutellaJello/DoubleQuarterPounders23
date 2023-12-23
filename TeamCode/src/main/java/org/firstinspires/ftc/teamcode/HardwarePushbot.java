//package org.firstinspires.ftc.teamcode;
//
//
//import static android.os.SystemClock.sleep;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
///**
// * This is NOT an opmode.
// *
// * This class can be used to define all the specific hardware for a single robot.
// * In this case that robot is a Pushbot.
// * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
// *
// * This hardware class assumes the following device names have been configured on the robot:
// * Note:  All names are lower case and some have single spaces between words.
// *
// * Motor channel:  Left  drive motor:        "left_drive"
// * Motor channel:  Right drive motor:        "right_drive"
// * Motor channel:  Manipulator drive motor:  "left_arm"
// * Servo channel:  Servo to open left claw:  "left_hand"
// * Servo channel:  Servo to open right claw: "right_hand"
// */
//@Disabled
//public class HardwarePushbot
//{
//    /* local OpMode members. */
//    HardwareMap hwMap           =  null;
//    /* Public OpMode members. */
//    public DcMotorEx leftFront;
//    public DcMotorEx  rightFront;
//    public DcMotorEx  leftBack;
//    public DcMotorEx  rightBack;
//
//    BNO055IMU imu;
//
//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;
//    public static final double BackPowerBoost = 1.3;
//
//    private ElapsedTime period  = new ElapsedTime();
//
//    /* Constructor */
//
//    /* Initialize standard Hardware interfaces */
//    public void init(HardwareMap ahwMap) {
//        // Save reference to Hardware map
//        hwMap = ahwMap;
//
//        // Define and Initialize Motors
//        leftFront   = hwMap.get(DcMotorEx.class, "frontleft");
//        rightFront  = hwMap.get(DcMotorEx.class, "frontright");
//        leftBack   = hwMap.get(DcMotorEx.class, "backleft");
//        rightBack  = hwMap.get(DcMotorEx.class, "backright");
//
//        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        leftBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set all motors to zero power
//        setAllPower(0);
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        initIMU(ahwMap);
//    }
//
//    public void initIMU(HardwareMap ahwMap) {
//        hwMap = ahwMap;
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        //sleep(3000);
//    }
//
//    //Set power to all motors
//    public void setAllPowerVel(double p){
//        setMotorPower(p,p,p,p);
//    }
//
//    public void setMotorPowerVel(double lF, double rF, double lB, double rB){
//        leftFront.setVelocity(lF);
//        leftBack.setVelocity(lB*BackPowerBoost);
//        rightBack.setVelocity(rB*BackPowerBoost);
//        rightFront.setVelocity(rF);
//    }
//
//    public void setAllPower(double p){
//        setMotorPower(p,p,p,p);
//    }
//
//    public void setMotorPower(double lF, double rF, double lB, double rB){
//        rightFront.setPower(rF);
//        leftFront.setPower(lF);
//        leftBack.setPower(lB*BackPowerBoost);
//        rightBack.setPower(rB*BackPowerBoost);
////        rightFront.setPower(rF);
//    }
//
//    public void setAllVelocity(double p){
//        setMotorVelocity(p,p,p,p);
//    }
//
//    public void setMotorVelocity(double lF, double rF, double lB, double rB){
//        rightFront.setVelocity(rF);
//        leftFront.setVelocity(lF);
//        leftBack.setVelocity(lB);
//        rightBack.setVelocity(rB);
//    }
//}