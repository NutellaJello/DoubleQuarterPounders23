package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    /* Declare OpMode members. */
    public Servo claw = null;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
    }

    public void clawOpen() {
        claw.setPosition(0.83);
    }

    public void clawClose() {
        claw.setPosition(0.62);
    }


}
