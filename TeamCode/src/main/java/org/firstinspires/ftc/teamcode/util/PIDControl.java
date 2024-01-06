package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl{
    double intergralSum =0;
    double Kp = 0.15;
    double Ki= 0.0;
    double Kd =0.03;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;




    public double PIDValue(double reference, double state){
        double error = reference-state;
        intergralSum +=error *timer.seconds();
        double derivative = (error-lastError) / timer.seconds();

        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;
        lastError = error;

        timer.reset();

        double output =  (error*Kp) + (derivative*Kd) + (intergralSum*Ki);
        return output;
    }


}