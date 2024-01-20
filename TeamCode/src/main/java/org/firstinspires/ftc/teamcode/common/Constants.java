package org.firstinspires.ftc.teamcode.common;

public class Constants {
    public static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.3;     // Max Turn speed to limit turn rate 0.2
    public static final double     HEADING_THRESHOLD       = 2.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Pro portional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    public static final double     P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_GAIN           = 0.02;     // Larger is more responsive, but also less stable

    public static final int SLIDE_VELOCITY = 500;
    public static final int SLIDE_HIGH_POSITION = -1200;

    public static final int SLIDE_LOW_POSITION = -50;

    //test

}
