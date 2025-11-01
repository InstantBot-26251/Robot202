package org.firstinspires.ftc.teamcode.shooter.constants;

import org.firstinspires.ftc.teamcode.util.math.MathPM;

public class Constants {
    // Constants
    public static double FULL_POWER = 1.0;
    public static double REJECT_POWER = 0.5;
    public static double TEST_HOOD_ANGLE = 30.0;
    public static double DEFAULT_HEIGHT_DIFF = MathPM.inchesToMeters(41.45); // meters
    public static double FLYWHEEL_RPM = 6000;
    public static double WHEEL_DIAMETER = MathPM.inchesToMeters(1.93);
    public static double DRAG_COEFFICIENT = 0.8; // TODO: tune
    public static double FLYWHEEL_SPINUP_TIME = 0.3; // seconds to wait for flywheel to reach speed
}
