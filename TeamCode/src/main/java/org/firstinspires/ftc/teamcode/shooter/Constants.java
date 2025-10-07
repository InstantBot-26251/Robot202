package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.util.math.MathPM;

public class Constants {
    // Constants
    private static double FULL_POWER = 1.0;
    private static double REJECT_POWER = 0.5;
    private static double TEST_HOOD_ANGLE = 30.0;
    private static double DEFAULT_HEIGHT_DIFF = 0.5; // meters - TODO: measure
    private static double FLYWHEEL_RPM = 3000; // TODO: measure
    private static double WHEEL_DIAMETER = MathPM.inchesToMeters(4.0); // TODO: measure actual diameter
    private static double DRAG_COEFFICIENT = 0.8; // TODO: tune
    private static double FLYWHEEL_SPINUP_TIME = 0.3; // seconds to wait for flywheel to reach speed
}
