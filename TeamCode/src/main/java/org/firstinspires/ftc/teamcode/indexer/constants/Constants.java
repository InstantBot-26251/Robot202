package org.firstinspires.ftc.teamcode.indexer.constants;

public class Constants {
    // Constants
    public static final double ROTOR_CALIBRATION_SPEED = 0.15;
    public static final double ROTOR_INDEX_SPEED = 0.3;
    public static final double ROTOR_DISPENSE_SPEED = 0.4;

    // Ball detection thresholds
    public static final int ARTIFACT_PRESENCE_THRESHOLD = 50; // Any RGB value > this = ball present

    // Ball color thresholds (RGB values) - TODO: Tune these values
    public static final int GREEN_THRESHOLD = 150;
    public static final int PURPLE_RED_THRESHOLD = 120; // Purple has red component
    public static final int PURPLE_BLUE_THRESHOLD = 120; // Purple has blue component
    public static final double PURPLE_MIN_RATIO = 0.7; // Red/Blue ratio for purple detection

}
