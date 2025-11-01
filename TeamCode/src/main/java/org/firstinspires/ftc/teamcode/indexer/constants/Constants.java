package org.firstinspires.ftc.teamcode.indexer.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    /**
     * ROTOR PID CONTROL
     */
    // PID Constants - TUNE THESE
    public static double ROTOR_kP = 0.015;
    public static double ROTOR_kI = 0.0;
    public static double ROTOR_kD = 0.0001;

    // Position Constants - TUNE THESE
    public static double ENCODER_TICKS_PER_SLOT = 680; // Distance between slots in encoder ticks
    public static double TRANSFER_POSITION_OFFSET = 999; // Offset from slot 0 to transfer position

    // Control Constants
    public static double ROTOR_ERROR_TOLERANCE = 10.0; // Encoder ticks
    public static double ROTOR_MAX_POWER = 0.6;
    public static double ROTOR_MIN_POWER = -0.6;

    /**
     * HOPPER CONTROL
     */
    public static double HOPPER_KICK_POWER = 1.0;
    public static double HOPPER_STOP_POWER = 0.0;

    /**
     * ARTIFACT DETECTION THRESHOLDS
     */
    // Presence detection

    // Color detection thresholds - TUNE THESE
    public static int GREEN_THRESHOLD = 150;
    public static int PURPLE_RED_THRESHOLD = 120;
    public static int PURPLE_BLUE_THRESHOLD = 120;
    public static double PURPLE_MIN_RATIO = 0.7; // Red/Blue ratio for purple detection

    /**
     * TIMING CONSTANTS (milliseconds)
     */
    public static double ARTIFACT_SETTLE_TIME = 0.3; // Time for artifact to settle in slot
    public static double ROTATION_TIMEOUT = 2.0; // Max time for rotation
    public static double TRANSFER_TIMEOUT = 1.5; // Max time for transfer
    public static double HOPPER_KICK_DURATION = 0.5; // Time hopper runs to kick artifact
    public static double VERIFICATION_DELAY = 0.2; // Delay before verifying artifact state

    /**
     * CALIBRATION
     */
    public static double CALIBRATION_TIMEOUT = 3.0;
}