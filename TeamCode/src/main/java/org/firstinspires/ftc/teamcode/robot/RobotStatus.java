package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public class RobotStatus {

    public enum Alliance {
        NONE, RED, BLUE,
    }

    public enum RobotState {
        DISABLED, AUTONOMOUS_INIT, AUTONOMOUS_ENABLED, TELEOP_INIT, TELEOP_ENABLED
    }

    // Add Motif State

    public static long delayMs = 0;
    public static Alliance alliance = Alliance.NONE;
    public static RobotState robotState = RobotState.DISABLED;
    public static Pose robotPose = new Pose();
    public static boolean liveView = false;

    public static void resetValues() {
        delayMs = 0;
        robotState = RobotState.DISABLED;
        robotPose = new Pose();
    }

    public static boolean isEnabled() {
        return robotState == RobotState.TELEOP_ENABLED || robotState == RobotState.AUTONOMOUS_ENABLED;
    }

    public static boolean isTeleop() {
        return robotState == RobotState.TELEOP_INIT || robotState == RobotState.TELEOP_ENABLED;
    }
}
