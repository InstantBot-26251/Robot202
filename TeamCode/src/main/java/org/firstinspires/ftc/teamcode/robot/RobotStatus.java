package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

import java.util.concurrent.atomic.AtomicInteger;

public class RobotStatus {

    public enum Alliance {
        NONE, RED, BLUE,
    }

    public enum RobotState {
        DISABLED, AUTONOMOUS_INIT, AUTONOMOUS_ENABLED, TELEOP_INIT, TELEOP_ENABLED
    }

    public static long delayMs = 0;
    public static Alliance alliance = Alliance.NONE;
    public static RobotState robotState = RobotState.DISABLED;
    public static Pose robotPose = new Pose();
    public static boolean liveView = false;

    private final AtomicInteger currentMotif = new AtomicInteger(0); // 0 = UNKNOWN, 1..n for motifs
    private volatile int[] ballIndexOrder = new int[0]; // e.g. {2,1,3}

    public static boolean isEnabled() {
        return robotState == RobotState.TELEOP_ENABLED || robotState == RobotState.AUTONOMOUS_ENABLED;
    }

    public static boolean isTeleop() {
        return robotState == RobotState.TELEOP_INIT || robotState == RobotState.TELEOP_ENABLED;
    }
}
