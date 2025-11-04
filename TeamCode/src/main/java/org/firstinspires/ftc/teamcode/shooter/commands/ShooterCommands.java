package org.firstinspires.ftc.teamcode.shooter.commands;

import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.DEFAULT_HEIGHT_DIFF;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.DRAG_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.FLYWHEEL_SPINUP_TIME;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.FULL_POWER;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.TEST_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.WHEEL_DIAMETER;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.ShooterState;
import org.firstinspires.ftc.teamcode.util.math.MathPM;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



import java.util.function.Supplier;

public class ShooterCommands {

    public static final Supplier<Command> SPIN_UP;
    public static final Supplier<Command> STOP;
    public static final Supplier<Command> SPIN_SLOW;
    public static final Supplier<Command> SET_HOOD_ANGLE;
    public static final Supplier<Command> AUTO_AIM;
    public static final Supplier<Command> AUTO_AIM_AND_SHOOT; // This is what Enigma needs!
    public static final Supplier<Command> SHOOT;
    public static final Supplier<Command> REJECT;




    static {
        Shooter shooter = Shooter.getInstance();
        Hood hood = Hood.getInstance();

        // Spin up shooter at fixed power
        SPIN_UP = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setState(ShooterState.SHOOTING)),
                Commands.runOnce(() -> shooter.startShooting(1)) // full power for now
        );

        // Stop shooter
        STOP = () -> Commands.sequence(
                Commands.runOnce(shooter::stopShooting),
                Commands.runOnce(() -> shooter.setState(ShooterState.RESTING))
        );

        // Adjust hood angle (30 deg for testing)
        SET_HOOD_ANGLE = () -> Commands.sequence(
                Commands.runOnce(() -> hood.setAngle(30.0))
        );

        // Spin up shooter for rejection - ideally half power
        SPIN_SLOW = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setState(ShooterState.REJECTION)),
                Commands.runOnce(() -> shooter.startShooting(0.75)) // TODO: Test and Tune (TAT)
        );


        // Automatically calculate hood angle based on AprilTag distance and spin up
        AUTO_AIM = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    double distance = getTargetDistance();

                    if (distance == -1) {
                        // No valid target detected, use default angle
                        Log.i("Warning", ": No valid target detected for auto-aim");
                        String telemetry = "Not able to shoot";
                        hood.setAngle(0);
                    } else if (distance > 0) {
                        double angle = MathPM.calculateAngleFromRPM(
                                FLYWHEEL_RPM,
                                WHEEL_DIAMETER,
                                DRAG_COEFFICIENT,
                                distance,
                                DEFAULT_HEIGHT_DIFF
                        );
                        double fudge = 3.75;
                        hood.setAngle(angle + fudge);
                    }
                })
        );

        // Automatically calculate hood angle + spin up + shoot
        AUTO_AIM_AND_SHOOT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    double distance = 0;

                    // Get detections from ATVision
                    if (ATVision.getInstance() != null
                            && !ATVision.getInstance().getDetections().isEmpty()) {

                        // Grab the first detected tag
                        AprilTagDetection best = ATVision.getInstance().getDetections().get(0);

                        if (best != null && best.ftcPose != null) {
                            double dist = best.ftcPose.range;

                            // Convert to meters
                            distance = MathPM.inchesToMeters(dist);

                        }
                    }
                    double heightDiff = 0.5;
                    double flywheelRPM = FLYWHEEL_RPM; // TODO: measure
                    double wheelDiameter = MathPM.inchesToMeters(0.1); // TODO: measure

                    double angle = MathPM.calculateAngleFromRPM(
                            flywheelRPM, wheelDiameter, 0.8, distance, heightDiff
                    ); // TODO: fudge the values (TAT)

                    hood.setAngle(angle);

                    Commands.runOnce(() -> shooter.startShooting(1.0));
                })
        );

        SHOOT = () -> Commands.sequence(
                SPIN_UP.get(),
                Commands.waitSeconds(FLYWHEEL_SPINUP_TIME)
                // TODO: Add actual shooting mechanism trigger here when ready
        );

        REJECT = () -> SPIN_SLOW.get();

    }

    /**
     * Gets the distance to the target using AprilTag detection
     * @return distance in meters, or 0 if no valid detection
     */
    private static double getTargetDistance() {
        ATVision vision = ATVision.getInstance();

        if (vision == null || vision.getDetections().isEmpty()) {
            return 0;
        }

        AprilTagDetection bestTag = vision.getDetections().get(0);

        if (bestTag == null || bestTag.ftcPose == null) {
            return 0;
        }

        return MathPM.inchesToMeters(bestTag.ftcPose.range);
    }

    }
