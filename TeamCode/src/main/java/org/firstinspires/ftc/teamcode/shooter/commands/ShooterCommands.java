package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.shooter.Constants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
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
                Commands.runOnce(() -> shooter.setState(Shooter.ShooterState.SHOOTING)),
                Commands.runOnce(() -> shooter.startShooting()) // full power for now
        );

        // Stop shooter
        STOP = () -> Commands.sequence(
                Commands.runOnce(shooter::stopShooting),
                Commands.runOnce(() -> shooter.setState(Shooter.ShooterState.RESTING))
        );

        // Adjust hood angle (30 deg for testing)
        SET_HOOD_ANGLE = () -> Commands.sequence(
                Commands.runOnce(() -> hood.setAngle(30.0))
        );

        // Spin up shooter for rejection - ideally half power
        SPIN_SLOW = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setState(Shooter.ShooterState.REJECTION)),
                Commands.runOnce(() -> shooter.startShooting(0.5)) // TODO: Test and Tune (TAT)
        );

        // Automatically calculate hood angle + spin up + shoot
        AUTO_AIM = () -> Commands.sequence(
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
                    double heightDiff = 0.5; // TODO: measure
                    double flywheelRPM = 3000; // TODO: measure
                    double wheelDiameter = MathPM.inchesToMeters(0.1); // TODO: measure

                    double angle = MathPM.calculateAngleFromRPM(
                            flywheelRPM, wheelDiameter, 0.8, distance, heightDiff
                    ); // TODO: fudge the values (TAT)

                    hood.setAngle(angle);

                    Commands.runOnce(() -> shooter.startShooting(1.0));
                })
        );
    }
    }
