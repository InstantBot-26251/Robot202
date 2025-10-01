package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.Math.MathPM;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ShooterCommands {

    public static final Supplier<Command> SPIN_UP;
    public static final Supplier<Command> STOP;
    public static final Supplier<Command> SET_HOOD_ANGLE;
    public static final Supplier<Command> SHOOT_BALL;
    public static final Supplier<Command> AUTO_AIM_AND_SHOOT;

    static {
        Shooter shooter = Shooter.getInstance();
        Hood hood = Hood.getInstance();

        // Spin up shooter at fixed power
        SPIN_UP = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setState(Shooter.ShooterState.SHOOTING)),
                Commands.runOnce(() -> shooter.startShooting(1.0)) // full power for now
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

        // Fire one ball (assumes shooter already spinning & ready)
        SHOOT_BALL = () -> Commands.sequence(
                // TODO: add feeder command once you build feeder subsystem
                Commands.runOnce(() -> System.out.println("Firing ball..."))
        );

        // Automatically calculate hood angle + spin up + shoot
        AUTO_AIM_AND_SHOOT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    // Example: use math to compute angle
                    double distance = MathPM.inchesToMeters(5.0); // TODO: measure from vision
                    double heightDiff = 0.5;
                    double flywheelRPM = 3000;
                    double wheelDiameter = MathPM.inchesToMeters(0.1); // TODO: measure

                    double angle = MathPM.calculateAngleFromRPM(
                            flywheelRPM, wheelDiameter, 0.8, distance, heightDiff
                    );

                    hood.setAngle(angle);
                }),
                Commands.defer(SPIN_UP),    // spin shooter
                Commands.waitMillis(500),   // wait for shooter to reach speed (tune)
                Commands.defer(SHOOT_BALL), // release one ball
                Commands.defer(STOP)        // stop shooter
        );
    }
}
