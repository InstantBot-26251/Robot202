package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Unnamed;
import org.firstinspires.ftc.teamcode.chassis.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Unnamed;

public class FollowPathCommand extends CommandBase {
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private final Path path;
    private final double maxPower;

    public FollowPathCommand(Path path) {
        this(path, 1.0);
    }

    public FollowPathCommand(Path path, double maxPower) {
        telemetry = Unnamed.getInstance().getTelemetry();
        drivetrain = Drivetrain.getInstance();
        this.path = path;
        this.maxPower = maxPower;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setMaxPower(maxPower);
        drivetrain.followPath(path);
    }

    @Override
    public void execute() {
        Pose pose = drivetrain.getPoseEstimate();
        telemetry.addLine();
        telemetry.addData("Path Running", drivetrain.isBusy());
        telemetry.addData("Pose", pose.getX() + ", " + pose.getY() + ", " + pose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.isBusy();
    }
}