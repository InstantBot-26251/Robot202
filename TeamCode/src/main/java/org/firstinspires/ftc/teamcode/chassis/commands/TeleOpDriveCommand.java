package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Drivetrain;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private DoubleSupplier fwd, str, rot;

    public TeleOpDriveCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
        this.drivetrain = Drivetrain.getInstance();
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setDriveVectors(fwd.getAsDouble(), str.getAsDouble(), rot.getAsDouble());
    }
}