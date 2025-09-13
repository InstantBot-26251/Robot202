package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionProcessCommand extends CommandBase {
    ATVision vision;
    Telemetry telemetry;

    public VisionProcessCommand(ATVision vision) {
        this.vision = ATVision.getInstance();
    }

    @Override
    public void initialize() {
        telemetry.addLine("VisionProcessCommand initialized.");
        telemetry.update();
    }

    @Override
    public void execute() {

    }

}
