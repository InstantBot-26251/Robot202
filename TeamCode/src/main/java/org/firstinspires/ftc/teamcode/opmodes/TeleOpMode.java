package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.shooter.Hood;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends LinearOpMode {
    private final Enigma enigma = Enigma.getInstance();
    private boolean lastStart = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // INIT
        enigma.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);

        // INIT_LOOP
        while (opModeInInit()) {
            if (gamepad1.square != lastStart && gamepad1.start) {
                switch (RobotStatus.alliance) {
                    case NONE:
                    case RED:
                        RobotStatus.alliance = BLUE;
                        break;
                    case BLUE:
                        RobotStatus.alliance = RED;
                        break;
                }
            }
            lastStart = gamepad1.start;

            telemetry.addData("Alliance", RobotStatus.alliance);
            telemetry.addData("Status", RobotStatus.robotState);
            telemetry.update();

        }

        // LOOP
        while (opModeIsActive()) {
            enigma.periodic();
        }

        // END
        enigma.disabledInit();
    }
}