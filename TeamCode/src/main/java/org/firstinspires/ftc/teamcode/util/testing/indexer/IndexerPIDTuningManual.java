package org.firstinspires.ftc.teamcode.util.testing.indexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.robot.RobotMap;

/**
 * Simple PID Tuner for Indexer
 *
 * CONTROLS:
 * DPAD_UP    - Move to Entry Position
 * DPAD_DOWN  - Move to Transfer Position
 *
 * Y (hold)   - Manual control (left stick Y controls motor)
 *
 * Change positions in FTC Dashboard
 */
@Config
@TeleOp(name = "Indexer Manual", group = "Tuning")
public class IndexerPIDTuningManual extends OpMode {

    private Indexer indexer;
    private DcMotor rotorMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RobotMap.getInstance().init(hardwareMap);
        indexer = Indexer.getInstance();
        indexer.initialize();
        rotorMotor = RobotMap.getInstance().INDEXER_ROTOR;
        indexer.onTeleopInit();

        telemetry.update();
    }

    @Override
    public void start() {
        indexer.calibrateRotor();
    }

    @Override
    public void loop() {
        // Manual control with left stick
        double power = -gamepad1.left_stick_y * 0.5; // Limit to 50% power
        rotorMotor.setPower(power);

        // Display telemetry
        int currentPos = indexer.getCurrentPosition();
        int targetPos = indexer.getTargetPosition();
        int error = targetPos - currentPos;

        telemetry.addData("Mode", gamepad1.y ? "MANUAL" : "PID");
        telemetry.addLine();
        telemetry.addData("Current Position", indexer.getCurrentPosition());
        telemetry.addData("Target Position", indexer.getTargetPosition());
        telemetry.addData("Error", error);
        telemetry.addData("At Target?", indexer.atTargetPosition());
        telemetry.addLine();
        telemetry.update();
    }

}