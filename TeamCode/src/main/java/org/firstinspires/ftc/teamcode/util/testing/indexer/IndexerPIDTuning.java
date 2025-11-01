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
@TeleOp(name = "Simple Indexer Tuner", group = "Tuning")
public class IndexerPIDTuning extends OpMode {

    // Editable in FTC Dashboard
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    public static int ENTRY_POSITION = 0;
    public static int TRANSFER_POSITION = 80;
    public static int TICKS_PER_SLOT = 680;
    public static int POSITION_TOLERANCE = 20;

    private Indexer indexer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RobotMap.getInstance().init(hardwareMap);
        indexer = Indexer.getInstance();
        indexer.initHardware();
        indexer.onTeleopInit();

        telemetry.addLine("Simple Indexer Tuner Ready!");
        telemetry.addLine();
        telemetry.addLine("DPAD_UP: Go to Entry");
        telemetry.addLine("DPAD_DOWN: Go to Transfer");
        telemetry.addLine("Hold Y + Left Stick: Manual Control");
        telemetry.update();
    }

    @Override
    public void start() {
        indexer.calibrateRotor();
    }

    @Override
    public void loop() {
        indexer.periodic();
            // Automatic PID control

            // DPAD_UP - Go to entry
            if (gamepad1.dpad_up) {
                indexer.rotateToTransferPosition();
            }

            // DPAD_DOWN - Go to transfer
            if (gamepad1.dpad_down) {
                indexer.rotateToSlot(0);
            }

            // Run periodic (updates PID)
            indexer.periodic();

        // Display telemetry
        int currentPos = indexer.getRotorPosition();
        int targetPos = indexer.getRotorTarget();
        int error = targetPos - currentPos;

        telemetry.addData("Mode", gamepad1.y ? "MANUAL" : "PID");
        telemetry.addLine();
        telemetry.addData("Current Position", currentPos);
        telemetry.addData("Target Position", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("At Target?", indexer.isAtTargetPosition());
        telemetry.addLine();
        telemetry.addData("Entry Position", ENTRY_POSITION);
        telemetry.addData("Transfer Position", TRANSFER_POSITION);
        telemetry.addData("Ticks Per Slot", TICKS_PER_SLOT);
        telemetry.addLine();
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kI", "%.5f", kI);
        telemetry.addData("kD", "%.5f", kD);
        telemetry.update();
    }


}