package org.firstinspires.ftc.teamcode.util.testing.indexer;

import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ROTOR_kD;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ROTOR_kI;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ROTOR_kP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.lynx.LynxModule;

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

    public static int ENTRY_POSITION = 0;
    public static int TRANSFER_POSITION = 80;
    public static int TICKS_PER_SLOT = (736 + 683 + 651 + 711 + 652 + 678 + 711 + 656) / 8;
    public static int POSITION_TOLERANCE = 20;

    public static double rotorTarget = 0;


    private Indexer indexer;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        indexer = Indexer.getInstance();
        indexer.initHardware();
        indexer.onTeleopInit();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
        if (gamepad1.a) {
            indexer.updatePID();
        }

        indexer.periodic();
            // Automatic PID control

            // DPAD_UP - Go to entry
            if (gamepad1.dpad_up) {
                indexer.rotateToNextSlot();
            }

            // DPAD_DOWN - Go to transfer
            if (gamepad1.dpad_down) {
                indexer.rotateToSlot(indexer.getRotorPosition() - TICKS_PER_SLOT);
            }
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
        telemetry.addData("kP", "%.5f", ROTOR_kP);
        telemetry.addData("kI", "%.5f", ROTOR_kI);
        telemetry.addData("kD", "%.5f", ROTOR_kD);
        telemetry.update();
    }


}