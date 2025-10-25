package org.firstinspires.ftc.teamcode.util.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.hardware.InstantServo;

@TeleOp(name = "Hood Calibration", group = "Calibration")
public class HoodTestingOpMode extends LinearOpMode {

    private Hood hood;
    private InstantServo hoodServo;
    private double testServoPos = 0.5;

    @Override
    public void runOpMode() {
        // Initialize
        RobotMap.getInstance().init(hardwareMap);
        hood = Hood.getInstance();
        hood.onTeleopInit();


        telemetry.addLine("=== HOOD CALIBRATION MODE ===");
        telemetry.addLine("Use DPAD to adjust servo position");
        telemetry.addLine("Measure physical angle with protractor");
        telemetry.addLine("Update constants in Hood.java");
        telemetry.addLine();
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual servo position control
            if (gamepad1.dpad_up) {
                hood.setAngle(70);
                sleep(100);

            }


            if (gamepad1.dpad_down) {
                testServoPos -= 0.01;
                testServoPos = Math.max(0.0, testServoPos);
                hood.setRawServoPosition(testServoPos);
                sleep(100);
            }

            if (gamepad1.dpad_right) {
                testServoPos += 0.1;
                testServoPos = Math.min(1.0, testServoPos);
                hood.setRawServoPosition(testServoPos);
                sleep(100);
            }

            if (gamepad1.dpad_left) {
                testServoPos -= 0.1;
                testServoPos = Math.max(0.0, testServoPos);
                hood.setRawServoPosition(testServoPos);
                sleep(100);
            }

            // Test angle mapping
            if (gamepad1.a) {
                hood.calibrationTest(testServoPos);
                sleep(200);
            }

            // Run calibration sequence
            if (gamepad1.b) {
                hood.runCalibrationSequence();
                sleep(200);
            }

            // Safe position
            if (gamepad1.x) {
                hood.setSafePosition();
                testServoPos = hood.getCurrentServoPosition();
                sleep(200);
            }

            // Print info
            if (gamepad1.y) {
                hood.printCalibrationInfo();
                sleep(200);
            }

            // Display current state
            telemetry.addLine("=== CURRENT STATE ===");
            telemetry.addData("Servo Position", "%.3f", testServoPos);
            telemetry.addData("Servo Position", "%.3f", hood.getCurrentServoPosition());
            telemetry.addData("Calculated Angle", "%.1f°", hood.getCurrentAngle());
            telemetry.addLine();
            telemetry.addLine("=== MEASURE PHYSICAL ANGLE ===");
            telemetry.addLine("Use protractor/angle finder");
            telemetry.addLine("Record: Servo %.3f = ___°");
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("DPAD UP/DOWN: ±0.01");
            telemetry.addLine("DPAD L/R: ±0.1");
            telemetry.addLine("A: Test mapping");
            telemetry.addLine("B: Calibration sequence");
            telemetry.addLine("X: Safe position");
            telemetry.addLine("Y: Show info");

            hood.periodic();
            telemetry.update();
        }
    }
}