package org.firstinspires.ftc.teamcode.util.testing.shooter;

import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.WHEEL_DIAMETER;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fy;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.math.MathPM;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Vision Distance Test")
public class VisionDistanceTester extends OpMode {

    private static final int DASHBOARD_FPS = 10;

    private FtcDashboard dashboard;
    private MultipleTelemetry multiTelemetry;

    private VisionPortal visionPortal;
    private ATLivestream atLivestream;
    private AprilTagProcessor aprilTagProcessor;

    private Hood hood;

    private boolean hoodLocked = false; // Is hood currently locked at a set angle
    private double lockedAngle = 0;

    private boolean prevA = false; // Button state for locking
    private boolean prevB = false; // Button state for unlocking


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotMap.getInstance().init(hardwareMap);

        // Initialize hood
        hood = Hood.getInstance();
        hood.onTeleopInit();


        multiTelemetry.clearAll();
        multiTelemetry.addLine("Initializing AprilTag Distance OpMode...");
        multiTelemetry.update();
        try {
            atLivestream = new ATLivestream();

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(atLivestream)
                    .addProcessor(aprilTagProcessor)
                    .build();

            // Stream to dashboard for visualization
            dashboard.startCameraStream((org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource) atLivestream, DASHBOARD_FPS);

            multiTelemetry.addLine("VisionPortal initialized successfully.");
        } catch (Exception e) {
            multiTelemetry.addLine("Initialization error: " + e.toString());
        }

        multiTelemetry.addLine("Press PLAY to begin detection.");
        multiTelemetry.update();
    }

    @Override
    public void start() {
        multiTelemetry.clearAll();
        multiTelemetry.addLine("AprilTag Distance OpMode started.");
        multiTelemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;


        multiTelemetry.clearAll();
        multiTelemetry.addLine("AprilTag Distance Measurement");
        multiTelemetry.addLine("--------------------------------");

        double calculatedAngle = -1;
        double rangeInches = -1;

        if (aprilTagProcessor != null) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                if (tag != null && tag.ftcPose != null) {
                    rangeInches = tag.ftcPose.range;
                    if (tag != null && tag.ftcPose != null) {
                        rangeInches = tag.ftcPose.range;

                        double exitVelocity = MathPM.calculateExitVelocity(
                                FLYWHEEL_RPM, WHEEL_DIAMETER
                        );

                        calculatedAngle = MathPM.calculateLaunchAngle(
                                MathPM.inchesToMeters(rangeInches), exitVelocity, MathPM.inchesToMeters(41.45)
                        );

                        if (calculatedAngle > 0) {
                            // clamp to 0–70
                            calculatedAngle = Math.max(0, Math.min(70, calculatedAngle));
                        }                    }
                }
            }

            // Handle button presses
            if (a && !prevA && calculatedAngle > 0) {
                hoodLocked = true;
                lockedAngle = calculatedAngle;
                hood.setAngle(lockedAngle);
            }
            if (b && !prevB) {
                hoodLocked = false; // unlock hood
            }
            prevA = a;
            prevB = b;

            // Update hood telemetry (locked or unlocked)
            if (hoodLocked) {
                multiTelemetry.addData("Hood Status", "LOCKED");
                multiTelemetry.addData("Locked Angle (deg)", "%.2f", lockedAngle);
            } else {
                multiTelemetry.addData("Hood Status", "UNLOCKED");
                multiTelemetry.addData("Current Hood Angle (deg)", "%.2f", hood.getCurrentAngle());
            }

            multiTelemetry.addData("Calculated Angle (deg)", "%.2f", calculatedAngle);
            multiTelemetry.addData("Range (in)", "%.2f", rangeInches);

            multiTelemetry.update();
        }
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();

        multiTelemetry.addLine("AprilTag Distance OpMode stopped.");
        multiTelemetry.update();
    }
}
