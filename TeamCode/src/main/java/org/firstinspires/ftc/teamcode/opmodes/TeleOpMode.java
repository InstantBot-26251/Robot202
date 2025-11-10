package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands.AUTO_AIM;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.DEFAULT_HEIGHT_DIFF;
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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.math.MathPM;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "TeleOp", group = "opmodes")
public class TeleOpMode extends OpMode {
    Chassis chassis;
    double x, y, rx;

    private static final int DASHBOARD_FPS = 10;

    private FtcDashboard dashboard;

    private VisionPortal visionPortal;
    private ATLivestream atLivestream;
    private AprilTagProcessor aprilTagProcessor;

    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;

    private static final int SLOT_0 = 0;
    private static final int SLOT_1 = 680;
    private static final int SLOT_2 = 1360;

    private static final double MANIP_RESPONSE = 1.5;
    private static final double DRIVER_RESPONSE = 1.5;

    private ATVision vision;

//    private static final int[] SLOT_POSITIONS = {SLOT_0, SLOT_1, SLOT_2};

//    private boolean wasManual = false;   // Track manual → auto transition

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        vision = ATVision.getInstance();

        indexer.initHardware();

        indexer.onTeleopInit();
        shooter.onTeleopInit();
        hood.onTeleopInit();

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


            telemetry.addLine("VisionPortal initialized successfully.");
        } catch (Exception e) {
            telemetry.addLine("Initialization error: " + e.toString());
        }


        chassis = new Chassis(hardwareMap);
    }

    @Override
    public void loop() {
        y = -applyResponseCurve(gamepad1.left_stick_y, DRIVER_RESPONSE);
        x = -applyResponseCurve(gamepad1.left_stick_x, DRIVER_RESPONSE);
        rx = applyResponseCurve(gamepad1.right_stick_x, DRIVER_RESPONSE);

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

                        calculatedAngle = MathPM.calculateAngleFromRPM(
                                FLYWHEEL_RPM, WHEEL_DIAMETER, 0.8, getTargetDistance(), DEFAULT_HEIGHT_DIFF
                        );

                        if (calculatedAngle > 0) {
                            // clamp to 0–70
                            calculatedAngle = Math.max(0, Math.min(70, calculatedAngle));
                        }                   }}}}
//        boolean manualHeld = gamepad2.left_bumper;


//        if (manualHeld) {
//            wasManual = true;

            double power = -applyResponseCurve(gamepad2.left_stick_y * 0.7, MANIP_RESPONSE);
            indexer.setPower(power);

//        } else {
//
//            if (wasManual) {
//                wasManual = false;
//                snapIndexerToNearestSlot();
//            }

            hood.hoodServo.setPosition(gamepad2.right_stick_y);


            if (gamepad2.a) {
                indexer.rotateToNextSlot();
            }

            if (gamepad2.b) {
                ShooterCommands.REJECT.get();
            }

            if (gamepad2.dpad_up) {
                indexer.startHopper();
            }

            if (gamepad2.dpad_down) {
                indexer.stopHopper();
            }


        if (gamepad2.x) {
            ShooterCommands.SPIN_UP.get();
        }

        if (gamepad2.y) {
            ShooterCommands.STOP.get();
        }


        if (gamepad2.dpad_left) {
            hood.setAngle(calculatedAngle);
        }

        if (gamepad2.dpad_right) {
            hood.setDefaultPosition();
        }

        // DRIVER CONTROLS
        if (gamepad1.y){
            chassis.resetYaw();
        }

        chassis.drive(x, y, rx);

        // Always run periodic
        indexer.periodic();
        shooter.periodic();
        hood.periodic();


        // TELEMETRY
//        telemetry.addData("Manual?", manualHeld);
        telemetry.addData("Pos", indexer.getRotorPosition());
        telemetry.addData("Slot", indexer.getCurrentSlot());
        telemetry.addData("Hood Angle", hood.getCurrentAngle());
        telemetry.addData("Hood Position", hood.hoodServo.getPosition());
        telemetry.addData("Motor Velocity (fl), ", chassis.fl.getVelocity());
        telemetry.addData("Motor Velocity (fr), ", chassis.fr.getVelocity());
        telemetry.addData("Motor Velocity (bl), ", chassis.bl.getVelocity());
        telemetry.addData("Motor Velocity (br), ", chassis.br.getVelocity());
        telemetry.update();
    }


//    private void snapIndexerToNearestSlot() {
//        int current = indexer.getRotorPosition();
//
//        int nearestSlot = 0;
//        int smallestDiff = Math.abs(current - SLOT_POSITIONS[0]);
//
//        for (int i = 1; i < SLOT_POSITIONS.length; i++) {
//            int diff = Math.abs(current - SLOT_POSITIONS[i]);
//            if (diff < smallestDiff) {
//                smallestDiff = diff;
//                nearestSlot = i;
//            }
//        }
//
//        indexer.rotateToSlot(nearestSlot);
//    }

    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }

    /**
     * Gets the distance to the target using AprilTag detection
     * @return distance in meters, or 0 if no valid detection
     */
    private static double getTargetDistance() {
        ATVision vision = ATVision.getInstance();

        if (vision == null || vision.getDetections().isEmpty()) {
            return 0;
        }

        AprilTagDetection bestTag = vision.getDetections().get(0);

        if (bestTag == null || bestTag.ftcPose == null) {
            return 0;
        }

        return MathPM.inchesToMeters(bestTag.ftcPose.range);
    }
}
