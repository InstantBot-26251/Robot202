package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands.AUTO_AIM;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.DEFAULT_HEIGHT_DIFF;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.DRAG_COEFFICIENT;
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
import org.firstinspires.ftc.teamcode.chassis.PedroChassis;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.ShooterState;
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

//    private PedroChassis chassis;

    double x, y, rx;

    private static final int DASHBOARD_FPS = 10;

    private FtcDashboard dashboard;
    private MultipleTelemetry multiTelemetry;

    private VisionPortal visionPortal;
    private ATLivestream atLivestream;
    private AprilTagProcessor aprilTagProcessor;

    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;

    private boolean hoodLocked = false; // Is hood currently locked at a set angle
    private double lockedAngle = 0;

    private boolean prevRIGHT = false; // Button state for locking
    private boolean prevLEFT = false; // Button state for unlocking

    private static final int SLOT_0 = 0;
    private static final int SLOT_1 = 680;
    private static final int SLOT_2 = 1360;

    private static final double MANIP_RESPONSE = 1.9;
    private static final double DRIVER_STRAIGHT_RESPONSE = 1.75;
    private static final double DRIVER_TURN_RESPONSE = 1.3;

    private ATVision vision;

//    private static final int[] SLOT_POSITIONS = {SLOT_0, SLOT_1, SLOT_2};

//    private boolean wasManual = false;   // Track manual → auto transition

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotMap.getInstance().init(hardwareMap);

        chassis = new Chassis(hardwareMap);

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

            multiTelemetry.addLine("VisionPortal initialized successfully.");
        } catch (Exception e) {
            multiTelemetry.addLine("Initialization error: " + e.toString());
        }


    }

    @Override
    public void loop() {
        boolean RIGHT = gamepad1.right_bumper;
        boolean LEFT = gamepad1.left_bumper;


        y = applyResponseCurve(gamepad1.left_stick_y, DRIVER_STRAIGHT_RESPONSE);
        x = applyResponseCurve(gamepad1.left_stick_x, DRIVER_STRAIGHT_RESPONSE);
        rx = -applyResponseCurve(gamepad1.right_stick_x, DRIVER_TURN_RESPONSE);


        double calculatedAngle = -1;
        double rangeInches = -1;
        double exitVelocity = -1;

        if (aprilTagProcessor != null) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                if (tag != null && tag.ftcPose != null) {
                    rangeInches = tag.ftcPose.range;

                    exitVelocity = MathPM.calculateExitVelocity(
                            FLYWHEEL_RPM, WHEEL_DIAMETER
                    ) * 0.75;

                    double heightDiffInches = 41.45; // Positive = target is higher

                    calculatedAngle = MathPM.calculateLaunchAngle(
                            MathPM.inchesToMeters(rangeInches),
                            exitVelocity,
                            MathPM.inchesToMeters(heightDiffInches)
                    );

                    if (calculatedAngle > 0) {
                        // Apply fixed offset
                        double ANGLE_OFFSET = 0.0;  // ← TUNE THIS (start with 5-10)
                        calculatedAngle = calculatedAngle - ANGLE_OFFSET;

                        // Clamp to safe range
                        calculatedAngle = Math.max(5, Math.min(65, calculatedAngle));

                        // Detailed telemetry
                        telemetry.addData("Range", "%.1f in", rangeInches);
                        telemetry.addData("Exit Vel", "%.2f m/s", exitVelocity);
                        telemetry.addData("Height", "%.1f in", heightDiffInches);
                        telemetry.addData("Calc Angle", "%.2f°", calculatedAngle);
                        telemetry.addData("Hood Angle", "%.2f°", hood.getCurrentAngle());
                    }


                    // CRITICAL DEBUG INFO
                    telemetry.addLine("=== AUTO-AIM DIAGNOSTICS ===");
                    telemetry.addData("Range", "%.1f in (%.2f m)",
                            rangeInches, MathPM.inchesToMeters(rangeInches));
                    telemetry.addData("Height Diff", "%.1f in (%.2f m)",
                            heightDiffInches, MathPM.inchesToMeters(heightDiffInches));
                    telemetry.addData("Exit Velocity", "%.2f m/s", exitVelocity);
                    telemetry.addData("Flywheel RPM", FLYWHEEL_RPM);
                    telemetry.addData("Wheel Diameter", "%.4f m", WHEEL_DIAMETER);

                    telemetry.addLine("---");
                    telemetry.addData("RAW Calc Angle", "%.2f°", calculatedAngle);

                    telemetry.addLine("---");
                    telemetry.addData("Current Hood", "%.2f°", hood.getCurrentAngle());
                    telemetry.addData("Current Servo", "%.3f", hood.getCurrentServoPosition());
                }

            }
        }
//
//        if (RIGHT && !prevRIGHT && calculatedAngle > 0) {
//            hoodLocked = true;
//            lockedAngle = calculatedAngle;
//            hood.setAngle(lockedAngle);
//        }
//        if (LEFT && !prevLEFT) {
//            hoodLocked = false; // unlock hood
//        }
//        prevRIGHT = RIGHT;
//        prevLEFT = LEFT;


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



        if (gamepad2.b) {
            shooter.setState(ShooterState.INTAKING);
            shooter.startShooting1(0.25);
            shooter.startShooting2(0.25);
        }

        if (gamepad2.dpad_up) {
            indexer.startHopper();
        }

        if (gamepad2.dpad_down) {
            indexer.stopHopper();
        }


        if (gamepad2.x) {
            hood.hoodServo.setPosition(0.5);
            shooter.setState(ShooterState.SHOOTING);
            shooter.startShooting1(-0.9);
            shooter.startShooting2(-0.9);
        }

        if (gamepad2.y) {
            shooter.setState(ShooterState.RESTING);
            shooter.startShooting2(0);
            shooter.startShooting1(0);
        }

        if (gamepad2.dpad_left) {
            hood.setAngle(calculatedAngle);
        }

        if (gamepad2.right_bumper) {
            indexer.rotateToNextSlot();
        }

        if (gamepad2.left_bumper) {
            indexer.rotateToPreviousSlot();
        }

        if (gamepad2.a) {
            hood.hoodServo.setPosition(0.95);
            shooter.startShooting1(-0.63);
            shooter.startShooting2(-0.63);
        }

        // DRIVER CONTROLS

        if (gamepad1.right_bumper) {
            chassis.enableSlowMode();
        } else {
            chassis.disableSlowMode();
        }
        // RESET HEADING
        if (gamepad1.y) {
            chassis.resetYaw();
        }


//        if (gamepad1.y){
//            chassis.resetYaw();
//        }

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
        telemetry.addData("Shooter State", shooter.getState());
//        telemetry.addData("Motor Velocity (fl), ", chassis.fl.getVelocity());
//        telemetry.addData("Motor Velocity (fr), ", chassis.fr.getVelocity());
//        telemetry.addData("Motor Velocity (bl), ", chassis.bl.getVelocity());
//        telemetry.addData("Motor Velocity (br), ", chassis.br.getVelocity());
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