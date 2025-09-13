// Maybe for review or teaching or smth like that


//package org.firstinspires.ftc.teamcode.vision;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.teamcode.vision.ATLivestream;
//
//import java.util.List;
//import java.util.Locale;
//
//@TeleOp(name = "ATLivestream TeleOp", group = "vision")
//public class ATTestingOpMode extends OpMode {
//
//    private static final String WEBCAM_NAME = "Webcam 1";
//    private static final Size CAM_RES = new Size(640, 480);
//    private static final int DASHBOARD_FPS = 12;
//
//    private FtcDashboard dashboard;
//    private MultipleTelemetry multiTelemetry;
//
//    private VisionPortal visionPortal;
//    private ATLivestream atLivestream;
//    private AprilTagProcessor aprilTagProcessor;
//
//    private Integer lockedId = null;
//    private Double lockedX = null;
//    private Double lockedY = null;
//    private String lockedMeta = null;
//
//    private boolean prevA = false;
//    private boolean prevB = false;
//
//    @Override
//    public void init() {
//        dashboard = FtcDashboard.getInstance();
//        multiTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        multiTelemetry.clearAll();
//        multiTelemetry.addLine("Initializing ATLivestream TeleOp...");
//        multiTelemetry.update();
//
//        try {
//            atLivestream = new ATLivestream();
//            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
//                    .addProcessor(atLivestream)
//                    .addProcessor(aprilTagProcessor)
//                    .setCameraResolution(CAM_RES)
//                    .build();
//
//
//            dashboard.startCameraStream((org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource) atLivestream, DASHBOARD_FPS);
//
//            multiTelemetry.addLine("VisionPortal created and dashboard stream started.");
//        } catch (Exception e) {
//            multiTelemetry.addLine("Initialization error: " + e.toString());
//        }
//
//        multiTelemetry.addLine(String.format(Locale.US, "Camera configured: %s", WEBCAM_NAME));
//        multiTelemetry.addLine("Press PLAY to start. gamepad1.a lock, gamepad1.b clear lock.");
//        multiTelemetry.update();
//    }
//
//    @Override
//    public void start() {
//        multiTelemetry.clearAll();
//        multiTelemetry.addLine("ATLivestream TeleOp started.");
//        multiTelemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        boolean a = gamepad1.a;
//        boolean b = gamepad1.b;
//
//        if (a && !prevA) {
//            if (aprilTagProcessor != null) {
//                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//                if (detections != null && !detections.isEmpty()) {
//                    AprilTagDetection best = detections.get(0);
//                    lockedId = best != null ? best.id : null;
//                    lockedMeta = (best != null && best.metadata != null) ? best.metadata.name : null;
//                    if (best != null && best.ftcPose != null) {
//                        lockedX = best.ftcPose.x;
//                        lockedY = best.ftcPose.y;
//                    } else {
//                        lockedX = null;
//                        lockedY = null;
//                    }
//                }
//            }
//        }
//        if (b && !prevB) {
//            lockedId = null;
//            lockedX = null;
//            lockedY = null;
//            lockedMeta = null;
//        }
//        prevA = a;
//        prevB = b;
//
//        multiTelemetry.clearAll();
//
//        multiTelemetry.addLine("ATLivestream TeleOp");
//        multiTelemetry.addLine("-------------------");
//
//        if (aprilTagProcessor == null) {
//            multiTelemetry.addLine("AprilTag processor not initialized.");
//        } else {
//            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//            if (detections == null || detections.isEmpty()) {
//                multiTelemetry.addLine("No live AprilTags detected.");
//            } else {
//                multiTelemetry.addData("Live Count", detections.size());
//                int shown = 0;
//                for (AprilTagDetection d : detections) {
//                    if (d == null) continue;
//                    shown++;
//                    if (shown > 3) break;
//
//                    multiTelemetry.addLine(String.format(Locale.US, "---- Tag %d ----", shown));
//                    multiTelemetry.addData("ID", d.id);
//                    String meta = (d.metadata != null) ? d.metadata.name : "Unknown";
//                    multiTelemetry.addData("Meta", meta);
//
//                    if (d.ftcPose != null) {
//                        multiTelemetry.addData("X (in)", String.format(Locale.US, "%.2f", d.ftcPose.x));
//                        multiTelemetry.addData("Y (in)", String.format(Locale.US, "%.2f", d.ftcPose.y));
//                        multiTelemetry.addData("Z (in)", String.format(Locale.US, "%.2f", d.ftcPose.z));
//                        multiTelemetry.addData("Range (in)", String.format(Locale.US, "%.2f", d.ftcPose.range));
//                        multiTelemetry.addData("Bearing (deg)", String.format(Locale.US, "%.2f", d.ftcPose.bearing));
//                    } else if (d.center != null) {
//                        multiTelemetry.addData("Center (px)", String.format(Locale.US, "%.0f, %.0f", d.center.x, d.center.y));
//                    }
//                }
//            }
//        }
//
//        multiTelemetry.addLine("");
//        multiTelemetry.addLine("Locked Tag (gamepad1.a to lock, b to clear):");
//        if (lockedId == null) {
//            multiTelemetry.addLine("No tag locked.");
//        } else {
//            multiTelemetry.addData("Locked ID", lockedId);
//            multiTelemetry.addData("Locked Meta", lockedMeta == null ? "Unknown" : lockedMeta);
//            if (lockedX != null && lockedY != null) {
//                multiTelemetry.addData("Locked X (in)", String.format(Locale.US, "%.2f", lockedX));
//                multiTelemetry.addData("Locked Y (in)", String.format(Locale.US, "%.2f", lockedY));
//            }
//        }
//
//        multiTelemetry.addLine("");
//        multiTelemetry.addData("Pipeline present?", atLivestream != null);
//        multiTelemetry.addData("Stream FPS (target)", DASHBOARD_FPS);
//        multiTelemetry.update();
//    }
//
//}
