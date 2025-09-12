package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class ATDetection extends LinearOpMode {
    // Camera Calibration Values

    public double fx;
    public double fy;

    public double cx;
    public double cy;

    public final boolean calibratedCamera = false;

    private AprilTagProcessor aprilTagProcessor;
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 840))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                int id = detection.id;
                double x = detection.ftcPose.x; // inches
                double y = detection.ftcPose.y;
                double pitch = detection.ftcPose.pitch;
                double yaw = detection.ftcPose.yaw;
                double roll = detection.ftcPose.roll;
                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing; // degrees

                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.update();

            sleep(20);
        }
    }
}