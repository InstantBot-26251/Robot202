package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Usage:
 *   ATPipeline pipeline = new ATPipeline(hardwareMap, "Webcam 1");
 *   pipeline.setResolution(new Size(640, 480)); (optional, default used otherwise)
 *   pipeline.init();           (build but does not block)
 *   waitForStart();
 *   pipeline.start();          (start/activate portal if needed)
 *   // inside loop: pipeline.getDetections()
 *   pipeline.stop();            (when stopping OpMode)
 */

public class ATPipeline {
    private final HardwareMap hardwareMap;
    private final String webcamName;
    private Size cameraResolution = new Size(640, 480);

    private AprilTagDetection detection;

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTagProcessor = null;
    private boolean initialized = false;
    private boolean started = false;

    public boolean calibratedCamera = false;

    public ATPipeline(HardwareMap hardwareMap, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    public void setResolution(Size resolution) {
        this.cameraResolution = resolution;
    }

    // call in init

    public synchronized void init() {
        if (initialized) return;

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(cameraResolution)
                    .build();
        } catch (Exception e) {
            RobotLog.ee("ATPipeline", "Failed to create VisionPortal: %s", e.toString());
            visionPortal = null;
        }

        initialized = true;
    }

  // start pipeline
    public synchronized void start() {
        if (!initialized) init();
        if (started) return;
        started = true;
    }

    // detections
    public synchronized List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return Collections.emptyList();

        try {
            List<AprilTagDetection> raw = aprilTagProcessor.getDetections();
            if (raw == null || raw.isEmpty()) return Collections.emptyList();

            return new ArrayList<>(raw);
        } catch (Exception e) {
            RobotLog.ee("ATPipeline", "Error getting detections: %s", e.toString());
            return Collections.emptyList();
        }
    }

    // GETTERS
    public double getID() {
        return detection.id;
    }

    public double getX() {
        return detection.ftcPose.x;
    }

    public double getY() {
        return detection.ftcPose.y;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public boolean isStarted() {
        return started;
    }
}
