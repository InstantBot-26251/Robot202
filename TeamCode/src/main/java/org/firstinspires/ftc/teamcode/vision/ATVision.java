
package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Unnamed;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;

public class ATVision extends SubsystemTemplate {
    private Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final ATVision INSTANCE = new ATVision();

    public static ATVision getInstance() {
        return INSTANCE;
    }

    // INITIALIZE

    @Override
    public void onAutonomousInit() {
        telemetry = Unnamed.getInstance().getTelemetry();

        makeProcessor();
        makePortal();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Unnamed.getInstance().getTelemetry();

        makeProcessor();
        makePortal();
    }

    private void makeProcessor() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
    }

    private void makePortal() {
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(RobotStatus.liveView)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTag);

        if (RobotStatus.liveView) {
            portalBuilder.enableLiveView(true);
            portalBuilder.addProcessor(new ATLivestream());
        }

        visionPortal = portalBuilder.build();
    }

    // GETTERS

    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // SETTERS

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void closePortal() {
        visionPortal.close();
    }
}
