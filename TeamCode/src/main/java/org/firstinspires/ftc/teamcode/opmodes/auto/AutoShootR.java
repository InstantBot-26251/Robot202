package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.MOVE_TIME;
import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.SHOOT_TIME;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.shooter.constants.Constants.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_cy;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.arducam_fy;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterState;
import org.firstinspires.ftc.teamcode.util.math.MathPM;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Shoot 3 RED")
@Config
public class AutoShootR extends OpMode {
    DcMotorEx fl, fr, bl, br;

    AutoState currentState = AutoState.MOVETOSHOOTINGPOSITION;

    private ElapsedTime runtime = new ElapsedTime();

    private ATVision vision;

    Chassis chassis;

    private static final int DASHBOARD_FPS = 10;

    private FtcDashboard dashboard;

    private VisionPortal visionPortal;
    private ATLivestream atLivestream;
    private AprilTagProcessor aprilTagProcessor;

    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;

    private boolean hasStartedShooting = false;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);

        chassis = new Chassis(hardwareMap);


        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        vision = ATVision.getInstance();

        indexer.initHardware();
        indexer.onAutonomousInit();
        shooter.onAutonomousInit();
        hood.onAutonomousInit();

//        try {
//            // atLivestream = new ATLivestream();
//
//            aprilTagProcessor = new AprilTagProcessor.Builder()
//                    .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
//                    .build();
//
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
//                    .setCameraResolution(new Size(640, 480))
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                    //.addProcessor(atLivestream)
//                    .addProcessor(aprilTagProcessor)
//                    .build();
//            // Stream to dashboard for visualization
//            //dashboard.startCameraStream((org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource) atLivestream, DASHBOARD_FPS);
//
//
//            telemetry.addLine("VisionPortal initialized successfully.");
//        } catch (Exception e) {
//            telemetry.addLine("Initialization error: " + e.toString());
//        }
    }

    @Override
    public void init_loop() {
        runtime.reset();
        telemetry.addData("Current State", currentState);
    }

    @Override
    public void start() {
        runtime.reset();
        currentState = AutoState.MOVETOSHOOTINGPOSITION;
    }

    @Override
    public void loop() {
        shooter.periodic();
        indexer.periodic();
        hood.periodic();
        vision.periodic();

        double calculatedAngle = -1;
        double rangeInches = -1;
//        if (aprilTagProcessor != null) {
//            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//            if (detections != null && !detections.isEmpty()) {
//                AprilTagDetection tag = detections.get(0);
//                if (tag != null && tag.ftcPose != null) {
//                    rangeInches = tag.ftcPose.range;
//                    if (tag != null && tag.ftcPose != null) {
//                        rangeInches = tag.ftcPose.range;
//
//                        double exitVelocity = MathPM.calculateExitVelocity(
//                                FLYWHEEL_RPM, WHEEL_DIAMETER
//                        );
//
//                        calculatedAngle = MathPM.calculateLaunchAngle(
//                                MathPM.inchesToMeters(rangeInches), exitVelocity, MathPM.inchesToMeters(41.45)
//                        );
//
//                        if (calculatedAngle > 0) {
//                            // clamp to 0–70
//                            calculatedAngle = Math.max(0, Math.min(70, calculatedAngle));
//                        }                   }
//                }
//            }
//        }

        double elapsedTime = runtime.seconds();

        switch (currentState) {
            case MOVETOSHOOTINGPOSITION:
                if (elapsedTime < MOVE_TIME) {
                    chassis.moveBackward();
                } else {
                    chassis.stopMotors();
                    runtime.reset();
                    currentState = AutoState.SHOOTARTIFACT1;
                }
                break;

            case SHOOTARTIFACT1:
                if (!hasStartedShooting) {
                    // start shooting
                    chassis.stopMotors();
                    hood.setAngle(0.95);
                    shooter.startShooting1(-0.63);
                    shooter.startShooting2(-0.63);
                    indexer.startHopper();
                    hasStartedShooting = true;

                } else if (elapsedTime < SHOOT_TIME) {
                    // continue shooting until the time runs out
                    //  just wait -- motors already running
                } else {
                    // stop everything
                    indexer.stopHopper();
                    shooter.setState(ShooterState.RESTING);
                    shooter.startShooting1(0);
                    shooter.startShooting2(0);
                    // transition to next state
                    runtime.reset();
                    currentState = AutoState.MOVEOFFLINE;
                }
                break;

            case MOVEOFFLINE:
                chassis.strafeRight();
                break;
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Elapsed Time", "%.2f seconds", elapsedTime);
        telemetry.update();
    }
}
