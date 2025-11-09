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
import org.firstinspires.ftc.teamcode.util.math.MathPM;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Shoot 3")
@Config
public class AutoShoot extends OpMode {
    DcMotorEx fl, fr, bl, br;

    AutoState currentState = AutoState.MOVETOSHOOTINGPOSITION;

    private ElapsedTime runtime = new ElapsedTime();
    private double stateStartTime = 0;

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

        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        vision = ATVision.getInstance();

        indexer.onTeleopInit();
        shooter.onTeleopInit();
        hood.onTeleopInit();

        indexer = Indexer.getInstance();

        indexer.initHardware();

        indexer.onTeleopInit();

        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lr");
        br = hardwareMap.get(DcMotorEx.class, "rr");

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

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
    public void init_loop() {
        runtime.reset();
        telemetry.addData("Current State", currentState);
        stateStartTime = runtime.seconds();
    }


    @Override
    public void start() {
        runtime.reset();
        currentState = AutoState.MOVETOSHOOTINGPOSITION;
        stateStartTime = runtime.seconds();
    }

    @Override
    public void loop() {
        shooter.periodic();
        indexer.periodic();
        hood.periodic();

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
                        }                   }}}}

        double elapsedTime = runtime.seconds() - stateStartTime;

        switch (currentState) {

            case MOVETOSHOOTINGPOSITION:
                if (elapsedTime < MOVE_TIME) {
                    chassis.moveBackward();
                } else {
                    chassis.stopMotors();
                     currentState = AutoState.SHOOTARTIFACT1;
                }
                break;

            case SHOOTARTIFACT1:
                if (!hasStartedShooting) {
                    // First time in this state - start shooting
                    hood.setAngle(calculatedAngle);
                    indexer.startHopper();
                    shooter.startShooting(1);
                    hasStartedShooting = true;

                } else if (elapsedTime < SHOOT_TIME) {
                    // Continue shooting until time expires
                    // Motors are already running, just wait
                } else {
                    // stop everything
                    indexer.stopHopper();
                    shooter.stopShooting();
                    // transition to next state
                     currentState = AutoState.INDEXARTIFACT1;
                     stateStartTime = runtime.seconds();
                     break;
                }
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Elapsed Time", "%.2f seconds", elapsedTime);
        telemetry.update();
    }


}
