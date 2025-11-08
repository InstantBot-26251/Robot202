package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants.MOVE_TIME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.vision.ATLivestream;
import org.firstinspires.ftc.teamcode.vision.ATVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Shoot 3 MOTIF")
@Config
public class AutoShoot extends OpMode {
    DcMotorEx fl, fr, bl, br;

    AutoState currentState = AutoState.MOVETOSHOOTINGPOSITION;

    private ElapsedTime runtime = new ElapsedTime();
    private double stateStartTime = 0;

    private ATVision vision;

    private static final int DASHBOARD_FPS = 10;

    private FtcDashboard dashboard;

    private VisionPortal visionPortal;
    private ATLivestream atLivestream;
    private AprilTagProcessor aprilTagProcessor;

    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;

    public static final double time = 1.75;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);

        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        vision = ATVision.getInstance();

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
        double elapsedTime = runtime.seconds() - stateStartTime;

        switch (currentState) {

            case MOVETOSHOOTINGPOSITION:
                if (elapsedTime < MOVE_TIME) {
                    moveBackward();
                } else {
                    stopMotors();
                     currentState = AutoState.INDEXARTIFACT1;
                }
                break;

            case SHOOTARTIFACT1:
                indexer.startHopper();
                try {
                    wait(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
        }

        telemetry.addData("Current State", currentState);
        telemetry.addData("Elapsed Time", "%.2f seconds", elapsedTime);
        telemetry.update();
    }

    public void moveBackward() {
        fl.setPower(-0.5);
        fr.setPower(-0.5);
        bl.setPower(-0.5);
        br.setPower(-0.5);
    }

    public void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
