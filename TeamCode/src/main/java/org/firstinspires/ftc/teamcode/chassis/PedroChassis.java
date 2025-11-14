package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

import java.util.function.Supplier;

public class PedroChassis extends SubsystemTemplate {
    // Singleton
    private static final PedroChassis INSTANCE = new PedroChassis();
    public static PedroChassis getInstance() { return INSTANCE; }


    private Follower follower;
    private Telemetry telemetry;

    private boolean automatedDrive = false;
    private boolean slowMode = false;
    private double slowMultiplier = 0.5;

    private Supplier<PathChain> lazyPathChain;

    private boolean isFieldCentric = true;
    private double driveScale = 1.0;


    // --- Constructor ---
    public PedroChassis() {
    }


    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        follower.setPose(RobotStatus.robotPose);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        follower = Constants.createFollower(RobotMap.getInstance().getHardwareMap());
        follower.setStartingPose(RobotStatus.robotPose);
        follower.update();

        follower.startTeleopDrive();   // required for Pedro teleop
        setMaxPower(1.0);
    }

    // GETTERS

    public Pose getPoseEstimate() {
        return follower.getPose();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }
    // SETTERS

    public void setPosition(Pose pose) {
        follower.setPose(pose);
    }

    public void setDriveVectors(double fwd, double str, double rot) {
        follower.setTeleOpDrive(
                fwd * driveScale,
                str * driveScale,
                rot * driveScale,
                !isFieldCentric);  // TRUE = robot-centric, FALSE = field-centric
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public void resetHeading() {
        Pose oldPose = getPoseEstimate();
        follower.setPose(new Pose(oldPose.getX(), oldPose.getY()));
    }

    public void enableSlowMode() {
        driveScale = 0.25;
    }

    public void disableSlowMode() {
        driveScale = 1.0;
    }

    @Override
    public void periodic() {
        follower.update();

        RobotStatus.robotPose = follower.getPose();

        telemetry.addData("FC", isFieldCentric);
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Busy", follower.isBusy());
    }
}
