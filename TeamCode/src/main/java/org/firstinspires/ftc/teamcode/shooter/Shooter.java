package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.util.hardware.InstantMotor;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

public class Shooter extends SubsystemTemplate {

    InstantMotor shooter;
    Telemetry telemetry;

    public enum ShooterState {
        RESTING, SHOOTING, REJECTION
    }

    ShooterState state;

    private static final Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private Shooter() {
        state = ShooterState.RESTING;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        shooter = new InstantMotor(RobotMap.getInstance().SHOOTER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setState(ShooterState.RESTING);
        periodic();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        shooter = new InstantMotor(RobotMap.getInstance().SHOOTER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setState(ShooterState.RESTING);
    }

    public void setState(ShooterState targetState) {
        state = targetState;
    }

    public String getState() {
        return "The state is " + state + " .";
    }

    public void startShooting(double velocity) {
        setState(ShooterState.SHOOTING);
        shooter.setPower(velocity);
    }

    public void reject() {
        setState(ShooterState.REJECTION);
        shooter.setPower(0.5);
    }

    public void stopShooting() {
        setState(ShooterState.RESTING);
        shooter.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addLine();
        telemetry.addData("Shooter state", state);
    }
}
