package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.util.hardware.InstantMotor;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

public class Shooter extends SubsystemTemplate {

    InstantMotor shooter1;
    InstantMotor shooter2;

    Telemetry telemetry;

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

        shooter1 = new InstantMotor(RobotMap.getInstance().SHOOTER_1);
        shooter2 = new InstantMotor(RobotMap.getInstance().SHOOTER_2);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setState(ShooterState.RESTING);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        shooter1 = new InstantMotor(RobotMap.getInstance().SHOOTER_1);
        shooter2 = new InstantMotor(RobotMap.getInstance().SHOOTER_2);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setState(ShooterState.RESTING);
    }

    public void setState(ShooterState targetState) {
        state = targetState;
    }

    public String getState() {
        return "The state is " + state + " .";
    }

    public void startShooting1(double velocity) {
        setState(ShooterState.SHOOTING);
        shooter1.setPower(velocity);
    }
    public void startShooting2(double velocity) {
        setState(ShooterState.SHOOTING);
        shooter2.setPower(velocity);
    }

    public void reject1() {
        setState(ShooterState.REJECTING);
        shooter1.setPower(0.5);
    }

    public void reject2() {
        setState(ShooterState.REJECTING);
        shooter2.setPower(0.5);
    }

    public void stopShooting1() {
        setState(ShooterState.RESTING);
        shooter1.setPower(0);
    }
    public void stopShooting2() {
        setState(ShooterState.RESTING);
        shooter2.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addLine();
        telemetry.addData("Shooter state", state);
    }
}
