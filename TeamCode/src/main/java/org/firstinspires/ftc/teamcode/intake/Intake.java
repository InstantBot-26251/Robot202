package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.Unnamed;
import org.firstinspires.ftc.teamcode.util.subsystem.SubsystemTemplate;

public class Intake extends SubsystemTemplate {
    Telemetry telemetry;

    DcMotor intake;

    // Motor power constants (tune these!)
    private static final double INTAKE_POWER = 1.0;
    private static final double REJECT_POWER = -1.0;


    public enum IntakeState {
        INTAKE, REJECTION, STOPPED
    }

    IntakeState state;

    private static final Intake INSTANCE = new Intake();

    public static Intake getInstance() {
        return INSTANCE;
    }

    private Intake() {
        state = IntakeState.STOPPED;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Unnamed.getInstance().getTelemetry();
        configureHardware();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Unnamed.getInstance().getTelemetry();
        configureHardware();
    }

    private void configureHardware() {
        intake = RobotMap.getInstance().INTAKE;
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // State control

    public void intake() {
        intake.setPower(INTAKE_POWER);
        state = IntakeState.INTAKE;
    }

    public void reject() {
        intake.setPower(REJECT_POWER);
        state = IntakeState.REJECTION;
    }

    public void stop() {
        intake.setPower(0);
        state = IntakeState.STOPPED;
    }

    // Getters

    public IntakeState getState() {
        return state;
    }

    // Periodic

    @Override
    public void periodic() {
        telemetry.addLine();
        telemetry.addData("Intake State", state);
        telemetry.addData("Intake Power", intake.getPower());
    }
}
