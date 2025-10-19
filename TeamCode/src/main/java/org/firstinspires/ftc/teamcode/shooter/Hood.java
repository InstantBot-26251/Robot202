package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.util.hardware.InstantServo;

public class Hood extends SubsystemTemplate {
    public InstantServo hoodServo;
    private double currentAngleDeg = 0;

    private static final Hood INSTANCE = new Hood();
    public static Hood getInstance() { return INSTANCE; }

    private Hood() {}

    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        hoodServo = new InstantServo(RobotMap.getInstance().HOOD);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        hoodServo = new InstantServo(RobotMap.getInstance().HOOD);

    }

    public void setAngle(double angleDeg) {
        // Map from degrees to servo [0,1] range
        currentAngleDeg = angleDeg;
        double servoPos = mapAngleToServo(angleDeg);
        hoodServo.setPosition(servoPos);
    }

    public double getCurrentAngle() {
        return currentAngleDeg;
    }

    public void setManualAngle(double servoPos) {
        hoodServo.setPosition(servoPos);
    }

    private double mapAngleToServo(double angleDeg) {
        // TODO: tune these empirically
        double minAngle = 15, maxAngle = 40;
        double minPos = 0.2, maxPos = 0.8;
        return (angleDeg - minAngle) / (maxAngle - minAngle) * (maxPos - minPos) + minPos;
    }

    @Override
    public void periodic() {
        telemetry.addLine();
        telemetry.addData("Hood Angle: ", getCurrentAngle());
    }
}
