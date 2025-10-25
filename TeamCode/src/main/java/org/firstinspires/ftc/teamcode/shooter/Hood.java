package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;
import org.firstinspires.ftc.teamcode.util.hardware.InstantServo;

public class Hood extends SubsystemTemplate {
    public InstantServo hoodServo;

    // Current state
    private double currentAngleDeg;
    private double targetAngleDeg;
    private double currentServoPosition;
    private ElapsedTime moveTimer;

    // Physical limits (tune these to match your robot!)
    private static final double MIN_ANGLE_DEG = 0;  // Minimum safe angle
    private static final double MAX_ANGLE_DEG = 70;  // Maximum safe angle
    private static final double MIN_SERVO_POS = 0.0;
    private static final double MAX_SERVO_POS = 1.0;

    private static final double CALIBRATED_MIN_ANGLE = 0;
    private static final double CALIBRATED_MAX_ANGLE = 70;
    private static final double CALIBRATED_MIN_POS = 0;
    private static final double CALIBRATED_MAX_POS = 1;


    // Movement parameters
    private static final double SERVO_MOVE_TIME = 0.5;  // Estimated time for full servo travel (seconds)
    private static final double POSITION_TOLERANCE = 0.02; // Servo position tolerance

    // Default/safe positions
    private static final double DEFAULT_ANGLE = 45.0;
    private static final double SAFE_ANGLE = 30.0;  // Safe angle for transport/idle

    private static final Hood INSTANCE = new Hood();
    public static Hood getInstance() { return INSTANCE; }

    private Hood() {
        currentAngleDeg = DEFAULT_ANGLE;
        targetAngleDeg = DEFAULT_ANGLE;
        currentServoPosition = 0.5;
        moveTimer = new ElapsedTime();
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();
        hoodServo = new InstantServo(RobotMap.getInstance().HOOD);

        setAngle(SAFE_ANGLE);
        moveTimer.reset();

        if (telemetry != null) {
            telemetry.addData("Hood", "Initialized at %.1f°", SAFE_ANGLE);
        }
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        // Ensure RobotMap has been initialized
        if (RobotMap.getInstance().HOOD == null) {
            if (telemetry != null)
                telemetry.addLine("Hood Error: RobotMap.init(hardwareMap) not called OR 'hood' not in config.");
            return;
        }
        // Set to safe starting position
//        setAngle(SAFE_ANGLE);
            hoodServo = new InstantServo(RobotMap.getInstance().HOOD);
        double p = RobotMap.getInstance().HOOD.getPosition();
        if (Double.isFinite(p)) {
            currentServoPosition = p;
            currentAngleDeg = mapServoToAngle(p);
            targetAngleDeg = currentAngleDeg;
        }
            moveTimer.reset();

        if (telemetry != null) {
            telemetry.addData("Hood", "Initialized at %.1f°", SAFE_ANGLE);
        }
    }

    public boolean setAngle(double angleDeg) {
        // Clamp to safe limits
        double clampedAngle = clampAngle(angleDeg);
        boolean wasClamped = (clampedAngle != angleDeg);

        if (wasClamped && telemetry != null) {
            telemetry.addData("Hood Warning", "Angle %.1f° clamped to %.1f°", angleDeg, clampedAngle);
        }

        targetAngleDeg = clampedAngle;
        currentAngleDeg = clampedAngle;

        // Convert to servo position
        double servoPos = mapAngleToServo(clampedAngle);

        // Validate servo position
        if (!isValidServoPosition(servoPos)) {
            if (telemetry != null) {
                telemetry.addData("Hood Error", "Invalid servo position calculated");
            }
            return false;
        }

        currentServoPosition = servoPos;

        // Set servo position
        if (hoodServo != null) {
            hoodServo.setPosition(servoPos);
            moveTimer.reset();
        }

        return !wasClamped;
    }

    public double getCurrentAngle() {
        return currentAngleDeg;
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public double getCurrentServoPosition() {
        return currentServoPosition;
    }

    public void setSafePosition() {
        setAngle(SAFE_ANGLE);
    }

    public void setDefaultPosition() {
        setAngle(DEFAULT_ANGLE);
    }

    public boolean isAtTarget() {
        return moveTimer.seconds() >= SERVO_MOVE_TIME;
    }

    public double getTimeSinceMove() {
        return moveTimer.seconds();
    }

    public boolean setRawServoPosition(double servoPos) {
        if (!isValidServoPosition(servoPos)) {
            if (telemetry != null) {
                telemetry.addData("Hood Error", "Invalid raw servo position: %.2f", servoPos);
            }
            return false;
        }
        if (hoodServo != null) {
         applyServo(servoPos);

//            moveTimer.reset();
            return true;
        }

        return false;
    }
    public void setManualAngle(double servoPos) {
        hoodServo.setPosition(servoPos);
    }

    private double mapAngleToServo(double angleDeg) {
        double angleRange = CALIBRATED_MAX_ANGLE - CALIBRATED_MIN_ANGLE;
        double servoRange = CALIBRATED_MAX_POS - CALIBRATED_MIN_POS;
        if (Math.abs(angleRange) < 1e-3) return CALIBRATED_MIN_POS;

        double normalized = (angleDeg - CALIBRATED_MIN_ANGLE) / angleRange;
        double servoPos = normalized * servoRange + CALIBRATED_MIN_POS;

        return Math.max(MIN_SERVO_POS, Math.min(MAX_SERVO_POS, servoPos));
        // Clamp to servo limits
    }

    private double mapServoToAngle(double servoPos) {
        double servoRange = CALIBRATED_MAX_POS - CALIBRATED_MIN_POS;
        double angleRange = CALIBRATED_MAX_ANGLE - CALIBRATED_MIN_ANGLE;

        // Edge case: avoid division by zero
        if (Math.abs(servoRange) < 0.001) {
            return CALIBRATED_MIN_ANGLE;
        }

        double normalized = (servoPos - CALIBRATED_MIN_POS) / servoRange;
        double angle = normalized * angleRange + CALIBRATED_MIN_ANGLE;

        return clampAngle(angle);
    }

    private double clampAngle(double angleDeg) {
        return Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, angleDeg));
    }

    private boolean isValidServoPosition(double servoPos) {
        return servoPos >= MIN_SERVO_POS && servoPos <= MAX_SERVO_POS
                && !Double.isNaN(servoPos) && !Double.isInfinite(servoPos);
    }

    public boolean isInitialized() {
        return hoodServo != null && telemetry != null;
    }

    public double[] getAngleRange() {
        return new double[] { MIN_ANGLE_DEG, MAX_ANGLE_DEG };
    }

    public void calibrationTest(double testServoPos) {
        if (setRawServoPosition(testServoPos)) {
            double calculatedAngle = mapServoToAngle(testServoPos);
            if (telemetry != null) {
                telemetry.addLine("=== CALIBRATION TEST ===");
                telemetry.addData("Servo Position", "%.3f", testServoPos);
                telemetry.addData("Calculated Angle", "%.1f°", calculatedAngle);
                telemetry.addLine("Measure actual angle and update constants!");
            }
        }
    }

    public void runCalibrationSequence() {
        if (telemetry != null) {
            telemetry.addLine("=== HOOD CALIBRATION ===");
            telemetry.addLine("Testing positions...");
        }

        double[] testPositions = {0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0};

        for (double pos : testPositions) {
            if (telemetry != null) {
                telemetry.addData("Servo Pos", "%.2f → Angle: %.1f°",
                        pos, mapServoToAngle(pos));
            }
        }
    }

    @Override
    public void periodic() {
        if (telemetry == null) {
            return;
        }

        telemetry.addLine();
        telemetry.addLine("=== HOOD STATUS ===");
        telemetry.addData("Current Angle", "%.1f°", currentAngleDeg);
        telemetry.addData("Target Angle", "%.1f°", targetAngleDeg);
        telemetry.addData("Servo Position", "%.3f", currentServoPosition);
        telemetry.addData("At Target", isAtTarget() ? "YES" : "MOVING");
        telemetry.addData("Time Since Move", "%.2f s", getTimeSinceMove());
        telemetry.addData("Valid Range", "%.0f° - %.0f°", MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        // Visual angle indicator
        String angleBar = getAngleBar(currentAngleDeg, 20);
        telemetry.addData("Angle", angleBar);
    }

    private String getAngleBar(double angle, int length) {
        double normalized = (angle - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        int position = (int)(normalized * length);
        position = Math.max(0, Math.min(length - 1, position));

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < length; i++) {
            if (i == position) {
                bar.append("|");
            } else if (i < position) {
                bar.append("=");
            } else {
                bar.append(" ");
            }
        }
        bar.append("] ");
        bar.append(String.format("%.1f°", angle));
        return bar.toString();
    }
    public void printCalibrationInfo() {
        if (telemetry == null) return;

        telemetry.addLine("=== HOOD CALIBRATION INFO ===");
        telemetry.addData("Min Angle", "%.1f° @ servo %.2f", CALIBRATED_MIN_ANGLE, CALIBRATED_MIN_POS);
        telemetry.addData("Max Angle", "%.1f° @ servo %.2f", CALIBRATED_MAX_ANGLE, CALIBRATED_MAX_POS);
        telemetry.addData("Angle Range", "%.1f° - %.1f°", MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        telemetry.addData("Servo Range", "%.2f - %.2f", MIN_SERVO_POS, MAX_SERVO_POS);
        telemetry.addData("Move Time", "%.2f s", SERVO_MOVE_TIME);
    }

    private void applyServo(double servoPos) {
        if (hoodServo == null) return;
        hoodServo.setPosition(servoPos);
        currentServoPosition = servoPos;
        currentAngleDeg = mapServoToAngle(servoPos);
        targetAngleDeg = currentAngleDeg;
        moveTimer.reset();
    }
}
