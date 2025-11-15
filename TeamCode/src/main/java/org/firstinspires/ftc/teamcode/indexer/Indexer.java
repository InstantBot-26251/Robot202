package org.firstinspires.ftc.teamcode.indexer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.ENCODER_TICKS_PER_SLOT;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.GREEN_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.PURPLE_BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.PURPLE_MIN_RATIO;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.PURPLE_RED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.ROTOR_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.ROTOR_kD;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.ROTOR_kI;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.ROTOR_kP;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.SLOTS;
import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.TRANSFER_POSITION_OFFSET;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.indexer.Enums.IndexerState;
import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;
import org.firstinspires.ftc.teamcode.robot.Enigma;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

import java.util.HashMap;
import java.util.Map;

public class Indexer extends SubsystemTemplate {

    // Hardware - 6 color sensors total (2 per slot as redundant backups)
    private DcMotorEx rotorMotor;
    private CRServo hopper;

    // Slot 0 sensors (both scan same position)
    private RevColorSensorV3 slot0SensorA;  // Primary sensor for slot 0
    private RevColorSensorV3 slot0SensorB;  // Backup sensor for slot 0

    // Slot 1 sensors (both scan same position)
    private RevColorSensorV3 slot1SensorA; // Primary sensor for slot 1
    private RevColorSensorV3 slot1SensorB; // Backup sensor for slot 1

    // Slot 2 sensors (both scan same position)
    private RevColorSensorV3 slot2SensorA; // Primary sensor for slot 2
    private RevColorSensorV3 slot2SensorB; // Backup sensor for slot 2

    // PID Controller for position control
    private PIDController rotorPid;


    // State tracking
    private IndexerState state = IndexerState.IDLE;
    private boolean isCalibrated = false;
    private int currentSlot = 0; // Which slot is currently aligned with intake (0-2)

    // Artifact storage - maps slot number (0-2) to artifact color and presence
    private Map<Integer, ArtifactSlot> slots = new HashMap<>();


    private static Indexer INSTANCE;

    public static Indexer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Indexer();
        }
        return INSTANCE;
    }

    private Indexer() {
        rotorPid = new PIDController(ROTOR_kP, ROTOR_kI, ROTOR_kD);

        // Initialize all slots as empty
        slots.put(0, new ArtifactSlot());
        slots.put(1, new ArtifactSlot());
        slots.put(2, new ArtifactSlot());
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        reset();
        calibrateRotor();

        rotorPid.setSetPoint(getRotorPosition());
    }

    @Override
    public void onTeleopInit() {
        telemetry = Enigma.getInstance().getTelemetry();

        reset();
        calibrateRotor();

        rotorPid.setSetPoint(getRotorPosition());
    }

    public void reset() {
        state = IndexerState.IDLE;
        isCalibrated = false;
        currentSlot = 0;

        // Clear all slots
        for (ArtifactSlot slot : slots.values()) {
            slot.clear();
        }


//        stopHopper();
//        stopRotor();

        Log.i("Indexer", "Reset complete");
    }

    public void initHardware() {
        rotorMotor = RobotMap.getInstance().INDEXER_ROTOR;
        hopper = RobotMap.getInstance().INDEXER_HOPPER;
//        slot0SensorA = RobotMap.getInstance().COLOR1;
//        slot0SensorB = RobotMap.getInstance().COLOR2;
//        slot1SensorA = RobotMap.getInstance().COLOR3;
//        slot1SensorB = RobotMap.getInstance().COLOR4;
//        slot2SensorA = RobotMap.getInstance().COLOR5;
//        slot2SensorB = RobotMap.getInstance().COLOR6;
    }


    //------------------------------CALIBRATION---------------------------------------------//


    /**
     * Calibrates the indexer by rotating until slot 0 is aligned with intake
     */
    public void calibrateRotor() {
        if (isCalibrated) {
            Log.i("Indexer", "Already calibrated");
            return;
        }

        state = IndexerState.CALIBRATING;
        currentSlot = 0;

        // Reset encoder - assume we're starting at entry position
        rotorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rotorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        isCalibrated = true;
        state = IndexerState.IDLE;

        Log.i("Indexer", "Calibrated - starting at entry position (slot 0)");
    }

    //---------------------------------------POSITION CONTROL--------------------------//
    /**
     * Rotates to a specific slot number (0-2)
     * Always rotates forward only - never backward
     */
    public void rotateToSlot(int targetSlot) {
        if (!isCalibrated || targetSlot < 0 || targetSlot > 2) {
            Log.e("Indexer", "Cannot rotate: not calibrated or invalid slot " + targetSlot);
            return;
        }

        if (currentSlot == targetSlot) {
            Log.i("Indexer", "Already at slot " + targetSlot);
            return;
        }

        state = IndexerState.ROTATING;

        // Calculate forward rotation distance (always forward, never backward)
        int slotsToRotate = (targetSlot - currentSlot + 3) % 3;

        // Calculate target encoder position (always add, never subtract)
        double targetPos = getRotorPosition() + (slotsToRotate * ENCODER_TICKS_PER_SLOT);
        rotorPid.setSetPoint(targetPos);

        currentSlot = targetSlot;

        Log.i("Indexer", "Rotating " + slotsToRotate + " slot(s) forward to slot " + targetSlot);
    }


    /**
     * Rotates to transfer position (where hopper kicks artifacts to shooter)
     * This works correctly regardless of how many full rotations have occurred
     */
    public void rotateToTransferPosition() {
        if (!isCalibrated) {
            Log.e("Indexer", "Cannot rotate to transfer: not calibrated");
            return;
        }

        state = IndexerState.ROTATING;

        // Get current actual position
        double currentPos = getRotorPosition();

        // Normalize current position to [0, TICKS_PER_ROTATION)
        double normalizedCurrentPos = normalizePosition(currentPos);

        // Transfer position is always at this fixed position in normalized space
        // (TRANSFER_POSITION_OFFSET ticks past slot 0)
        double normalizedTargetPos = TRANSFER_POSITION_OFFSET;

        // Handle case where transfer position might exceed one rotation
        if (normalizedTargetPos >= ENCODER_TICKS_PER_SLOT * SLOTS) {
            normalizedTargetPos = normalizePosition(normalizedTargetPos);
        }

        // Find shortest path
        double distanceToMove = calculateShortestPath(normalizedCurrentPos, normalizedTargetPos);

        // Apply movement to actual (non-normalized) position
        double targetPos = currentPos + distanceToMove;
        rotorPid.setSetPoint(targetPos);

        String direction = distanceToMove >= 0 ? "forward" : "backward";

        Log.i("Indexer", String.format("Rotating %s to transfer position (from %.0f to %.0f, distance: %.0f)",
                direction, currentPos, targetPos, distanceToMove));
    }

    public void rotateToPreviousSlot() {
        if (!isCalibrated) {
            Log.e("Indexer", "Cannot rotate previous: indexer not calibrated");
            return;
        }

        // Wrap-around backwards slot selection
        int previousSlot = (currentSlot - 1 + 3) % 3;

        Log.i("Indexer", "Rotating to previous slot: " + previousSlot);

        double targetPos = getRotorPosition() - ENCODER_TICKS_PER_SLOT;
        rotorPid.setSetPoint(targetPos);
    }


    /**
     * Rotates to the next slot (shortest path)
     */
    public void rotateToNextSlot() {
        int nextSlot = (currentSlot + 1) % 3;
        rotateToSlot(nextSlot);
    }


    /**
     * Stops rotor motor
     */
    public void stopRotor() {
        rotorMotor.setPower(0);
        if (state == IndexerState.ROTATING) {
            state = IndexerState.IDLE;
        }
    }

    /**
     * Checks if rotor has reached target position
     */
    public boolean isAtTargetPosition() {
        double error = Math.abs(rotorMotor.getCurrentPosition() - rotorPid.getSetPoint());
        return error < ROTOR_ERROR_TOLERANCE;
    }


    //------------------------ARTIFACT DETECTION---------------------------------------------//

    /**
     * Checks if a artifact is present in a specific slot using sensors
     * Returns true if EITHER sensor detects a artifact (safeguard against holes)
     * @param slotNumber 0, 1, or 2
     * @return true if artifact detected by at least one sensor
     */

    public boolean isArtifactInSlot(int slotNumber) {
        RevColorSensorV3 sensorA = getSensorA(slotNumber);
        RevColorSensorV3 sensorB = getSensorB(slotNumber);

        if (sensorA == null || sensorB == null) return false;

        boolean detectedByA = isArtifactPresent(sensorA);
        boolean detectedByB = isArtifactPresent(sensorB);

        // Artifact is present if EITHER sensor detects it
        // This handles cases where holes in the artifact might obscure one sensor
        return detectedByA || detectedByB;
    }

    /**
     * Gets detection confidence for a slot (both sensors agree is high confidence)
     * @return 2 = both sensors detect, 1 = one sensor detects, 0 = no detection
     */
    public int getDetectionConfidence(int slotNumber) {
        RevColorSensorV3 sensorA = getSensorA(slotNumber);
        RevColorSensorV3 sensorB = getSensorB(slotNumber);

        if (sensorA == null || sensorB == null) return 0;

        int confidence = 0;
        if (isArtifactPresent(sensorA)) confidence++;
        if (isArtifactPresent(sensorB)) confidence++;

        return confidence;
    }


    /**
     * Generic artifact presence check for any sensor
     */
    private boolean isArtifactPresent(RevColorSensorV3 sensor) {
        int green = sensor.green();
        int red = sensor.red();
        int blue = sensor.blue();

        // Artifact present if any color value exceeds threshold
        return (green > GREEN_THRESHOLD) || (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD);
    }

    /**
     * Detects artifact color in a specific slot using redundant sensors
     * Attempts to use both sensors and averages the readings for accuracy
     */
    public ArtifactColor detectColorInSlot(int slotNumber) {
        RevColorSensorV3 sensorA = getSensorA(slotNumber);
        RevColorSensorV3 sensorB = getSensorB(slotNumber);

        if (sensorA == null || sensorB == null) return ArtifactColor.NONE;

        // Get readings from both sensors
        boolean aHasArtifact = isArtifactPresent(sensorA);
        boolean bHasArtifact = isArtifactPresent(sensorB);

        // If neither sensor detects a artifact
        if (!aHasArtifact && !bHasArtifact) {
            return ArtifactColor.NONE;
        }

        // If only one sensor detects, use that one
        if (aHasArtifact && !bHasArtifact) {
            return classifyColor(sensorA.red(), sensorA.green(), sensorA.blue());
        }
        else if (!aHasArtifact) {
            return classifyColor(sensorB.red(), sensorB.green(), sensorB.blue());
        }

        // Both sensors detect artifact - average the readings for better accuracy
        int avgRed = (sensorA.red() + sensorB.red()) / 2;
        int avgGreen = (sensorA.green() + sensorB.green()) / 2;
        int avgBlue = (sensorA.blue() + sensorB.blue()) / 2;

        return classifyColor(avgRed, avgGreen, avgBlue);
    }

    /**
     * Classifies RGB values into artifact color (GREEN or PURPLE only)
     */
    private ArtifactColor classifyColor(int red, int green, int blue) {

        // Detect GREEN - green channel dominant
        if (green > GREEN_THRESHOLD && green > red && green > blue) {
            return ArtifactColor.GREEN;
        }

        // Detect PURPLE - combination of red and blue, low green
        // Purple = Red + Blue, minimal Green
        if (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD) {
            // Check that red and blue are similar (purple is balanced)
            double ratio = Math.min(red, blue) / (double) Math.max(red, blue);

            // Purple should have low green relative to red/blue
            int avgRedBlue = (red + blue) / 2;

            if (ratio > PURPLE_MIN_RATIO && green < avgRedBlue) {
                return ArtifactColor.PURPLE;
            }
        }

        return ArtifactColor.UNKNOWN;
    }

    //----------------------------Artifact Management---------------------------------------//
    /**
     * Updates artifact tracking for all slots based on sensor readings
     * We NEED TO CALL this periodically to maintain accurate artifact state
     */
    public void updateArtifactTracking() {
        for (int i = 0; i < 3; i++) {
            ArtifactSlot slot = slots.get(i);
            if (slot == null) continue;

            boolean artifactPresent = isArtifactInSlot(i);
            int confidence = getDetectionConfidence(i);

            if (artifactPresent) {
                // Artifact detected in slot
                if (!slot.hasConfirmedArtifact()) {
                    // New artifact detected
                    ArtifactColor color = detectColorInSlot(i);
                    slot.setColor(color);
                    slot.setHasConfirmedArtifact(true);
                    slot.setDetectionConfidence(confidence);

                    String confidenceStr = confidence == 2 ? "BOTH sensors" : "ONE sensor";
                    Log.i("Indexer", "Artifact detected in slot " + i + ": " + color + " (" + confidenceStr + ")");
                } else {
                    // Update confidence for existing artifact
                    slot.setDetectionConfidence(confidence);
                }
            } else {
                // No artifact detected
                if (slot.hasConfirmedArtifact()) {
                    // Artifact has left the slot
                    Log.i("Artifact left slot ", i +"");
                    slot.clear();
                }
            }
        }
    }

    /**
     * Finds the slot number containing the specified color
     * @return slot number (0-2) or -1 if not found
     */
    public int findArtifactSlot(ArtifactColor targetColor) {
        for (int i = 0; i < 3; i++) {
            ArtifactSlot slot = slots.get(i);
            if (slot != null && slot.getColor() == targetColor && slot.hasConfirmedArtifact()) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Gets artifact count across all slots
     */
    public int getArtifactCount() {
        int count = 0;
        for (ArtifactSlot slot : slots.values()) {
            if (slot.hasConfirmedArtifact()) count++;
        }
        return count;
    }

    //---------------------------------HOPPER CONTROL--------------------------//

    public void startHopper() {
        hopper.setPower(-1);
    }

    public void stopHopper() {
        hopper.setPower(0);
    }

    public boolean isArtifactTransferred(int slotNumber) {
        return !isArtifactInSlot(slotNumber);
    }

    //-----------------------------SENSOR HELPER METHODS-------------------------//

    private RevColorSensorV3 getSensorA(int slotNumber) {
        switch (slotNumber) {
            case 0: return slot0SensorA;
            case 1: return slot1SensorA;
            case 2: return slot2SensorA;
            default: return null;
        }
    }

    private RevColorSensorV3 getSensorB(int slotNumber) {
        switch (slotNumber) {
            case 0: return slot0SensorB;
            case 1: return slot1SensorB;
            case 2: return slot2SensorB;
            default: return null;
        }
    }

    //------------------------POSITION TRACKING--------------------------------------//
    /**
     * Normalizes an encoder position to range [0, TICKS_PER_ROTATION)
     * This keeps position values manageable and simplifies rotation math
     */
    private double normalizePosition(double position) {
        double ticksPerRotation = ENCODER_TICKS_PER_SLOT * SLOTS;
        // Use modulus arithmetic to wrap position into one rotation
        double normalized = position % ticksPerRotation;
        // Handle negative positions
        if (normalized < 0) {
            normalized += ticksPerRotation;
        }
        return normalized;
    }

    /**
     * Calculates the shortest distance to move from current to target position
     * Returns positive for forward movement, negative for backward movement
     */
    private double calculateShortestPath(double currentNormalized, double targetNormalized) {
        double ticksPerRotation = ENCODER_TICKS_PER_SLOT * SLOTS;

        // Calculate forward distance
        double forwardDist = (targetNormalized - currentNormalized + ticksPerRotation) % ticksPerRotation;

        // Calculate backward distance
        double backwardDist = (currentNormalized - targetNormalized + ticksPerRotation) % ticksPerRotation;

        // Return the shorter path (negative if backward)
        if (forwardDist <= backwardDist) {
            return forwardDist;
        } else {
            return -backwardDist;
        }
    }



    //------------------------GETTERS & SETTERS-----------------------------//

    public IndexerState getState() {
        return state;
    }

    public void setState(IndexerState state) {
        this.state = state;
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public int getCurrentSlot() {
        return currentSlot;
    }

    public boolean isCurrentSlotAvailable() {
        ArtifactSlot slot = slots.get(currentSlot);
        return slot != null && !slot.hasConfirmedArtifact();
    }

    public boolean isFull() {
        return getArtifactCount() >= 3;
    }

    public boolean isEmpty() {
        return getArtifactCount() == 0;
    }

    private void setRotorPower(double power) {
        rotorMotor.setPower(power);
    }

    public void setRotorPos(double pos) {
        rotorPid.setSetPoint(pos);
    }

    public Map<Integer, ArtifactSlot> getSlots() {
        return new HashMap<>(slots); // Return copy for safety
    }

    public ArtifactSlot getSlot(int slotNumber) {
        return slots.get(slotNumber);
    }

    public ArtifactColor getArtifactColorInSlot(int slotNumber) {
        ArtifactSlot slot = slots.get(slotNumber);
        return slot != null ? slot.getColor() : ArtifactColor.NONE;
    }

    public int getRotorPosition() {
        return rotorMotor.getCurrentPosition();
    }

    public int getRotorTarget() {
        return rotorMotor.getTargetPosition();
    }

    /**
     * For debugging and PID tuning
     */
    public void updatePID() {
        rotorPid.setPID(ROTOR_kP, ROTOR_kI, ROTOR_kD);
    }

    public void setPower(double power) {
        rotorMotor.setPower(power);
    }

    public void cancelPIDMovement() {
        // Change state FIRST - this prevents periodic() from continuing to run PID
        state = IndexerState.IDLE;

        // Stop the motor immediately
        rotorMotor.setPower(0);

        // Reset PID setpoint to current position to prevent any further movement
        rotorPid.setSetPoint(getRotorPosition());

        // Log for debugging
        Log.i("Indexer", "PID movement canceled - manual control active at position " + getRotorPosition());
    }

    @Override
    public void periodic() {
        updatePID();
        if (state == IndexerState.ROTATING) {
            double output = rotorPid.calculate(rotorMotor.getCurrentPosition());

            setRotorPower(output);

            if (isAtTargetPosition()) {
                stopRotor();
                Log.i("Indexer", "Rotation complete - at slot " + currentSlot);
            }
        }
        // Telemetry
        if (telemetry != null) {
            telemetry.addLine();
            telemetry.addData("Indexer State", state);
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Artifact Count", getArtifactCount());
            telemetry.addData("Rotor Pos", getRotorPosition());
            telemetry.addData("Rotor Target", getRotorTarget());
            telemetry.addData("At Target", isAtTargetPosition());

            for (int i = 0; i < 3; i++) {
                telemetry.addData("Slot " + i, slots.get(i));
            }
        }
    }


}
