package org.firstinspires.ftc.teamcode.indexer;

import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ARTIFACT_PRESENCE_THRESHOLD;

import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.GREEN_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.PURPLE_BLUE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.PURPLE_MIN_RATIO;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.PURPLE_RED_THRESHOLD;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ROTOR_DISPENSE_SPEED;
import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.ROTOR_INDEX_SPEED;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.indexer.Enums.IndexerState;
import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.SubsystemTemplate;

import java.util.HashMap;
import java.util.Map;

public class Indexer extends SubsystemTemplate {

    // Hardware - 6 color sensors total (2 per slot as redundant backups)
    private DcMotorEx rotorMotor;
    private CRServo hopperServo;

    // Slot 0 sensors (both scan same position)
    private RevColorSensorV3 slot0SensorA;  // Primary sensor for slot 0
    private RevColorSensorV3 slot0SensorB;  // Backup sensor for slot 0

    // Slot 1 sensors (both scan same position)
    private RevColorSensorV3 slot1SensorA; // Primary sensor for slot 1
    private RevColorSensorV3 slot1SensorB; // Backup sensor for slot 1

    // Slot 2 sensors (both scan same position)
    private RevColorSensorV3 slot2SensorA; // Primary sensor for slot 2
    private RevColorSensorV3 slot2SensorB; // Backup sensor for slot 2

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
        // Initialize all slots as empty
        slots.put(0, new ArtifactSlot());
        slots.put(1, new ArtifactSlot());
        slots.put(2, new ArtifactSlot());
    }

    @Override
    public SubsystemTemplate initialize() {
        RobotMap map = RobotMap.getInstance();

        // Get hardware directly from RobotMap
        rotorMotor = map.INDEXER_ROTOR;
        hopperServo = map.INDEXER_HOPPER;

        // Map color sensors to indexer slots
        slot0SensorA = map.COLOR1;  // Slot 0, Sensor A
        slot0SensorB = map.COLOR2;  // Slot 0, Sensor B
        slot1SensorA = map.COLOR3;  // Slot 1, Sensor A
        slot1SensorB = map.COLOR4;  // Slot 1, Sensor B
        slot2SensorA = map.COLOR5;  // Slot 2, Sensor A
        slot2SensorB = map.COLOR6;  // Slot 2, Sensor B

        // Note: COLOR7 is available if needed for intake sensor

        stopHopper();
        return this;
    }


    @Override
    public void onAutonomousInit() {
        reset();
        calibrateRotor();
    }

    @Override
    public void onTeleopInit() {
        reset();
        calibrateRotor();
    }

    public void reset() {
        state = IndexerState.IDLE;
        isCalibrated = false;
        currentSlot = 0;

        // Clear all slots
        for (ArtifactSlot slot : slots.values()) {
            slot.clear();
        }

        stopRotor();
        stopHopper();
    }

    //------------------------------CALIBRATION---------------------------------------------//


    /**
     * Calibrates the indexer by rotating until slot 0 is aligned with intake
     */
    public void calibrateRotor() {
        state = IndexerState.CALIBRATING;
        currentSlot = 0;

        // For now, assume we start at slot 0
        isCalibrated = true;
        state = IndexerState.IDLE;
        System.out.println("Indexer calibrated - starting at slot 0");
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
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Artifact present if any color value exceeds threshold
        return (red > ARTIFACT_PRESENCE_THRESHOLD ||
                green > ARTIFACT_PRESENCE_THRESHOLD ||
                blue > ARTIFACT_PRESENCE_THRESHOLD);
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
        if (bHasArtifact && !aHasArtifact) {
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
        // No artifact present
        if (red < ARTIFACT_PRESENCE_THRESHOLD &&
                green < ARTIFACT_PRESENCE_THRESHOLD &&
                blue < ARTIFACT_PRESENCE_THRESHOLD) {
            return ArtifactColor.NONE;
        }

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
            boolean artifactPresent = isArtifactInSlot(i);
            int confidence = getDetectionConfidence(i);

            if (artifactPresent) {
                // Artifact detected in slot
                if (!slot.hasConfirmedArtifact) {
                    // New artifact detected
                    slot.color = detectColorInSlot(i);
                    slot.hasConfirmedArtifact = true;
                    slot.detectionConfidence = confidence;

                    if (confidence == 2) {
                        System.out.println("Artifact detected in slot " + i + ": " + slot.color + " (BOTH sensors)");
                    } else {
                        System.out.println("Artifact detected in slot " + i + ": " + slot.color + " (ONE sensor - hole in artifact?)");
                    }
                } else {
                    // Update confidence for existing artifact
                    slot.detectionConfidence = confidence;
                }
            } else {
                // No artifact detected
                if (slot.hasConfirmedArtifact) {
                    // Artifact has left the slot
                    System.out.println("Artifact left slot " + i);
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
            if (slots.get(i).color == targetColor && slots.get(i).hasConfirmedArtifact) {
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
            if (slot.hasConfirmedArtifact) count++;
        }
        return count;
    }

    /**
     * Checks if indexer is full (all 3 slots occupied)
     */
    public boolean isFull() {
        return getArtifactCount() >= 3;
    }

    /**
     * Checks if indexer is empty
     */
    public boolean isEmpty() {
        return getArtifactCount() == 0;
    }

    /**
     * Gets the current slot aligned with intake
     */
    public int getCurrentSlot() {
        return currentSlot;
    }

    /**
     * Checks if current slot is available for a new artifact
     */
    public boolean isCurrentSlotAvailable() {
        return !slots.get(currentSlot).hasConfirmedArtifact;
    }

    //-----------------------rotorMotor CONTROL--------------------------------//
    /**
     * Rotates to the next slot (advances by 1)
     */
    public void rotateToNextSlot() {
        if (!isCalibrated) return;

        state = IndexerState.ROTATING;
        rotorMotor.setPower(ROTOR_INDEX_SPEED);

        // Update current slot
        currentSlot = (currentSlot + 1) % 3;
    }

    /**
     * Rotates to a specific slot number
     */
    public void rotateToSlot(int targetSlot) {
        if (!isCalibrated || targetSlot < 0 || targetSlot > 2) return;

        state = IndexerState.ROTATING;

        // Calculate rotation direction and distance
        int slotsToRotate = (targetSlot - currentSlot + 3) % 3;

        if (slotsToRotate == 0) {
            // Already at target
            state = IndexerState.IDLE;
            return;
        }

        // Start rotation
        rotorMotor.setPower(ROTOR_INDEX_SPEED);
        currentSlot = targetSlot;

        System.out.println("Rotating to slot " + targetSlot);
    }

    /**
     * Rotates by a specific number of slots
     */
    public void rotateBySlots(int numSlots) {
        int targetSlot = (currentSlot + numSlots + 3) % 3;
        rotateToSlot(targetSlot);
    }

    public void stopRotor() {
        rotorMotor.setPower(0);
        if (state == IndexerState.ROTATING) {
            state = IndexerState.IDLE;
        }
    }

    /**
     * Checks if rotorMotor has finished rotating
     * In real implementation, check encoder position
     */
    public boolean isRotationComplete() {
        // TODO: Implement actual position checking
        return state != IndexerState.ROTATING;
    }

    //---------------------------------HOPPER CONTROL--------------------------//

    public void startHopper() {
        hopperServo.setPower(1);
    }

    public void stopHopper() {
        hopperServo.setPower(0);
    }



    //---------------------------------DISPENSING---------------------------//
    /**
     * Dispenses the artifact from the current slot
     */
    public void dispenseArtifact() {
        state = IndexerState.DISPENSING;
        startHopper();
        rotorMotor.setPower(ROTOR_DISPENSE_SPEED);
    }

    /**
     * Checks if artifact has been successfully dispensed from current slot
     * Artifact should no longer be detected by either sensor
     */
    public boolean isArtifactDispensed(int slotNumber) {
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

    public Map<Integer, ArtifactSlot> getSlots() {
        return new HashMap<>(slots); // Return copy for safety
    }

    public ArtifactSlot getSlot(int slotNumber) {
        return slots.get(slotNumber);
    }

    public ArtifactColor getArtifactColorInSlot(int slotNumber) {
        ArtifactSlot slot = slots.get(slotNumber);
        return slot != null ? slot.color : ArtifactColor.NONE;
    }

    @Override
    public void periodic() {
        // Continuously update artifact tracking
        updateArtifactTracking();

        // TODO: Add timeout logic for operations
        // TODO: Add rotation completion detection
    }

}
