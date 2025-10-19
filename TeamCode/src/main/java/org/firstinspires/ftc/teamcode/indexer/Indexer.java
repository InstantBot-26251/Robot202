package org.firstinspires.ftc.teamcode.indexer;

import static org.firstinspires.ftc.teamcode.indexer.constants.Constants.BALL_PRESENCE_THRESHOLD;

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
import org.firstinspires.ftc.teamcode.indexer.Enums.BallColor;
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

    // Ball storage - maps slot number (0-2) to ball color and presence
    private Map<Integer, BallSlot> slots = new HashMap<>();


    private static Indexer INSTANCE;

    public static Indexer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Indexer();
        }
        return INSTANCE;
    }

    private Indexer() {
        // Initialize all slots as empty
        slots.put(0, new BallSlot());
        slots.put(1, new BallSlot());
        slots.put(2, new BallSlot());
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
        for (BallSlot slot : slots.values()) {
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

    //------------------------BALL DETECTION---------------------------------------------//

    /**
     * Checks if a ball is present in a specific slot using sensors
     * Returns true if EITHER sensor detects a ball (safeguard against holes)
     * @param slotNumber 0, 1, or 2
     * @return true if ball detected by at least one sensor
     */

    public boolean isBallInSlot(int slotNumber) {
        RevColorSensorV3 sensorA = getSensorA(slotNumber);
        RevColorSensorV3 sensorB = getSensorB(slotNumber);

        if (sensorA == null || sensorB == null) return false;

        boolean detectedByA = isBallPresent(sensorA);
        boolean detectedByB = isBallPresent(sensorB);

        // Ball is present if EITHER sensor detects it
        // This handles cases where holes in the ball might obscure one sensor
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
        if (isBallPresent(sensorA)) confidence++;
        if (isBallPresent(sensorB)) confidence++;

        return confidence;
    }


    /**
     * Generic ball presence check for any sensor
     */
    private boolean isBallPresent(RevColorSensorV3 sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Ball present if any color value exceeds threshold
        return (red > BALL_PRESENCE_THRESHOLD ||
                green > BALL_PRESENCE_THRESHOLD ||
                blue > BALL_PRESENCE_THRESHOLD);
    }

    /**
     * Detects ball color in a specific slot using redundant sensors
     * Attempts to use both sensors and averages the readings for accuracy
     */
    public BallColor detectColorInSlot(int slotNumber) {
        RevColorSensorV3 sensorA = getSensorA(slotNumber);
        RevColorSensorV3 sensorB = getSensorB(slotNumber);

        if (sensorA == null || sensorB == null) return BallColor.NONE;

        // Get readings from both sensors
        boolean aHasBall = isBallPresent(sensorA);
        boolean bHasBall = isBallPresent(sensorB);

        // If neither sensor detects a ball
        if (!aHasBall && !bHasBall) {
            return BallColor.NONE;
        }

        // If only one sensor detects, use that one
        if (aHasBall && !bHasBall) {
            return classifyColor(sensorA.red(), sensorA.green(), sensorA.blue());
        }
        if (bHasBall && !aHasBall) {
            return classifyColor(sensorB.red(), sensorB.green(), sensorB.blue());
        }

        // Both sensors detect ball - average the readings for better accuracy
        int avgRed = (sensorA.red() + sensorB.red()) / 2;
        int avgGreen = (sensorA.green() + sensorB.green()) / 2;
        int avgBlue = (sensorA.blue() + sensorB.blue()) / 2;

        return classifyColor(avgRed, avgGreen, avgBlue);
    }

    /**
     * Classifies RGB values into ball color (GREEN or PURPLE only)
     */
    private BallColor classifyColor(int red, int green, int blue) {
        // No ball present
        if (red < BALL_PRESENCE_THRESHOLD &&
                green < BALL_PRESENCE_THRESHOLD &&
                blue < BALL_PRESENCE_THRESHOLD) {
            return BallColor.NONE;
        }

        // Detect GREEN - green channel dominant
        if (green > GREEN_THRESHOLD && green > red && green > blue) {
            return BallColor.GREEN;
        }

        // Detect PURPLE - combination of red and blue, low green
        // Purple = Red + Blue, minimal Green
        if (red > PURPLE_RED_THRESHOLD && blue > PURPLE_BLUE_THRESHOLD) {
            // Check that red and blue are similar (purple is balanced)
            double ratio = Math.min(red, blue) / (double) Math.max(red, blue);

            // Purple should have low green relative to red/blue
            int avgRedBlue = (red + blue) / 2;

            if (ratio > PURPLE_MIN_RATIO && green < avgRedBlue) {
                return BallColor.PURPLE;
            }
        }

        return BallColor.UNKNOWN;
    }

    //----------------------------Ball Management---------------------------------------//
    /**
     * Updates ball tracking for all slots based on sensor readings
     * We NEED TO CALL this periodically to maintain accurate ball state
     */
    public void updateBallTracking() {
        for (int i = 0; i < 3; i++) {
            BallSlot slot = slots.get(i);
            boolean ballPresent = isBallInSlot(i);
            int confidence = getDetectionConfidence(i);

            if (ballPresent) {
                // Ball detected in slot
                if (!slot.hasConfirmedBall) {
                    // New ball detected
                    slot.color = detectColorInSlot(i);
                    slot.hasConfirmedBall = true;
                    slot.detectionConfidence = confidence;

                    if (confidence == 2) {
                        System.out.println("Ball detected in slot " + i + ": " + slot.color + " (BOTH sensors)");
                    } else {
                        System.out.println("Ball detected in slot " + i + ": " + slot.color + " (ONE sensor - hole in ball?)");
                    }
                } else {
                    // Update confidence for existing ball
                    slot.detectionConfidence = confidence;
                }
            } else {
                // No ball detected
                if (slot.hasConfirmedBall) {
                    // Ball has left the slot
                    System.out.println("Ball left slot " + i);
                    slot.clear();
                }
            }
        }
    }

    /**
     * Finds the slot number containing the specified color
     * @return slot number (0-2) or -1 if not found
     */
    public int findBallSlot(BallColor targetColor) {
        for (int i = 0; i < 3; i++) {
            if (slots.get(i).color == targetColor && slots.get(i).hasConfirmedBall) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Gets ball count across all slots
     */
    public int getBallCount() {
        int count = 0;
        for (BallSlot slot : slots.values()) {
            if (slot.hasConfirmedBall) count++;
        }
        return count;
    }

    /**
     * Checks if indexer is full (all 3 slots occupied)
     */
    public boolean isFull() {
        return getBallCount() >= 3;
    }

    /**
     * Checks if indexer is empty
     */
    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    /**
     * Gets the current slot aligned with intake
     */
    public int getCurrentSlot() {
        return currentSlot;
    }

    /**
     * Checks if current slot is available for a new ball
     */
    public boolean isCurrentSlotAvailable() {
        return !slots.get(currentSlot).hasConfirmedBall;
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
     * Dispenses the ball from the current slot
     */
    public void dispenseBall() {
        state = IndexerState.DISPENSING;
        startHopper();
        rotorMotor.setPower(ROTOR_DISPENSE_SPEED);
    }

    /**
     * Checks if ball has been successfully dispensed from current slot
     * Ball should no longer be detected by either sensor
     */
    public boolean isBallDispensed(int slotNumber) {
        return !isBallInSlot(slotNumber);
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

    public Map<Integer, BallSlot> getSlots() {
        return new HashMap<>(slots); // Return copy for safety
    }

    public BallSlot getSlot(int slotNumber) {
        return slots.get(slotNumber);
    }

    public BallColor getBallColorInSlot(int slotNumber) {
        BallSlot slot = slots.get(slotNumber);
        return slot != null ? slot.color : BallColor.NONE;
    }

    @Override
    public void periodic() {
        // Continuously update ball tracking
        updateBallTracking();

        // TODO: Add timeout logic for operations
        // TODO: Add rotation completion detection
    }

}
