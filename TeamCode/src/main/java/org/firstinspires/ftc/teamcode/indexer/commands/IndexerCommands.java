package org.firstinspires.ftc.teamcode.indexer.commands;

import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.BALL_SETTLE_TIME;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.DISPENSE_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.GATE_DELAY;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.REJECT_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.ROTATION_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.VERIFICATION_DELAY;

import com.arcrobotics.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.indexer.Enums.BallColor;
import org.firstinspires.ftc.teamcode.indexer.Enums.IndexerState;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class IndexerCommands {

    public static final Supplier<Command> CALIBRATE;
    public static final Supplier<Command> INDEX_BALL;
    public static final Supplier<Command> PREPARE_GREEN_BALL;
    public static final Supplier<Command> PREPARE_PURPLE_BALL;
    public static final Supplier<Command> DISPENSE;
    public static final Supplier<Command> DISPENSE_GREEN;
    public static final Supplier<Command> DISPENSE_PURPLE;
    public static final Supplier<Command> REJECT_CURRENT_BALL;
    public static final Supplier<Command> STOP;
    public static final Supplier<Command> RESET;
    public static final Supplier<Command> ROTATE_NEXT;

    static {
        Indexer indexer = Indexer.getInstance();

        //-----------------------CALIBRATION------------------------------//

        /**
         * Calibrates the indexer rotor position
         */
        CALIBRATE = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("Starting indexer calibration...");
                    indexer.calibrateRotor();
                }),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    if (indexer.isCalibrated()) {
                        System.out.println("Indexer calibration complete! Ready at slot 0");
                    } else {
                        System.out.println("WARNING: Indexer calibration failed!");
                        indexer.setState(IndexerState.ERROR);
                    }
                })
        );

        //----------------------ROTATION-------------------------------------------//

        /**
         * Rotates to the next slot
         */
        ROTATE_NEXT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("Rotating to next slot...");
                    indexer.rotateToNextSlot();
                }),
                Commands.waitSeconds(ROTATION_DURATION),
                Commands.runOnce(() -> {
                    indexer.stopRotor();
                    System.out.println("Now at slot " + indexer.getCurrentSlot());
                })
        );

        //---------------------------------------BALL INTAKE/INDEXING-----------------------//

        /**
         * Indexes a single ball into the current slot:
         * 1. Wait for ball to fully enter current slot (both sensors detect)
         * 2. Detect and log ball color
         * 3. Rotate to next slot if not full
         */
        INDEX_BALL = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    if (indexer.isFull()) {
                        System.out.println("ERROR: Indexer is full! Cannot index more balls.");
                        return;
                    }

                    if (!indexer.isCurrentSlotAvailable()) {
                        System.out.println("ERROR: Current slot " + indexer.getCurrentSlot() + " is occupied!");
                        return;
                    }

                    System.out.println("Waiting for ball in slot " + indexer.getCurrentSlot() + "...");
                    indexer.setState(IndexerState.INDEXING);
                }),

                // Wait for ball to be entering the slot
                Commands.waitUntil(() -> indexer.isBallInSlot(indexer.getCurrentSlot()))
                        .withTimeout((long) 5.0),

                // Wait for ball to fully enter (both sensors)
                Commands.waitSeconds(BALL_SETTLE_TIME),
                Commands.waitUntil(() -> indexer.isBallInSlot(indexer.getCurrentSlot()))
                        .withTimeout((long) 2.0),

                // Verify and log ball
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();

                    if (!indexer.isBallInSlot(slot)) {
                        System.out.println("WARNING: Ball not fully detected in slot " + slot);
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    BallColor color = indexer.detectColorInSlot(slot);
                    System.out.println("Ball indexed in slot " + slot + ": " + color);
                    System.out.println("Total balls: " + indexer.getBallCount());

                    indexer.setState(IndexerState.IDLE);
                }),

                // Rotate to next slot if not full
                Commands.either(
                        ROTATE_NEXT.get(),
                        Commands.runOnce(() -> System.out.println("Indexer is now full!")),
                        () -> !indexer.isFull()
                )
        );

        //------------------------PREPARE SPECIFIC COLOR---------------------//

        /**
         * Rotates indexer to position green ball at current slot for dispensing
         */
        PREPARE_GREEN_BALL = () -> prepareBall(BallColor.GREEN);

        /**
         * Rotates indexer to position green ball at current slot for dispensing
         */
        PREPARE_PURPLE_BALL = () -> prepareBall(BallColor.PURPLE);
        //----------------------DISPENSING-----------------------//

        /**
         * Dispenses the ball from the current slot
         */
        DISPENSE = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();
                    BallColor color = indexer.getBallColorInSlot(slot);
                    System.out.println("Dispensing " + color + " ball from slot " + slot + "...");
                    indexer.setState(IndexerState.DISPENSING);
                }),

                // Open gate
                Commands.runOnce(indexer::openGate),
                Commands.waitSeconds(GATE_DELAY),

                // Rotate to push ball out
                Commands.runOnce(indexer::dispenseBall),
                Commands.waitSeconds(DISPENSE_DURATION),

                // Stop rotation
                Commands.runOnce(indexer::stopRotor),
                Commands.waitSeconds(VERIFICATION_DELAY),

                // Verify ball was dispensed
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();

                    if (!indexer.isBallDispensed(slot)) {
                        System.out.println("WARNING: Ball still detected in slot " + slot + " after dispense!");
                        System.out.println("Ball may be stuck. Consider running REJECT command.");
                        indexer.setState(IndexerState.ERROR);
                    } else {
                        System.out.println("Ball dispensed successfully from slot " + slot);
                        indexer.getSlot(slot).clear(); // Clear the slot manually if needed
                        System.out.println("Remaining balls: " + indexer.getBallCount());
                        indexer.setState(IndexerState.IDLE);
                    }
                }),

                // Close gate
                Commands.runOnce(indexer::closeGate)
        );

        /**
         * Full sequence: prepare and dispense green ball
         */
        DISPENSE_GREEN = () -> Commands.sequence(
                PREPARE_GREEN_BALL.get(),
                DISPENSE.get()
        );

        /**
         * Full sequence: prepare and dispense purple ball
         */
        DISPENSE_PURPLE = () -> Commands.sequence(
                PREPARE_GREEN_BALL.get(),
                DISPENSE.get()
        );

        //---------------------REJECTION-------------------

        /**
         * Rejects the ball from the current slot
         * Opens gate and reverses rotation to eject ball
         */
        REJECT_CURRENT_BALL = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();
                    BallColor color = indexer.getBallColorInSlot(slot);
                    System.out.println("Rejecting " + color + " ball from slot " + slot + "...");
                    indexer.setState(IndexerState.REJECTING);
                }),

                // Open gate
                Commands.runOnce(indexer::openGate),
                Commands.waitSeconds(GATE_DELAY),

                // Rotate backwards to eject
                Commands.runOnce(() -> indexer.rotateBySlots(-1)),
                Commands.waitSeconds(REJECT_DURATION),

                // Stop and verify
                Commands.runOnce(indexer::stopRotor),
                Commands.waitSeconds(VERIFICATION_DELAY),

                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();

                    if (indexer.isBallInSlot(slot)) {
                        System.out.println("WARNING: Ball still in slot " + slot + " after rejection attempt!");
                        indexer.setState(IndexerState.ERROR);
                    } else {
                        System.out.println("Ball rejected successfully from slot " + slot);
                        indexer.getSlot(slot).clear();
                        indexer.setState(IndexerState.IDLE);
                    }
                }),

                // Close gate and rotate back
                Commands.runOnce(indexer::closeGate),
                Commands.runOnce(() -> indexer.rotateBySlots(1))
        );

        //---------------------UTIL.---------------------------------//

        /**
         * Stops all indexer motion immediately
         */
        STOP = () -> Commands.runOnce(() -> {
            indexer.stopRotor();
            indexer.closeGate();
            indexer.setState(IndexerState.IDLE);
            System.out.println("Indexer stopped");
        });

        /**
         * Resets indexer to initial state and recalibrates
         */
        RESET = () -> Commands.sequence(
                Commands.runOnce(() -> System.out.println("Resetting indexer...")),
                Commands.runOnce(indexer::reset),
                CALIBRATE.get(),
                Commands.runOnce(() -> System.out.println("Indexer reset complete"))
        );
    }

    //--------------------------------HELPER METHODS--------------------------------//

    /**
     * Helper method to prepare a ball of specific color for dispensing
     */
    private static Command prepareBall(BallColor targetColor) {
        Indexer indexer = Indexer.getInstance();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("Searching for " + targetColor + " ball...");

                    // Find the slot containing target color
                    int slot = indexer.findBallSlot(targetColor);

                    if (slot == -1) {
                        System.out.println("ERROR: No " + targetColor + " ball found in indexer!");
                        System.out.println("Available balls:");
                        for (int i = 0; i < 3; i++) {
                            System.out.println("  Slot " + i + ": " + indexer.getSlot(i));
                        }
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    System.out.println(targetColor + " ball found in slot " + slot);

                    // Rotate to that slot if not already there
                    if (indexer.getCurrentSlot() != slot) {
                        System.out.println("Rotating from slot " + indexer.getCurrentSlot() + " to slot " + slot);
                        indexer.rotateToSlot(slot);
                    } else {
                        System.out.println("Already at correct slot " + slot);
                    }
                }),

                // Wait for rotation to complete
                Commands.waitSeconds(ROTATION_DURATION * 2), // Allow time for rotation
                Commands.runOnce(indexer::stopRotor),
                Commands.waitSeconds(VERIFICATION_DELAY),

                // Verify correct color at current slot
                Commands.runOnce(() -> {
                    int currentSlot = indexer.getCurrentSlot();
                    BallColor detectedColor = indexer.detectColorInSlot(currentSlot);

                    if (detectedColor == targetColor) {
                        System.out.println("Confirmed: " + targetColor + " ball ready at slot " + currentSlot);
                        indexer.setState(IndexerState.IDLE);
                    } else {
                        System.out.println("WARNING: Expected " + targetColor + " but detected " + detectedColor + " at slot " + currentSlot);
                        System.out.println("Ball tracking may be out of sync. Consider resetting indexer.");
                        indexer.setState(IndexerState.ERROR);
                    }
                })
        );
    }
}