package org.firstinspires.ftc.teamcode.indexer.commands;

import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.BALL_SETTLE_TIME;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.DISPENSE_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.HOPPER_DELAY;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.REJECT_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.ROTATION_DURATION;
import static org.firstinspires.ftc.teamcode.indexer.commands.CommandsConstants.VERIFICATION_DELAY;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;
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

        // Calibrates the indexer rotor position
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

        // Rotates to the next slot

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

        /*
         * Indexes a single ball into the current slot. Algorithm:
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
                Commands.waitUntil(() -> indexer.isArtifactInSlot(indexer.getCurrentSlot()))
                        .withTimeout((long) 5.0),

                // Wait for ball to fully enter (both sensors)
                Commands.waitSeconds(BALL_SETTLE_TIME),
                Commands.waitUntil(() -> indexer.isArtifactInSlot(indexer.getCurrentSlot()))
                        .withTimeout((long) 2.0),

                // Verify and log ball
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();

                    if (!indexer.isArtifactInSlot(slot)) {
                        System.out.println("WARNING: Ball not fully detected in slot " + slot);
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    ArtifactColor color = indexer.detectColorInSlot(slot);
                    System.out.println("Ball indexed in slot " + slot + ": " + color);
                    System.out.println("Total balls: " + indexer.getArtifactCount());

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

        // Rotates indexer to position green ball at current slot for dispensing
        PREPARE_GREEN_BALL = () -> prepareBall(ArtifactColor.GREEN);

        // Rotates indexer to position purple ball at current slot for dispensing

        PREPARE_PURPLE_BALL = () -> prepareBall(ArtifactColor.PURPLE);
        //----------------------DISPENSING-----------------------//

        // Dispenses the ball from the current slot
        DISPENSE = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();
                    ArtifactColor color = indexer.getArtifactColorInSlot(slot);
                    System.out.println("Dispensing " + color + " ball from slot " + slot + "...");
                    indexer.setState(IndexerState.DISPENSING);
                }),

                // Open gate
                Commands.runOnce(indexer::startHopper),
                Commands.waitSeconds(HOPPER_DELAY),

                // Rotate to push ball out
                Commands.runOnce(indexer::dispenseArtifact),
                Commands.waitSeconds(DISPENSE_DURATION),

                // Stop rotation
                Commands.runOnce(indexer::stopRotor),
                Commands.waitSeconds(VERIFICATION_DELAY),

                // Verify ball was dispensed
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();

                    if (!indexer.isArtifactDispensed(slot)) {
                        System.out.println("WARNING: Ball still detected in slot " + slot + " after dispense!");
                        System.out.println("Ball may be stuck. Consider running REJECT command.");
                        indexer.setState(IndexerState.ERROR);
                    } else {
                        System.out.println("Ball dispensed successfully from slot " + slot);
                        indexer.getSlot(slot).clear(); // Clear the slot manually if needed
                        System.out.println("Remaining balls: " + indexer.getArtifactCount());
                        indexer.setState(IndexerState.IDLE);
                    }
                }),

                // Close gate
                Commands.runOnce(indexer::stopHopper)
        );

        // Full sequence: prepare and dispense green ball
        DISPENSE_GREEN = () -> Commands.sequence(
                PREPARE_GREEN_BALL.get(),
                DISPENSE.get()
        );

        // Full sequence: prepare and dispense purple ball
        DISPENSE_PURPLE = () -> Commands.sequence(
                PREPARE_PURPLE_BALL.get(),
                DISPENSE.get()
        );

        //---------------------REJECTION-------------------

        /* Rejection Algorithm:
         * Rejects the ball from the current slot
         * Opens gate and reverses rotation to eject ball
         */
        REJECT_CURRENT_BALL = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();
                    ArtifactColor color = indexer.getArtifactColorInSlot(slot);
                    System.out.println("Rejecting " + color + " ball from slot " + slot + "...");
                    indexer.setState(IndexerState.REJECTING);
                }),
                // Snapshot the slot we intend to eject
                Commands.runOnce(() -> indexer.getSlot(indexer.getCurrentSlot())),
                Commands.defer(() -> {
                    final int snapshotSlot = indexer.getCurrentSlot();
                    return Commands.sequence(
                            // Start Hopper
                            Commands.runOnce(indexer::startHopper),
                            Commands.waitSeconds(HOPPER_DELAY),

                            // Rotate backwards to eject
                            Commands.runOnce(() -> indexer.rotateBySlots(-1)),
                            Commands.waitSeconds(REJECT_DURATION),
                            Commands.runOnce(indexer::stopRotor),
                            Commands.waitSeconds(VERIFICATION_DELAY),

                            // verify the ball in the og slot was actually ejected
                            Commands.runOnce(() -> {
                                if (indexer.isArtifactInSlot(snapshotSlot)) {
                                    System.out.println("WARNING: Ball still in slot " + snapshotSlot + " after rejection!");
                                    indexer.setState(IndexerState.ERROR);
                                } else {
                                    System.out.println("Ball rejected successfully from slot " + snapshotSlot);
                                    indexer.getSlot(snapshotSlot).clear();
                                    indexer.setState(IndexerState.IDLE);
                                }
                            }),


                            // Stop and verify
                            Commands.runOnce(indexer::stopRotor),
                            Commands.waitSeconds(VERIFICATION_DELAY),

                            Commands.runOnce(() -> {
                                int slot = indexer.getCurrentSlot();

                                if (indexer.isArtifactInSlot(slot)) {
                                    System.out.println("WARNING: Ball still in slot " + slot + " after rejection attempt!");
                                    indexer.setState(IndexerState.ERROR);
                                } else {
                                    System.out.println("Ball rejected successfully from slot " + slot);
                                    indexer.getSlot(slot).clear();
                                    indexer.setState(IndexerState.IDLE);
                                }
                            }),

                            // stop hopper and return precisely to the original alignment
                            Commands.runOnce(indexer::stopHopper),
                            Commands.runOnce(() -> indexer.rotateToSlot(snapshotSlot)),
                            Commands.waitSeconds(ROTATION_DURATION),
                            Commands.runOnce(indexer::stopRotor)
                    );
                })
        );

        //---------------------UTIL.---------------------------------//

        // Stops all indexer motion immediately

        STOP = () -> Commands.runOnce(() -> {
            indexer.stopRotor();
            indexer.stopHopper();
            indexer.setState(IndexerState.IDLE);
            System.out.println("Indexer stopped");
        });

        // Resets indexer to initial state and recalibrates

        RESET = () -> Commands.sequence(
                Commands.runOnce(() -> System.out.println("Resetting indexer...")),
                Commands.runOnce(indexer::reset),
                CALIBRATE.get(),
                Commands.runOnce(() -> System.out.println("Indexer reset complete"))
        );
    }

    //--------------------------------HELPER METHODS--------------------------------//

    // Helper method to prepare a ball of specific color for dispensing
    private static Command prepareBall(ArtifactColor targetColor) {
        Indexer indexer = Indexer.getInstance();

        int targetSlot = indexer.findArtifactSlot(targetColor);
        int currentSlot = indexer.getCurrentSlot();

        if (targetSlot == -1) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        System.out.println("ERROR: No " + targetColor + " ball found in indexer!");
                        System.out.println("Available balls:");
                        for (int i = 0; i < 3; i++) {
                            System.out.println("  Slot " + i + ": " + indexer.getSlot(i));
                        }
                        indexer.setState(IndexerState.ERROR);
                    })
            );
        }

        // Steps to rotate (0, 1, or 2). If 0, we'll still dwell for one ROTATION_DURATION.
        int steps = (targetSlot - currentSlot + 3) % 3;
        double rotationWaitSeconds = ROTATION_DURATION * (steps == 0 ? 1 : steps);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("Searching for " + targetColor + " ball...");
                    if (currentSlot != targetSlot) {
                        System.out.println("Rotating from slot " + currentSlot + " to slot " + targetSlot + " (" + steps + " step"
                                + (steps == 1 ? "" : "s") + ")");
                        indexer.rotateToSlot(targetSlot);
                    } else {
                        System.out.println("Already at correct slot " + targetSlot);
                    }
                }),

                // wait proportional to number of steps (or one duration if already aligned)
                Commands.waitSeconds(rotationWaitSeconds),

                Commands.runOnce(indexer::stopRotor),
                Commands.waitSeconds(VERIFICATION_DELAY),

                // Verify correct color at current slot
                Commands.runOnce(() -> {
                    int s = indexer.getCurrentSlot();
                    ArtifactColor detected = indexer.detectColorInSlot(s);
                    if (detected == targetColor) {
                        System.out.println("Confirmed: " + targetColor + " ball ready at slot " + s);
                        indexer.setState(IndexerState.IDLE);
                    } else {
                        System.out.println("WARNING: Expected " + targetColor + " but detected " + detected + " at slot " + s);
                        System.out.println("Ball tracking may be out of sync. Consider resetting indexer.");
                        indexer.setState(IndexerState.ERROR);
                    }
                })
        );
    }
}