package org.firstinspires.ftc.teamcode.indexer.commands;

import static org.firstinspires.ftc.teamcode.indexer.IndexerConstants.*;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;
import org.firstinspires.ftc.teamcode.indexer.Enums.IndexerState;
import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.ATVision;

import java.util.function.Supplier;

public class IndexerCommands {

    // ==================== SUPPLIER COMMANDS (Internal Use) ====================

    // Core Operations
    private static final Supplier<Command> RESET;
    private static final Supplier<Command> INDEX_ARTIFACT;
    private static final Supplier<Command> INDEX_AT_TRANSFER_POSITION;
    private static final Supplier<Command> ROTATE_NEXT;

    // Transfer Operations
    private static final Supplier<Command> TRANSFER_CURRENT_ARTIFACT;
    private static final Supplier<Command> TRANSFER_GREEN;
    private static final Supplier<Command> TRANSFER_PURPLE;

    // Sequence Operations
    private static final Supplier<Command> TRANSFER_SEQUENCE_PPG;
    private static final Supplier<Command> TRANSFER_SEQUENCE_PGP;
    private static final Supplier<Command> TRANSFER_SEQUENCE_GPP;
    private static final Supplier<Command> TRANSFER_SEQUENCE_AUTO;

    // Utility
    private static final Supplier<Command> STOP;

    // ==================== STATIC COMMANDS (For Control Mapping) ====================

    // Primary Actions
    public static Command INDEX;           // Index artifact into current slot
    public static Command TRANSFER;        // Transfer artifact from current slot
    public static Command INDEX_AT_TRANSFER; // Move to transfer, then index in other slots
    public static Command NEXT_SLOT;       // Rotate to next slot

    // Color-Specific Transfers
    public static Command TRANSFER_GREEN_ARTIFACT;
    public static Command TRANSFER_PURPLE_ARTIFACT;

    // Sequence Transfers
    public static Command SEQUENCE_PPG;
    public static Command SEQUENCE_PGP;
    public static Command SEQUENCE_GPP;
    public static Command SEQUENCE_AUTO;   // Auto-detect motif and transfer

    // Utility
    public static Command STOP_INDEXER;
    public static Command RESET_INDEXER;

    static {
        Indexer indexer = Indexer.getInstance();

        //===================== SUPPLIER COMMAND DEFINITIONS =====================//

        RESET = () -> Commands.sequence(
                Commands.runOnce(() -> Log.i("IndexerCommands", "Resetting indexer...")),
                Commands.runOnce(indexer::reset),
                Commands.runOnce(indexer::calibrateRotor),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> {
                    if (indexer.isCalibrated()) {
                        Log.i("IndexerCommands", "Reset complete - ready at slot 0");
                    } else {
                        Log.e("IndexerCommands", "Reset failed - calibration error");
                        indexer.setState(IndexerState.ERROR);
                    }
                })
        );

        ROTATE_NEXT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    Log.i("IndexerCommands", "Rotating to next slot...");
                    indexer.rotateToNextSlot();
                }),
                Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT),
                Commands.runOnce(() -> {
                    if (!indexer.isAtTargetPosition()) {
                        Log.e("IndexerCommands", "Rotation timeout!");
                        indexer.setState(IndexerState.ERROR);
                    } else {
                        Log.i("IndexerCommands", "Now at slot " + indexer.getCurrentSlot());
                    }
                })
        );

        INDEX_ARTIFACT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    if (indexer.isFull()) {
                        Log.e("IndexerCommands", "Indexer is full! Cannot index more artifacts.");
                        return;
                    }

                    if (!indexer.isCurrentSlotAvailable()) {
                        Log.e("IndexerCommands", "Current slot " + indexer.getCurrentSlot() + " is occupied!");
                        return;
                    }

                    Log.i("IndexerCommands", "Waiting for artifact in slot " + indexer.getCurrentSlot() + "...");
                    indexer.setState(IndexerState.INDEXING);
                }),

                // Wait for artifact to enter slot
                Commands.waitUntil(() -> indexer.isArtifactInSlot(indexer.getCurrentSlot()))
                        .withTimeout((long) 5.0),

                // Wait for artifact to settle
                Commands.waitSeconds(ARTIFACT_SETTLE_TIME),

                // Update tracking and verify
                Commands.runOnce(() -> {
                    indexer.updateArtifactTracking();

                    int slot = indexer.getCurrentSlot();
                    if (!indexer.isArtifactInSlot(slot)) {
                        Log.e("IndexerCommands", "Artifact not detected in slot " + slot + " after settle time");
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    ArtifactColor color = indexer.detectColorInSlot(slot);
                    Log.i("IndexerCommands", "Artifact indexed in slot " + slot + ": " + color);
                    Log.i("IndexerCommands", "Total artifacts: " + indexer.getArtifactCount());

                    indexer.setState(IndexerState.IDLE);
                }),

                // Rotate to next slot if not full
                Commands.either(
                        ROTATE_NEXT.get(),
                        Commands.runOnce(() -> Log.i("IndexerCommands", "Indexer is now full!")),
                        () -> !indexer.isFull()
                )
        );

        INDEX_AT_TRANSFER_POSITION = () -> Commands.sequence(
                // First, move to transfer position and stay there
                Commands.runOnce(() -> {
                    Log.i("IndexerCommands", "Moving to transfer position for indexing...");
                    indexer.rotateToTransferPosition();
                }),
                Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT),

                Commands.runOnce(() -> {
                    if (!indexer.isAtTargetPosition()) {
                        Log.e("IndexerCommands", "Failed to reach transfer position!");
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }
                    Log.i("IndexerCommands", "At transfer position - ready to index artifacts in other slots");
                }),

                // Return to slot alignment to begin indexing
                Commands.runOnce(() -> indexer.rotateToSlot(indexer.getCurrentSlot())),
                Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT),

                // Now index artifact normally (will use other slots)
                INDEX_ARTIFACT.get()
        );
        TRANSFER_CURRENT_ARTIFACT = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    int slot = indexer.getCurrentSlot();
                    ArtifactColor color = indexer.getArtifactColorInSlot(slot);
                    Log.i("IndexerCommands", "Transferring " + color + " artifact from slot " + slot + "...");
                    indexer.setState(IndexerState.TRANSFERRING);
                }),

                // Snapshot the slot we're transferring from
                Commands.defer(() -> {
                    final int transferSlot = indexer.getCurrentSlot();

                    return Commands.sequence(
                            // Rotate to transfer position
                            Commands.runOnce(indexer::rotateToTransferPosition),
                            Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT),

                            // Start hopper to kick artifact
                            Commands.runOnce(indexer::startHopper),
                            Commands.waitSeconds(HOPPER_KICK_DURATION),
                            Commands.runOnce(indexer::stopHopper),

                            // Wait and verify transfer
                            Commands.waitSeconds(VERIFICATION_DELAY),
                            Commands.runOnce(() -> {
                                indexer.updateArtifactTracking();

                                if (!indexer.isArtifactTransferred(transferSlot)) {
                                    Log.e("IndexerCommands", "Artifact still in slot " + transferSlot + " after transfer!");
                                    indexer.setState(IndexerState.ERROR);
                                } else {
                                    Log.i("IndexerCommands", "Artifact transferred successfully from slot " + transferSlot);
                                    indexer.getSlot(transferSlot).clear();
                                    Log.i("IndexerCommands", "Remaining artifacts: " + indexer.getArtifactCount());
                                    indexer.setState(IndexerState.IDLE);
                                }
                            }),

                            // Return to slot alignment
                            Commands.runOnce(() -> indexer.rotateToSlot(transferSlot)),
                            Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT)
                    );
                })
        );

        TRANSFER_GREEN = () -> prepareAndTransferArtifact(ArtifactColor.GREEN);
        TRANSFER_PURPLE = () -> prepareAndTransferArtifact(ArtifactColor.PURPLE);

        TRANSFER_SEQUENCE_PPG = () -> Commands.sequence(
                Commands.runOnce(() -> Log.i("IndexerCommands", "Starting PPG sequence transfer...")),
                TRANSFER_PURPLE.get(),
                TRANSFER_PURPLE.get(),
                TRANSFER_GREEN.get(),
                Commands.runOnce(() -> Log.i("IndexerCommands", "PPG sequence complete"))
        );

        TRANSFER_SEQUENCE_PGP = () -> Commands.sequence(
                Commands.runOnce(() -> Log.i("IndexerCommands", "Starting PGP sequence transfer...")),
                TRANSFER_PURPLE.get(),
                TRANSFER_GREEN.get(),
                TRANSFER_PURPLE.get(),
                Commands.runOnce(() -> Log.i("IndexerCommands", "PGP sequence complete"))
        );

        TRANSFER_SEQUENCE_GPP = () -> Commands.sequence(
                Commands.runOnce(() -> Log.i("IndexerCommands", "Starting GPP sequence transfer...")),
                TRANSFER_GREEN.get(),
                TRANSFER_PURPLE.get(),
                TRANSFER_PURPLE.get(),
                Commands.runOnce(() -> Log.i("IndexerCommands", "GPP sequence complete"))
        );

        TRANSFER_SEQUENCE_AUTO = () -> Commands.sequence(
                Commands.runOnce(() -> Log.i("IndexerCommands", "Detecting artifact motif from vision...")),

                Commands.defer(() -> {
                    // Get motif from vision
                    String motif = getMotifFromVision();

                    Log.i("IndexerCommands", "Detected motif: " + motif);

                    // Execute appropriate sequence based on motif
                    switch (motif) {
                        case "PPG":
                            return TRANSFER_SEQUENCE_PPG.get();
                        case "PGP":
                            return TRANSFER_SEQUENCE_PGP.get();
                        case "GPP":
                            return TRANSFER_SEQUENCE_GPP.get();
                        default:
                            return Commands.runOnce(() -> {
                                Log.e("IndexerCommands", "Unknown motif detected: " + motif);
                                Log.e("IndexerCommands", "Defaulting to PPG sequence");
                            }).andThen(TRANSFER_SEQUENCE_PPG.get());
                    }
                })
        );

        STOP = () -> Commands.runOnce(() -> {
            indexer.stopRotor();
            indexer.stopHopper();
            indexer.setState(IndexerState.IDLE);
            Log.i("IndexerCommands", "Indexer stopped");
        });
    }

    //===================== STATIC COMMAND DEFINITIONS (For Control Mapping) =====================//

    static {
        Indexer indexer = Indexer.getInstance();

        // Primary indexer actions
        INDEX = Commands.deferredProxy(() -> {
            if (indexer.isFull()) {
                Log.e("IndexerCommands", "Cannot index - indexer is full");
                return Commands.none();
            }
            return Commands.defer(INDEX_ARTIFACT, indexer);
        });

        INDEX_AT_TRANSFER = Commands.deferredProxy(() -> {
            if (indexer.isFull()) {
                Log.e("IndexerCommands", "Cannot index - indexer is full");
                return Commands.none();
            }
            return Commands.defer(INDEX_AT_TRANSFER_POSITION, indexer);
        });

        TRANSFER = Commands.deferredProxy(() -> {
            if (indexer.isEmpty()) {
                Log.e("IndexerCommands", "Cannot transfer - indexer is empty");
                return Commands.none();
            }
            return Commands.defer(TRANSFER_CURRENT_ARTIFACT, indexer);
        });

        NEXT_SLOT = Commands.deferredProxy(() -> Commands.defer(ROTATE_NEXT, indexer));

        // Color-specific transfers
        TRANSFER_GREEN_ARTIFACT = Commands.deferredProxy(() -> {
            if (indexer.findArtifactSlot(ArtifactColor.GREEN) == -1) {
                Log.e("IndexerCommands", "No green artifact in indexer");
                return Commands.none();
            }
            return Commands.defer(TRANSFER_GREEN, indexer);
        });

        TRANSFER_PURPLE_ARTIFACT = Commands.deferredProxy(() -> {
            if (indexer.findArtifactSlot(ArtifactColor.PURPLE) == -1) {
                Log.e("IndexerCommands", "No purple artifact in indexer");
                return Commands.none();
            }
            return Commands.defer(TRANSFER_PURPLE, indexer);
        });

        // Sequence transfers
        SEQUENCE_PPG = Commands.deferredProxy(() -> Commands.defer(TRANSFER_SEQUENCE_PPG, indexer));
        SEQUENCE_PGP = Commands.deferredProxy(() -> Commands.defer(TRANSFER_SEQUENCE_PGP, indexer));
        SEQUENCE_GPP = Commands.deferredProxy(() -> Commands.defer(TRANSFER_SEQUENCE_GPP, indexer));

        SEQUENCE_AUTO = Commands.deferredProxy(() -> {
            if (indexer.isEmpty()) {
                Log.e("IndexerCommands", "Cannot transfer sequence - indexer is empty");
                return Commands.none();
            }
            return Commands.defer(TRANSFER_SEQUENCE_AUTO, indexer);
        });

        // Utility commands
        STOP_INDEXER = Commands.deferredProxy(() -> Commands.defer(STOP, indexer));
        RESET_INDEXER = Commands.deferredProxy(() -> Commands.defer(RESET, indexer));
    }

    //================================ HELPER METHODS ================================//

    /**
     * Helper method to prepare and transfer an artifact of specific color
     */
    private static Command prepareAndTransferArtifact(ArtifactColor targetColor) {
        Indexer indexer = Indexer.getInstance();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    Log.i("IndexerCommands", "Searching for " + targetColor + " artifact...");

                    int targetSlot = indexer.findArtifactSlot(targetColor);

                    if (targetSlot == -1) {
                        Log.e("IndexerCommands", "No " + targetColor + " artifact found in indexer!");
                        Log.e("IndexerCommands", "Available artifacts:");
                        for (int i = 0; i < 3; i++) {
                            Log.e("IndexerCommands", "  Slot " + i + ": " + indexer.getSlot(i));
                        }
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    int currentSlot = indexer.getCurrentSlot();
                    if (currentSlot != targetSlot) {
                        int stepsForward = (targetSlot - currentSlot + 3) % 3;
                        Log.i("IndexerCommands", "Rotating " + stepsForward + " slot(s) forward from slot "
                                + currentSlot + " to slot " + targetSlot);
                        indexer.rotateToSlot(targetSlot);
                    } else {
                        Log.i("IndexerCommands", "Already at correct slot " + targetSlot);
                    }
                }),

                // Wait for rotation to complete
                Commands.waitUntil(indexer::isAtTargetPosition).withTimeout((long) ROTATION_TIMEOUT),

                Commands.runOnce(() -> {
                    if (!indexer.isAtTargetPosition()) {
                        Log.e("IndexerCommands", "Rotation timeout while preparing artifact!");
                        indexer.setState(IndexerState.ERROR);
                        return;
                    }

                    // Verify correct color at current slot
                    int slot = indexer.getCurrentSlot();
                    ArtifactColor detected = indexer.detectColorInSlot(slot);

                    if (detected == targetColor) {
                        Log.i("IndexerCommands", "Confirmed: " + targetColor + " artifact ready at slot " + slot);
                    } else {
                        Log.e("IndexerCommands", "Expected " + targetColor + " but detected " + detected + " at slot " + slot);
                        Log.e("IndexerCommands", "Artifact tracking may be out of sync. Consider resetting indexer.");
                        indexer.setState(IndexerState.ERROR);
                    }
                }),

                // Transfer the artifact
                TRANSFER_CURRENT_ARTIFACT.get()
        );
    }

    /**
     * Gets artifact motif from vision system
     * Uses ATVision.getMotif() which returns the AprilTag metadata name (e.g., "PPG", "PGP", "GPP")
     *
     * @return String motif ("PPG", "PGP", "GPP", or "UNKNOWN")
     */
    private static String getMotifFromVision() {
        ATVision vision = ATVision.getInstance();
        String motif = vision.getMotif();

        Log.i("IndexerCommands", "Vision detected motif: " + motif);

        // Validate motif
        if (motif == null || motif.equals("UNKNOWN")) {
            Log.e("IndexerCommands", "No valid motif detected from vision, defaulting to PPG");
            return "PPG"; // Default fallback
        }

        return motif;
    }
}