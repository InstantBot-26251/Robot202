package org.firstinspires.ftc.teamcode.indexer;

import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;

/**
 * Represents a single slot in the indexer
 */

public class ArtifactSlot {
    public ArtifactColor color = ArtifactColor.NONE;
    public boolean hasConfirmedArtifact = false;
    public int detectionConfidence = 0; // 0 = none, 1 = one sensor, 2 = both sensors

    public void clear() {
        color = ArtifactColor.NONE;
        hasConfirmedArtifact = false;
        detectionConfidence = 0;
    }

    @Override
    public String toString() {
        if (!hasConfirmedArtifact) return "EMPTY";
        String confidenceStr = detectionConfidence == 2 ? "BOTH" : "ONE";
        return color + " (" + confidenceStr + " sensor)";
    }
}
