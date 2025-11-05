package org.firstinspires.ftc.teamcode.indexer;

import org.firstinspires.ftc.teamcode.indexer.Enums.ArtifactColor;

/**
 * Represents a single slot in the indexer
 */
public class ArtifactSlot {
    private ArtifactColor color = ArtifactColor.NONE;
    private boolean hasConfirmedArtifact = false;
    private int detectionConfidence = 0; // 0 = none, 1 = one sensor, 2 = both sensors

    // Getters
    public ArtifactColor getColor() {
        return color;
    }

    public boolean hasConfirmedArtifact() {
        return hasConfirmedArtifact;
    }

    public int getDetectionConfidence() {
        return detectionConfidence;
    }

    // Setters
    public void setColor(ArtifactColor color) {
        this.color = color;
    }

    public void setHasConfirmedArtifact(boolean hasConfirmedArtifact) {
        this.hasConfirmedArtifact = hasConfirmedArtifact;
    }

    public void setDetectionConfidence(int detectionConfidence) {
        this.detectionConfidence = detectionConfidence;
    }

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