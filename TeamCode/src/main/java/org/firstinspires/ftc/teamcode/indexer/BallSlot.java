package org.firstinspires.ftc.teamcode.indexer;

import org.firstinspires.ftc.teamcode.indexer.Enums.BallColor;

/**
 * Represents a single slot in the indexer
 */

public class BallSlot {
    public BallColor color = BallColor.NONE;
    public boolean hasConfirmedBall = false;
    public int detectionConfidence = 0; // 0 = none, 1 = one sensor, 2 = both sensors

    public void clear() {
        color = BallColor.NONE;
        hasConfirmedBall = false;
        detectionConfidence = 0;
    }

    @Override
    public String toString() {
        if (!hasConfirmedBall) return "EMPTY";
        String confidenceStr = detectionConfidence == 2 ? "BOTH" : "ONE";
        return color + " (" + confidenceStr + " sensor)";
    }
}
