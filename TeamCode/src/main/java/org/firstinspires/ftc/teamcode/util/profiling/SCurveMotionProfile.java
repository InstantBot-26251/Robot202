package org.firstinspires.ftc.teamcode.util.profiling;


public class SCurveMotionProfile {

    // Motion constraints
    private double maxVelocity;        // Maximum velocity (units/sec)
    private double maxAcceleration;    // Maximum acceleration (units/sec²)
    private double jerkTime;           // Time for jerk smoothing (seconds)

    // Profile state
    private double startPosition;
    private double targetPosition;
    private double totalDistance;
    private double profileDuration;

    // Phase durations
    private double accelTime;   // Duration of acceleration phase
    private double coastTime;   // Duration of constant velocity phase
    private double decelTime;   // Duration of deceleration phase

    // Direction flag
    private boolean movingForward;


    public SCurveMotionProfile(double maxVelocity, double maxAcceleration, double jerkTime) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
        this.jerkTime = Math.max(0.01, jerkTime); // Minimum jerk time to avoid division by zero

        // Initialize to zero
        this.startPosition = 0;
        this.targetPosition = 0;
        this.totalDistance = 0;
        this.profileDuration = 0;
    }


    public SCurveMotionProfile(double maxVelocity, double maxAcceleration) {
        this(maxVelocity, maxAcceleration, 0.15);
    }

    public void initialize(double start, double target) {
        this.startPosition = start;
        this.targetPosition = target;
        this.totalDistance = Math.abs(target - start);
        this.movingForward = target >= start;

        // Calculate the profile phases
        calculateProfile();
    }

    private void calculateProfile() {
        // Time to reach max velocity with S-curve smoothing
        accelTime = maxVelocity / maxAcceleration + jerkTime;
        decelTime = accelTime; // Symmetric acceleration/deceleration

        // Distance covered during acceleration and deceleration phases
        double accelDistance = 0.5 * maxVelocity * accelTime;
        double decelDistance = accelDistance;

        // Check if we have enough distance to reach max velocity
        if (accelDistance + decelDistance > totalDistance) {
            // Triangular profile - we never reach max velocity
            // Solve for the peak velocity we can actually reach
            double peakVelocity = Math.sqrt(totalDistance * maxAcceleration);

            // Recalculate times for triangular profile
            accelTime = peakVelocity / maxAcceleration + jerkTime;
            decelTime = accelTime;
            coastTime = 0;

            // Store the actual max velocity we'll reach
            maxVelocity = peakVelocity;
        } else {
            // Trapezoidal profile - we reach max velocity and coast
            double coastDistance = totalDistance - accelDistance - decelDistance;
            coastTime = coastDistance / maxVelocity;
        }

        // Total time for the motion
        profileDuration = accelTime + coastTime + decelTime;
    }


    public double getPosition(double elapsedTime) {
        double t = Math.min(elapsedTime, profileDuration);
        double position;

        if (t <= accelTime) {
            // Acceleration phase with S-curve
            position = getAccelPhasePosition(t);

        } else if (t <= accelTime + coastTime) {
            // Constant velocity phase
            double accelDist = 0.5 * maxVelocity * accelTime;
            double coastDist = maxVelocity * (t - accelTime);
            position = accelDist + coastDist;

        } else {
            // Deceleration phase with S-curve
            double accelDist = 0.5 * maxVelocity * accelTime;
            double coastDist = maxVelocity * coastTime;
            double decelDist = getDecelPhasePosition(t - accelTime - coastTime);
            position = accelDist + coastDist + decelDist;
        }

        // Apply direction and offset
        if (movingForward) {
            return startPosition + position;
        } else {
            return startPosition - position;
        }
    }

    public double getVelocity(double elapsedTime) {
        double t = Math.min(elapsedTime, profileDuration);
        double velocity;

        if (t <= accelTime) {
            // Accelerating with S-curve
            double tNorm = t / accelTime;
            velocity = maxVelocity * smoothStep(tNorm);

        } else if (t <= accelTime + coastTime) {
            // Constant velocity
            velocity = maxVelocity;

        } else {
            // Decelerating with S-curve
            double tDecel = t - accelTime - coastTime;
            double tNorm = tDecel / decelTime;
            velocity = maxVelocity * (1.0 - smoothStep(tNorm));
        }

        // Apply direction
        return movingForward ? velocity : -velocity;
    }

    public double getAcceleration(double elapsedTime) {
        double t = Math.min(elapsedTime, profileDuration);
        double acceleration;

        if (t <= accelTime) {
            // Accelerating
            double tNorm = t / accelTime;
            acceleration = (maxVelocity / accelTime) * smoothStepDerivative(tNorm);

        } else if (t <= accelTime + coastTime) {
            // Coasting - no acceleration
            acceleration = 0;

        } else {
            // Decelerating
            double tDecel = t - accelTime - coastTime;
            double tNorm = tDecel / decelTime;
            acceleration = -(maxVelocity / decelTime) * smoothStepDerivative(tNorm);
        }

        // Apply direction
        return movingForward ? acceleration : -acceleration;
    }

    /**
     * Calculates position during acceleration phase
     */
    private double getAccelPhasePosition(double t) {
        double tNorm = t / accelTime;
        double sCurve = smoothStep(tNorm);
        return 0.5 * maxVelocity * accelTime * sCurve;
    }

    /**
     * Calculates position during deceleration phase
     */
    private double getDecelPhasePosition(double tDecel) {
        double tNorm = tDecel / decelTime;
        double sCurve = smoothStep(tNorm);
        return maxVelocity * tDecel - (0.5 * maxVelocity * decelTime * sCurve);
    }

    private double smoothStep(double t) {
        t = Math.max(0, Math.min(1, t)); // Clamp to [0, 1]
        return t * t * (3.0 - 2.0 * t);
    }


    private double smoothStepDerivative(double t) {
        t = Math.max(0, Math.min(1, t)); // Clamp to [0, 1]
        return 6.0 * t * (1.0 - t);
    }


    public boolean isFinished(double elapsedTime) {
        return elapsedTime >= profileDuration;
    }

    // ==================== GETTERS ====================

    /**
     * @return Total duration of the motion profile in seconds
     */
    public double getDuration() {
        return profileDuration;
    }

    /**
     * @return Duration of acceleration phase in seconds
     */
    public double getAccelTime() {
        return accelTime;
    }

    /**
     * @return Duration of coast phase in seconds
     */
    public double getCoastTime() {
        return coastTime;
    }

    /**
     * @return Duration of deceleration phase in seconds
     */
    public double getDecelTime() {
        return decelTime;
    }

    /**
     * @return Starting position of the profile
     */
    public double getStartPosition() {
        return startPosition;
    }

    /**
     * @return Target position of the profile
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * @return Total distance to travel
     */
    public double getTotalDistance() {
        return totalDistance;
    }

    /**
     * @return Maximum velocity constraint
     */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * @return Maximum acceleration constraint
     */
    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * @return Jerk time smoothing parameter
     */
    public double getJerkTime() {
        return jerkTime;
    }

    // ==================== SETTERS ====================

    /**
     * Updates motion constraints. Call initialize() after changing constraints.
     *
     * @param maxVelocity Maximum velocity in units/sec
     * @param maxAcceleration Maximum acceleration in units/sec²
     * @param jerkTime Jerk smoothing time in seconds
     */
    public void setConstraints(double maxVelocity, double maxAcceleration, double jerkTime) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
        this.jerkTime = Math.max(0.01, jerkTime);
    }

    /**
     * @return String representation of the profile for debugging
     */
    @Override
    public String toString() {
        return String.format(
                "SCurveProfile[start=%.1f, target=%.1f, dist=%.1f, duration=%.2fs, " +
                        "phases=(%.2fs accel, %.2fs coast, %.2fs decel)]",
                startPosition, targetPosition, totalDistance, profileDuration,
                accelTime, coastTime, decelTime
        );
    }
}