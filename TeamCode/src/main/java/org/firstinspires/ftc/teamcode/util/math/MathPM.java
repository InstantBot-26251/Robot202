package org.firstinspires.ftc.teamcode.util.math;

public class MathPM {

    // Gravity accel
    private static final double GRAVITY = 9.81;

    // Calculation Tolerances
    private static final double VELOCITY_TOLERANCE = 0.01; // m/s
    private static final double MAX_ITERATIONS = 1000;
    private static final double ANGLE_MIN = 0.0;
    private static final double ANGLE_MAX = 90.0;

    // Conversion constants
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double METERS_TO_INCHES = 1.0 / INCHES_TO_METERS;

    /**
     * Calculates the optimal launch angle for projectile motion
     * @param distance Horizontal distance to target (meters)
     * @param initialVelocity Initial velocity of projectile (m/s)
     * @param heightDifference Height difference between shooter and target (meters, positive if target is higher)
     * @return Launch angle in degrees, or -1 if no solution exists
     */
    public static double calculateLaunchAngle(double distance, double initialVelocity, double heightDifference) {
        // Using my equations
        double v2 = initialVelocity * initialVelocity;
        double term = GRAVITY * (GRAVITY * distance * distance + 2 * heightDifference * v2);
        double discriminant = v2 * v2 - term;

        // Check if solution exists
        if (discriminant < 0) {
            return -1; // No solution - target is too far or velocity too low
        }

        // Two possible angles - we typically want the lower one
        double sqrtDisc = Math.sqrt(discriminant);
        double angle1 = Math.atan((v2 - sqrtDisc) / (GRAVITY * distance));
        double angle2 = Math.atan((v2 + sqrtDisc) / (GRAVITY * distance));

        // Convert to degrees and return the lower angle
        double angle1Deg = Math.toDegrees(angle1);
        double angle2Deg = Math.toDegrees(angle2);

        // Return the angle that's positive and reasonable
        if (angle1Deg >= 0 && angle1Deg <= 90) {
            return angle1Deg;
        } else if (angle2Deg >= 0 && angle2Deg <= 90) {
            return angle2Deg;
        } else {
            return -1; // No valid solution
        }
    }

    /**
     * Simplified version assuming target is at same height as shooter
     * @param distance Horizontal distance to target (meters)
     * @param initialVelocity Initial velocity of projectile (m/s)
     * @return Launch angle in degrees
     */
    public static double calculateLaunchAngle(double distance, double initialVelocity) {
        return calculateLaunchAngle(distance, initialVelocity, 0);
    }

    /**
     * Calculates the time of flight for the projectile
     * @param angle Launch angle in degrees
     * @param initialVelocity Initial velocity (m/s)
     * @param heightDifference Height difference (meters)
     * @return Time of flight in seconds
     */
    public static double calculateTimeOfFlight(double angle, double initialVelocity, double heightDifference) {
        double angleRad = Math.toRadians(angle);
        double vy0 = initialVelocity * Math.sin(angleRad);

        // Use quadratic formula
        double discriminant = vy0 * vy0 - 2 * GRAVITY * heightDifference;

        if (discriminant < 0) {
            return -1; // No real solution
        }

        return (vy0 + Math.sqrt(discriminant)) / GRAVITY;
    }

    /**
     * Calculates the maximum range for given initial velocity
     * @param initialVelocity Initial velocity (m/s)
     * @return Maximum possible range in meters
     */
    public static double calculateMaxRange(double initialVelocity) {
        return (initialVelocity * initialVelocity) / GRAVITY;
    }

    /**
     * Calculates the range for a given angle and velocity
     * @param angle Launch angle in degrees
     * @param initialVelocity Initial velocity (m/s)
     * @param heightDifference Height difference (meters)
     * @return Range in meters
     */
    public static double calculateRange(double angle, double initialVelocity, double heightDifference) {
        double angleRad = Math.toRadians(angle);
        double vx = initialVelocity * Math.cos(angleRad);
        double timeOfFlight = calculateTimeOfFlight(angle, initialVelocity, heightDifference);
        if (timeOfFlight <= 0) return -1;
        return vx * timeOfFlight;
    }

    /**
     * Validates if the shot is physically possible
     * @param distance Target distance (meters)
     * @param initialVelocity Initial velocity (m/s)
     * @param heightDifference Height difference (meters)
     * @return true if shot is possible, false otherwise
     */
    public static boolean isShotPossible(double distance, double initialVelocity, double heightDifference) {
        double maxRange = calculateMaxRange(initialVelocity);
        // Add some margin for height differences (this is mostly for real world)
        double effectiveMaxRange = maxRange + Math.abs(heightDifference);
        return distance <= effectiveMaxRange;
    }

    /**
     * Calculates ball exit velocity from flywheel parameters
     * @param flywheelRPM Flywheel speed in revolutions per minute
     * @param wheelDiameterMeters Flywheel diameter in meters
     * @param slipFactor Efficiency factor accounting for slip/compression (typically 0.7-0.9)
     * @return Estimated ball exit velocity in m/s
     */

    // Courtesy of Claude Sonnet 4 (these next few methods)
    public static double calculateExitVelocity(double flywheelRPM, double wheelDiameterMeters, double slipFactor) {
        // Step 1: Convert RPM to angular velocity (radians per second)
        double omega = (2 * Math.PI * flywheelRPM) / 60.0;

        // Step 2: Calculate linear velocity at wheel rim
        double radius = wheelDiameterMeters / 2.0;
        double rimVelocity = omega * radius;

        // Step 3: Apply slip factor to get realistic ball velocity
        return rimVelocity * slipFactor;
    }

    /**
     * Overloaded method with default slip factor of 0.8 (80% efficiency)
     * @param flywheelRPM Flywheel speed in RPM
     * @param wheelDiameterMeters Flywheel diameter in meters
     * @return Estimated ball exit velocity in m/s
     */
    public static double calculateExitVelocity(double flywheelRPM, double wheelDiameterMeters) {
        return calculateExitVelocity(flywheelRPM, wheelDiameterMeters, 0.8);
    }

    /**
     * Complete shooting solution: from flywheel RPM to launch angle
     * @param flywheelRPM Flywheel speed in RPM
     * @param wheelDiameterMeters Flywheel diameter in meters
     * @param slipFactor Slip factor (0.7-0.9)
     * @param distanceToTarget Horizontal distance to target in meters
     * @param heightDifference Height difference to target in meters
     * @return Launch angle in degrees, or -1 if impossible
     */
    public static double calculateAngleFromRPM(double flywheelRPM, double wheelDiameterMeters,
                                               double slipFactor, double distanceToTarget,
                                               double heightDifference) {
        double exitVelocity = calculateExitVelocity(flywheelRPM, wheelDiameterMeters, slipFactor);
        return calculateLaunchAngle(distanceToTarget, exitVelocity, heightDifference);
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    /**
     * Example usage and testing method
     */
    // Courtesy of Claude Sonnet 4
    public static void main(String[] args) {
        System.out.println("=== Shooter Math Calculator ===");

        // Example 1: Direct velocity calculation
        double distance = 5.0; // meters
        double velocity = 10.0; // m/s
        double heightDiff = 0.5; // target is 0.5m higher

        System.out.println("\n--- Example 1: Direct Velocity ---");
        System.out.printf("Distance to target: %.1f m%n", distance);
        System.out.printf("Initial velocity: %.1f m/s%n", velocity);
        System.out.printf("Height difference: %.1f m%n", heightDiff);

        if (isShotPossible(distance, velocity, heightDiff)) {
            double angle = calculateLaunchAngle(distance, velocity, heightDiff);
            if (angle > 0) {
                System.out.printf("Optimal launch angle: %.2f degrees%n", angle);
                System.out.printf("Time of flight: %.2f seconds%n",
                        calculateTimeOfFlight(angle, velocity, heightDiff));
            } else {
                System.out.println("No valid angle found!");
            }
        } else {
            System.out.println("Shot not possible - increase velocity or decrease distance");
        }

        // Example 2: Flywheel-based calculation
        System.out.println("\n--- Example 2: Flywheel-based ---");
        double flywheelRPM = 3000; // RPM
        double wheelDiameter = 0.1; // 10cm diameter wheel
        double slipFactor = 0.8; // 80% efficiency

        System.out.printf("Flywheel RPM: %.0f%n", flywheelRPM);
        System.out.printf("Wheel diameter: %.1f cm%n", wheelDiameter * 100);
        System.out.printf("Slip factor: %.1f%%%n", slipFactor * 100);

        double exitVel = calculateExitVelocity(flywheelRPM, wheelDiameter, slipFactor);
        System.out.printf("Calculated exit velocity: %.2f m/s%n", exitVel);

        double angleFromRPM = calculateAngleFromRPM(flywheelRPM, wheelDiameter, slipFactor, distance, heightDiff);
        if (angleFromRPM > 0) {
            System.out.printf("Required hood angle: %.2f degrees%n", angleFromRPM);
        } else {
            System.out.println("Shot not possible with current flywheel settings");
        }

        System.out.printf("Maximum possible range: %.2f m%n", calculateMaxRange(exitVel));
    }
}