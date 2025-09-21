package org.firstinspires.ftc.teamcode.util;

public class MathPM {
    // Initially made by Claude Sonnet 4, changed and iterated on by Lakshya Khandelwal

    // Gravity acceleration speed (m/s2)
    private static final double GRAVITY = 9.81;

    /**
     * Calculates the optimal launch angle for projectile motion
     * @param distance Horizontal distance to target (meters)
     * @param initialVelocity Initial velocity of projectile (m/s)
     * @param heightDifference Height difference between shooter and target (meters, positive if target is higher)
     * @return Launch angle in degrees, or -1 if no solution exists
     */
    public static double calculateLaunchAngle(double distance, double initialVelocity, double heightDifference) {
        // Using user's formula:

        double v2 = initialVelocity * initialVelocity;
        double discriminant = v2 * v2 - GRAVITY * (GRAVITY * distance * distance + 2 * heightDifference * v2);

        // Check if solution exists
        if (discriminant < 0) {
            return -1; // No solution - target is too far or velocity too low
        }

        // Two possible angles - we typically want the lower one for efficiency
        double angle1 = Math.atan((v2 - Math.sqrt(discriminant)) / (GRAVITY * distance));
        double angle2 = Math.atan((v2 + Math.sqrt(discriminant)) / (GRAVITY * distance));

        // Convert to degrees and return the lower angle
        double angle1Deg = Math.toRadians(angle1);
        double angle2Deg = Math.toRadians(angle2);

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
        double discriminant = vy0 * vy0 + 2 * GRAVITY * heightDifference;
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
        // Add some margin for height differences
        double effectiveMaxRange = maxRange + Math.abs(heightDifference);
        return distance <= effectiveMaxRange;
    }

    /**
     * Example usage and testing method (Courtesy of Claude Sonnet 4)
     */

    public static void main(String[] args) {
        // Example: Robot 5 meters from goal, shooting at 10 m/s
        double distance = 5.0; // meters
        double velocity = 10.0; // m/s
        double heightDiff = 0.5; // target is 0.5m higher

        System.out.println("=== Shooter Angle Calculator ===");
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

        System.out.printf("Maximum possible range: %.2f m%n", calculateMaxRange(velocity));
    }
}