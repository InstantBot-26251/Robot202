package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    IMU imu;
//    SparkFunOTOS otos;

//
//    public static double modifierFL = 0.88095238095;
//    public static double modifierFR = 0.89516129032;
//    public static double modifierBR = 0.9652173913;

    public double driveScale = 1;


    public static double modifierFL = 0.9160353435;
    public static double modifierFR = 0.92307692308;
    public static double modifierBL = 1.0;
    public static double modifierBR = 0.96774193548;

    // Store last velocities for telemetry
    private double lastVelocityFL = 0;
    private double lastVelocityFR = 0;
    private double lastVelocityBL = 0;
    private double lastVelocityBR = 0;

    public Chassis (HardwareMap hardwareMap) {

        // Initialize motors
        fl = hardwareMap.get(DcMotorEx.class, "lf");
        fr = hardwareMap.get(DcMotorEx.class, "rf");
        bl = hardwareMap.get(DcMotorEx.class, "lr");
        br = hardwareMap.get(DcMotorEx.class, "rr");

        // Set motor directions and zero power behavior
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize x
        imu = hardwareMap.get(IMU.class, "imu");

//        // Set up IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        // Initialize x
//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.setOffset(new SparkFunOTOS.Pose2D(2.5,3.75, Math.PI));

    }
    public void resetYaw() {
        imu.resetYaw();
    }

    public void drive(double x, double y, double rx) {
////        // Get the robot's current heading
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////        double botHeading = otos.getPosition().h;
//
//        // Adjust the input values for field-centric control
//        double adjustedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double adjustedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        x = x * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Apply velocity modifiers
        fl.setPower((frontLeftPower * modifierFL) * driveScale);
        fr.setPower((frontRightPower * modifierFR) * driveScale);
        bl.setPower((backLeftPower * modifierBL) * driveScale);
        br.setPower((backRightPower * modifierBR) * driveScale);

        // Store velocities for telemetry
        lastVelocityFL = fl.getVelocity();
        lastVelocityFR = fr.getVelocity();
        lastVelocityBL = bl.getVelocity();
        lastVelocityBR = br.getVelocity();
    }

    public void moveBackward() {
        fl.setPower(0.5 * modifierFL);
        fr.setPower(0.5 * modifierFR);
        bl.setPower(0.5 * modifierBL);
        br.setPower(0.5 * modifierBR);
    }

    public void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void strafeRight() {
        fl.setPower(0.5 * modifierFL);
        fr.setPower(-0.5 * modifierFR);
        br.setPower(0.5 * modifierBR);
        bl.setPower(-0.5 * modifierBL);
    }


    public void strafeLeft() {
        fl.setPower(-0.5 * modifierFL);
        fr.setPower(0.5 * modifierFR);
        br.setPower(-0.5 * modifierBR);
        bl.setPower(0.5 * modifierBL);
    }



    public void calibrateVelocityModifiers(long durationMs) {
        // Run all motors at full power
        fl.setPower(1.0);
        fr.setPower(1.0);
        bl.setPower(1.0);
        br.setPower(1.0);

        // Wait for motors to stabilize
        try {
            Thread.sleep(durationMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Get velocities
        double velFL = Math.abs(fl.getVelocity());
        double velFR = Math.abs(fr.getVelocity());
        double velBL = Math.abs(bl.getVelocity());
        double velBR = Math.abs(br.getVelocity());

        // Find minimum velocity
        double minVel = Math.min(Math.min(velFL, velFR), Math.min(velBL, velBR));

        // Calculate modifiers (slowest motor gets 1.0, others get reduced)
        modifierFL = minVel / velFL;
        modifierFR = minVel / velFR;
        modifierBL = minVel / velBL;
        modifierBR = minVel / velBR;

        // Stop motors
        stopMotors();
    }


    // Get current motor velocities for telemetry

    public double[] getCurrentVelocities() {
        return new double[] {
                lastVelocityFL,
                lastVelocityFR,
                lastVelocityBL,
                lastVelocityBR
        };
    }


    // Get current modifiers for telemetry
         public double[] getModifiers() {
        return new double[] {
                modifierFL,
                modifierFR,
                modifierBL,
                modifierBR
        };
    }

    public void enableSlowMode() {
        driveScale = 0.25;
    }

    public void disableSlowMode() {
        driveScale = 1.0;
    }
}