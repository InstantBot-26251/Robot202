package org.firstinspires.ftc.teamcode.util.testing.miscallenous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

@Autonomous(name = "Velocity Calibration", group = "calibration")
public class ModifierCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);

        telemetry.addLine("Press START to calibrate motor velocities");
        telemetry.addLine("Robot will run motors at full power for 3 seconds");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addLine("Calibrating...");
            telemetry.update();

            // Run calibration
            chassis.calibrateVelocityModifiers(3000);

            // Get and display results
            double[] modifiers = chassis.getModifiers();

            telemetry.addLine("Calibration Complete!");
            telemetry.addLine();
            telemetry.addLine("Copy these values to your Chassis class:");
            telemetry.addData("modifierFL", "%.11f", modifiers[0]);
            telemetry.addData("modifierFR", "%.11f", modifiers[1]);
            telemetry.addData("modifierBL", "%.11f", modifiers[2]);
            telemetry.addData("modifierBR", "%.11f", modifiers[3]);
            telemetry.addLine();
            telemetry.addLine("Or set them as default values:");
            telemetry.addLine(String.format("public static double modifierFL = %.11f;", modifiers[0]));
            telemetry.addLine(String.format("public static double modifierFR = %.11f;", modifiers[1]));
            telemetry.addLine(String.format("public static double modifierBL = %.11f;", modifiers[2]));
            telemetry.addLine(String.format("public static double modifierBR = %.11f;", modifiers[3]));
            telemetry.update();

            // Keep displaying results
            while (opModeIsActive()) {
                sleep(50);
            }
        }
    }
}