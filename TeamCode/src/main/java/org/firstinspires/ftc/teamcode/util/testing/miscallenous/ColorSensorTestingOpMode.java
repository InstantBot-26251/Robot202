package org.firstinspires.ftc.teamcode.util.testing.miscallenous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Config
@TeleOp(name = "Color Sensor Testing", group = "testing")
public class ColorSensorTestingOpMode extends OpMode {
    RevColorSensorV3 colorSensor;
    public static double redHueT = 30;
    public static double redHueB = 350;
    public static double greenHueT = 61;
    public static double greenHueB = 180;
    public static double purpleHueT = 260;
    public static double purpleHueB = 310;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color1");
    }

    @Override
    public void loop() {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float hue = JavaUtil.colorToHue(colors.toColor());
        float saturation = JavaUtil.colorToSaturation(colors.toColor());
        float value = JavaUtil.colorToValue(colors.toColor());

        if (hue < redHueT || hue > redHueB) {
            telemetry.addData("Detected Color", "Red");
        } else if (greenHueT >= 90 && greenHueB < 150) {
            telemetry.addData("Detected Color", "Green");
        } else if (hue >= 200 && hue < 260) {
            telemetry.addData("Detected Color", "Blue");
        } else if (purpleHueT >= 260 && purpleHueB <= 310 ) {
            telemetry.addData("Detected Color", "Purple");
        } else {
            telemetry.addData("Detected Color", "Unknown");
        }

        telemetry.update();
    }
}
