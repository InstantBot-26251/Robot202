package org.firstinspires.ftc.teamcode.util.testing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Color Sensor Testing", group = "testing")
public class ColorSensorTestingOpMode extends OpMode {
    RevColorSensorV3 colorSensor;


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

        if (hue < 30 || hue > 350) {
            telemetry.addData("Detected Color", "Red");
        } else if (hue >= 90 && hue < 150) {
            telemetry.addData("Detected Color", "Green");
        } else if (hue >= 200 && hue < 260) {
            telemetry.addData("Detected Color", "Blue");
        } else {
            telemetry.addData("Detected Color", "Unknown");
        }

        telemetry.update();
    }
}
