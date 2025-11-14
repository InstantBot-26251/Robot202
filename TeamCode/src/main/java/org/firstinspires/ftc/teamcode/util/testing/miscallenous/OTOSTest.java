package org.firstinspires.ftc.teamcode.util.testing.miscallenous;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="OTOS Test")
public class OTOSTest extends OpMode {
    private SparkFunOTOS otos;

    @Override
    public void init() {
        try {
            otos = hardwareMap.get(SparkFunOTOS.class, "otos");
            telemetry.addLine("OTOS Found!");
        } catch (Exception e) {
            telemetry.addLine("ERROR: " + e.getMessage());
        }
    }

    @Override
    public void loop() {
        if (otos != null) {
            telemetry.addData("Heading", otos.getPosition().h);
        }
    }
}
