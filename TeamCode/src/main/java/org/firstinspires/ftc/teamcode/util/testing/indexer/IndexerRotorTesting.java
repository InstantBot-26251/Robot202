package org.firstinspires.ftc.teamcode.util.testing.indexer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Indexer Rotor Testing")
public class IndexerRotorTesting extends OpMode {
    DcMotorEx indexer;


    @Override
    public void init() {
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            indexer.setPower(0.25);
        }
        if (gamepad2.x) {
            indexer.setPower(0.5);
        }
        if (gamepad2.b) {
            indexer.setPower(0.75);
        }
        if (gamepad2.a) {
            indexer.setPower(1);
        }
        if (gamepad2.right_bumper) {
            indexer.setPower(0);
        }
    }
}
