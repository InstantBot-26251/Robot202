package org.firstinspires.ftc.teamcode.util.testing.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter RPM")
public class ShooterRPM extends OpMode {

    private DcMotor flywheelMotor;
    private int lastPosition = 0;
    private double lastTime = 0;
    private double ticksPerSecond = 0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "shooter");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastPosition = flywheelMotor.getCurrentPosition();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        int currentPosition = flywheelMotor.getCurrentPosition();
        double currentTime = getRuntime();
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0) {
            int deltaPosition = currentPosition - lastPosition;
            ticksPerSecond = deltaPosition / deltaTime;
        }

        lastPosition = currentPosition;
        lastTime = currentTime;

        telemetry.addData("Ticks per second", "%.2f", ticksPerSecond);
        telemetry.update();
    }

 
}
