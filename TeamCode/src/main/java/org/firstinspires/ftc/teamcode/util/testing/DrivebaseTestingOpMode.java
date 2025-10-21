package org.firstinspires.ftc.teamcode.util.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Drive Testing")
public class DrivebaseTestingOpMode extends OpMode {
    DcMotor fl, fr, bl, br;
    ChassisSimple Chassis;
    double x;
    double y;
    double rx;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "lf");
        fr = hardwareMap.get(DcMotor.class, "rf");
        bl = hardwareMap.get(DcMotor.class, "lr");
        br = hardwareMap.get(DcMotor.class, "rr");
        Chassis = new ChassisSimple(hardwareMap);
    }

    @Override
    public void loop() {

        y = -applyResponseCurve(gamepad1.left_stick_y);
        x = applyResponseCurve(gamepad1.left_stick_x);
        rx = -applyResponseCurve(gamepad1.right_stick_x);

        if (gamepad1.a) {
            fl.setPower(1);
            fr.setPower(1);
            bl.setPower(1);
            br.setPower(1);
        }

        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Adjusted x", x);
        telemetry.addData("Adjusted y", y);
        telemetry.addData("Adjusted rx", rx);

        if(gamepad1.y){
             Chassis.resetYaw();
        }

        Chassis.drive(x, y, rx);
        telemetry.update();
    }

    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

}
