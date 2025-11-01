package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands.AUTO_AIM;
import static org.firstinspires.ftc.teamcode.shooter.commands.ShooterCommands.AUTO_AIM_AND_SHOOT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.indexer.Indexer;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.testing.miscallenous.ChassisSimple;
import org.firstinspires.ftc.teamcode.vision.ATVision;


@TeleOp(name = "TeleOp", group = "opmodes")
public class TeleOpMode extends OpMode {
    DcMotor fl, fr, bl, br;
    ChassisSimple Chassis;
    double x, y, rx;

    private static final int DASHBOARD_FPS = 10;

    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;


    private static final int SLOT_0 = 0;
    private static final int SLOT_1 = 680;
    private static final int SLOT_2 = 1360;

    private static final double MANIP_RESPONSE = 1.5;
    private static final double DRIVER_RESPONSE = 1.5;
    private FtcDashboard dashboard;
    private MultipleTelemetry multiTelemetry;

    private ATVision vision;

    private static final int[] SLOT_POSITIONS = {SLOT_0, SLOT_1, SLOT_2};

    private boolean wasManual = false;   // Track manual → auto transition

    @Override
    public void init() {
        indexer = Indexer.getInstance();
        shooter = Shooter.getInstance();
        hood = Hood.getInstance();
        vision = ATVision.getInstance();

        indexer.initHardware();

        indexer.onTeleopInit();
        shooter.onTeleopInit();
        hood.onTeleopInit();
        vision.onTeleopInit();

        fl = hardwareMap.get(DcMotor.class, "lf");
        fr = hardwareMap.get(DcMotor.class, "rf");
        bl = hardwareMap.get(DcMotor.class, "lr");
        br = hardwareMap.get(DcMotor.class, "rr");
        Chassis = new ChassisSimple(hardwareMap);
    }

    @Override
    public void loop() {
        y = -applyResponseCurve(gamepad1.left_stick_y, DRIVER_RESPONSE);
        x = applyResponseCurve(gamepad1.left_stick_x, DRIVER_RESPONSE);
        rx = -applyResponseCurve(gamepad1.right_stick_x, DRIVER_RESPONSE);


        boolean manualHeld = gamepad2.left_bumper;


        if (manualHeld) {
            wasManual = true;

            double power = -applyResponseCurve(gamepad2.left_stick_y * 0.4, MANIP_RESPONSE);
            indexer.setPower(power);

        } else {

            if (wasManual) {
                wasManual = false;
                snapIndexerToNearestSlot();
            }


            if (gamepad2.a) {
                indexer.rotateToNextSlot();
            }

            if (gamepad2.b) {
                indexer.rotateToTransferPosition();
            }

            if (gamepad2.dpad_up) {
                indexer.startHopper();
            }

            if (gamepad2.dpad_down) {
                indexer.stopHopper();
            }
        }


        if (gamepad2.x) {
            shooter.startShooting(1.0);
        }

        if (gamepad2.y) {
            shooter.stopShooting();
        }


        if (gamepad2.dpad_left) {
            AUTO_AIM.get();
        }

        if (gamepad2.dpad_right) {
            hood.setDefaultPosition();
        }

        // DRIVER CONTROLS
        if (gamepad1.y){
            Chassis.resetYaw();
        }

        Chassis.drive(x, y, rx);

        // Always run periodic
        indexer.periodic();
        shooter.periodic();
        hood.periodic();


        // TELEMETRY
        telemetry.addData("Manual?", manualHeld);
        telemetry.addData("Pos", indexer.getRotorPosition());
        telemetry.addData("Slot", indexer.getCurrentSlot());
        telemetry.update();
    }



    private void snapIndexerToNearestSlot() {
        int current = indexer.getRotorPosition();

        int nearestSlot = 0;
        int smallestDiff = Math.abs(current - SLOT_POSITIONS[0]);

        for (int i = 1; i < SLOT_POSITIONS.length; i++) {
            int diff = Math.abs(current - SLOT_POSITIONS[i]);
            if (diff < smallestDiff) {
                smallestDiff = diff;
                nearestSlot = i;
            }
        }

        indexer.rotateToSlot(nearestSlot);
    }

    private double applyResponseCurve(double input, double scale) {
        // Limit Input to 1 (MAX) and -1 (MIN)
        input = Math.max(-1, Math.min(1, input));

        // Apply Response Curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }
}
