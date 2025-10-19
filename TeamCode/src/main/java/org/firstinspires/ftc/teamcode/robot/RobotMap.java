package org.firstinspires.ftc.teamcode.robot;

import android.webkit.WebMessage;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    public HardwareMap hardwareMap;
    private final List<HardwareDevice> devicesH = new ArrayList<>();
    private final List<WebcamName> devicesC = new ArrayList<>();

    // April Tag Webcam
    public WebcamName WEBCAM;

    // OTOS
    public SparkFunOTOS OTOS;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    // Shooter motor
    public DcMotorEx SHOOTER;

    // Hood servo
    public Servo HOOD;

    // Indexer hardware
    public DcMotorEx INDEXER_ROTOR;
    public Servo INDEXER_GATE;

    // Color Sensors
    public RevColorSensorV3 COLOR1;
    public RevColorSensorV3 COLOR2;
    public RevColorSensorV3 COLOR3;
    public RevColorSensorV3 COLOR4;
    public RevColorSensorV3 COLOR5;
    public RevColorSensorV3 COLOR6;
    public RevColorSensorV3 COLOR7;

    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        devicesH.clear();
        devicesC.clear();

        this.hardwareMap = hardwareMap;

//        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

//        WEBCAM = hardwareMap.get(WebcamName.class, "Webcam 1");

//        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "frontRight");
//        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "backLeft");
//        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "backRight");

        // Shooter
        SHOOTER = hardwareMap.get(DcMotorEx.class, "shooter");
        HOOD = hardwareMap.get(Servo.class, "hood");
//
//        // Indexer
//        INDEXER_ROTOR = hardwareMap.get(DcMotorEx.class, "indexerRotor");
//        INDEXER_GATE = hardwareMap.get(Servo.class, "indexerGate");
//
//        COLOR1 = hardwareMap.get(RevColorSensorV3.class, "color1");
//        COLOR2 = hardwareMap.get(RevColorSensorV3.class, "color2");
//        COLOR3 = hardwareMap.get(RevColorSensorV3.class, "color3");
//        COLOR4 = hardwareMap.get(RevColorSensorV3.class, "color4");
//        COLOR5 = hardwareMap.get(RevColorSensorV3.class, "color5");
//        COLOR6 = hardwareMap.get(RevColorSensorV3.class, "color6");

        addDevicesH();
        addDevicesC();
    }

    private void addDevicesH() {
//        devicesH.add(getInstance().OTOS);
//        devicesH.add(getInstance().MOTOR_FL);
//        devicesH.add(getInstance().MOTOR_FR);
//        devicesH.add(getInstance().MOTOR_BL);
//        devicesH.add(getInstance().MOTOR_BR);
        devicesH.add(getInstance().SHOOTER);
        devicesH.add(getInstance().HOOD);
//        devicesH.add(getInstance().INDEXER_ROTOR);
//        devicesH.add(getInstance().INDEXER_GATE);
//        devicesH.add(getInstance().COLOR1);
//        devicesH.add(getInstance().COLOR2);
//        devicesH.add(getInstance().COLOR3);
//        devicesH.add(getInstance().COLOR4);
//        devicesH.add(getInstance().COLOR5);
//        devicesH.add(getInstance().COLOR6);
    }
    private void addDevicesC() {
//       devicesH.add(getInstance().WEBCAM);
    }

    // Getter Methods
    public List<HardwareDevice> getDevicesH() {
        return devicesH;
    }
    public List<WebcamName> getDevicesC() {
        return devicesC;
    }

    public List<LynxModule> getLynxModules() {
        return hardwareMap.getAll(LynxModule.class);
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }
}
