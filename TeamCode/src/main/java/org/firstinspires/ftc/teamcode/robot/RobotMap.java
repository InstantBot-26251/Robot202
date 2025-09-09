package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    public HardwareMap hardwareMap;
    private final List<HardwareDevice> devicesH = new ArrayList<>();
    private final List<OpenCvWebcam> devicesW = new ArrayList<>();

    // CAMERAS

    // Camera 1
    public OpenCvWebcam WEBCAM1;

    // Camera 1
//    public WebcamName webcam1;

    // Camera 2
//    public OpenCvWebcam webcam2;

    // SENSORS

    // OTOS
    public SparkFunOTOS OTOS;

    // Intake Color Sensor
    public RevColorSensorV3 INTAKE_COLOR_SENSOR;

    // MOTORS

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    // Intake
    public DcMotor INTAKE;

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
        devicesW.clear();

        this.hardwareMap = hardwareMap;

        // Webcam
        WEBCAM1 = hardwareMap.get(OpenCvWebcam.class, "webcam1");

        // OTOS
        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Intake Color Sensor
        INTAKE_COLOR_SENSOR = hardwareMap.get(RevColorSensorV3.class, "intakeColorS");

        // Drive Motors
        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "backRight");

        // Intake
        INTAKE = hardwareMap.get(DcMotor.class, "intake");
        addDevicesH();

    }

    private void addDevicesH() {
        devicesH.add(getInstance().OTOS);
        devicesH.add(getInstance().INTAKE_COLOR_SENSOR);
        devicesH.add(getInstance().MOTOR_FL);
        devicesH.add(getInstance().MOTOR_FR);
        devicesH.add(getInstance().MOTOR_BL);
        devicesH.add(getInstance().MOTOR_BR);
        devicesH.add(getInstance().INTAKE);

    }

    private void addDevicesW() {
        devicesW.add(getInstance().WEBCAM1);
//        devicesW.add(getInstance().webcam2);
    }

    // Getter Methods
    public List<HardwareDevice> getDevicesH() {
        return devicesH;
    }

    public List<OpenCvWebcam> getDevicesW() {
        return devicesW;
    }

    public List<LynxModule> getLynxModules() {
        return hardwareMap.getAll(LynxModule.class);
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }
}
