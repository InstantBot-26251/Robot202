package org.firstinspires.ftc.teamcode.robot;

import android.webkit.WebMessage;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    public HardwareMap hardwareMap;
    private final List<HardwareDevice> devicesH = new ArrayList<>();
    private final List<WebcamName> devicesC = new ArrayList<>();


    public WebcamName webcam;

    public SparkFunOTOS OTOS;

    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

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

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, "backRight");

        addDevicesH();
        addDevicesC();
    }

    private void addDevicesH() {
        devicesH.add(getInstance().OTOS);
        devicesH.add(getInstance().MOTOR_FL);
        devicesH.add(getInstance().MOTOR_FR);
        devicesH.add(getInstance().MOTOR_BL);
        devicesH.add(getInstance().MOTOR_BR);
    }
    private void addDevicesC() {
       devicesH.add(getInstance().webcam);
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
