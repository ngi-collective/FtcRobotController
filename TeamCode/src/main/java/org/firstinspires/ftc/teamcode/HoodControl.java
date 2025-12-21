package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodControl {
    public Servo hood;
    public void reset(Telemetry telemetry, HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "Hood");
        hood.setPosition(0);
    }

    public static class PassiveFullCourt {
    }
}