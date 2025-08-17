package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel {
    public static DcMotor flywheel;
    public static void flywheelsetup(){
        flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
    }
}
