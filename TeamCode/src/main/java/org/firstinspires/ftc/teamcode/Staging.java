package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Staging {
    public DcMotor stager;

    Staging(Telemetry telemetry, HardwareMap hardwareMap) {
        stager = hardwareMap.get(DcMotor.class, "Stager");
        stager.setDirection(DcMotorSimple.Direction.FORWARD);
        stager.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        stager.setPower(0.1);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        stager.setPower(0);
    }
}