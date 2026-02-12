package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.FlyWheelShooter;

import java.util.Arrays;


@Autonomous(name = "NothingAuto")
public abstract class ForwardAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        final double power = 0.25;

        //flywheel
        FlyWheelShooter shooter = new FlyWheelShooter(hardwareMap);

        // Front Left
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        // Front Right
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        // Back Left
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        // Back Right
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");

        //Object Camera = hardwareMap.get(CameraName.class, "Camera");
        //DcMotor flywheel = hardwareMap.get(DcMotor.class, "Flywheel");
        // Set motor directions:
        // Adjust these if your robot moves incorrectly.
        // One side of the robot needs to be reversed for mecanum wheels to work correctly
        // when driving forward/backward.
        // Typically, if you have motors on the left and right, one side is reversed.
        // Example: If motors are oriented the same way on both sides.a
        FR.setDirection(DcMotorSimple.Direction.REVERSE); // Or FORWARD, depends on mounting
        BR.setDirection(DcMotorSimple.Direction.REVERSE); // Or FORWARD
        BL.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        FL.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        int targetid;
        //IuseArchbtw
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set zero power behavior - FLOAT helps prevent coasting and makes control more precise
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


        //ActualMoveForward Script

        //straight
        FR.setPower(-power);
        BR.setPower(-power);
        BL.setPower(-power);
        FL.setPower(-power);
        sleep(1000);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        sleep(350);

        //turn (left)
        FR.setPower(-power);
        BR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
        sleep(200);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);

        shooter.getShootSpeed();
        sleep(1000);
        shooter.shoot(this);
    }

}
