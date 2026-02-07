package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;


@Autonomous(name = "NothingAuto")
public class ForwardAuto extends OpMode {
    public void init() {
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
        FR.setPower(-1);
        BR.setPower(-1);
        BL.setPower(-1);
        FL.setPower(-1);
        sleep(1500);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);

//        FR.setPower(1);
//        BR.setPower(1);
//        BL.setPower(-1);
//        FL.setPower(-1);
//        sleep(100);
//        shooter.shoot(FlyWheelShooter);
//        sleep(3000);
//        shooter.finishShooting();
//        sleep(500);
//        shooter.shoot(FlyWheelShooter);
//        leep(3000);
//        shooter.finishShooting();
//        sleep(500);
//        FR.setPower(-1);
//        BR.setPower(-1);
//        BL.setPower(1);
//        FL.setPower(1);
//        sleep(100);
//        FR.setPower(1);
//        BR.setPower(1);
//        BL.setPower(1);
//        FL.setPower(1);
//        sleep(1000);
    }

    @Override
    public void loop() {
        return;
    }
}
