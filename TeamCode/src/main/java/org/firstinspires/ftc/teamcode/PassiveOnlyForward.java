package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Passive Only Forward")

public class PassiveOnlyForward extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Front Left
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        // Front Right
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        // Back Left
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        // Back Right
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");

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

        FR.setPower(200);
        BR.setPower(200);
        BL.setPower(200);
        FL.setPower(200);

        sleep(1500);

        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
    }

}
