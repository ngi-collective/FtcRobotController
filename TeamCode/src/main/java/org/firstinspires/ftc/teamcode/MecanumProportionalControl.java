package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(name = "Mecanum Proportional Control")
public class MecanumProportionalControl extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize motors from hardware map
        //intake = hardwareMap.get(DcMotor.class, "Flywheel");
        //private DcMotor intake;
        // Front Left
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        // Front Right
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        // Back Left
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        // Back Right
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        // Set motor directions:
        // Adjust these if your robot moves incorrectly.
        // One side of the robot needs to be reversed for mecanum wheels to work correctly
        // when driving forward/backward.
        // Typically, if you have motors on the left and right, one side is reversed.
        // Example: If motors are oriented the same way on both sides.a
        FL.setDirection(DcMotorSimple.Direction.REVERSE); // Or FORWARD, depends on mounting
        BL.setDirection(DcMotorSimple.Direction.REVERSE); // Or FORWARD
        FR.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        BR.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // Set zero power behavior - FLOAT helps prevent coasting and makes control more precise
        for (DcMotor dcMotor : Arrays.asList(FL, FR, BL, BR)) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to begin");
        telemetry.addData(">", "Movement: Triggers (Fwd/Rev), Left Stick X (Strafe), Right Stick X (Rotate)");
        telemetry.addData(">", "Logitech F310: Ensure LED is ON for X-Input mode.");
        telemetry.update();

        waitForStart();

            while (opModeIsActive()) {

                // --- Read Gamepad Inputs ---
                // Forward/Backward from triggers
                double backwardPower = gamepad1.left_trigger; // 0.0 to 1.0
                double forwardPower = gamepad1.right_trigger; // 0.0 to 1.0
                // Net forward/backward. Positive is forward, negative is backward.
                double ySpeed = forwardPower - backwardPower;

                // Strafing from Left Joystick X-axis
                // Left stick X is -1.0 (left) to 1.0 (right).
                // Set to negative because left and right was swapped.
                double xSpeed = gamepad1.left_stick_x;

                // Rotation from Right Joystick X-axis
                // Right stick X is -1.0 (left/counter-clockwise) to 1.0 (right/clockwise).
                // Set to negative because left and right was swapped.
                double rotationSpeed = -gamepad1.right_stick_x;

                // --- Mecanum Drive Kinematics ---
                // These formulas allow for combining ySpeed, xSpeed, and rotationSpeed.

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1].
                double denominator = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(rotationSpeed), 1);

                double frontLeftPower = (ySpeed + xSpeed + rotationSpeed) / denominator;
                double backLeftPower = (ySpeed - xSpeed + rotationSpeed) / denominator;
                double frontRightPower = (ySpeed - xSpeed - rotationSpeed) / denominator;
                double backRightPower = (ySpeed + xSpeed - rotationSpeed) / denominator;

                // --- Set Motor Powers ---
                FL.setPower(frontLeftPower);
                BL.setPower(backLeftPower);
                FR.setPower(frontRightPower);
                BR.setPower(backRightPower);
                while (gamepad1.right_bumper){
                    //intake.setPower(gamepad1.right_trigger);
                }
                //intake.setPower(0);


                // --- Telemetry ---
                //telemetry.addData("Intake Power", intake.getPower());
                telemetry.addData("FL Encoders", FL.getCurrentPosition());
                telemetry.addData("FR Encoders", FR.getCurrentPosition());
                telemetry.addData("BL Encoders", BL.getCurrentPosition());
                telemetry.addData("BR Encoders", BR.getCurrentPosition());
                telemetry.addData("Forward Trigger", "%.2f", forwardPower);
                telemetry.addData("Backward Trigger", "%.2f", backwardPower);
                telemetry.addData("YSpeed (Fwd/Rev)", "%.2f", ySpeed);
                telemetry.addData("XSpeed (Strafe LX)", "%.2f", xSpeed);
                telemetry.addData("Rotation (Rotate RX)", "%.2f", rotationSpeed);
                telemetry.addData("Denominator", "%.2f", denominator);
                telemetry.addData("FL Power", "%.2f", FL.getPower());
                telemetry.addData("FR Power", "%.2f", FR.getPower());
                telemetry.addData("BL Power", "%.2f", BL.getPower());
                telemetry.addData("BR Power", "%.2f", BR.getPower());
                telemetry.update();
            }
        // Stop all motors once the OpMode is no longer active
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        }
    }
