package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.FlyWheelShooter;

import java.util.Arrays;

@TeleOp(name = "Mecanum Proportional Control")
public class MecanumProportionalControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"Intake");
        boolean IntakeOn = false;
        final int INTAKESPD = 500;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);



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

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to begin");
        telemetry.addData(">", "Movement: Triggers (Fwd/Rev), Left Stick X (Strafe), Right Stick X (Rotate)");
        telemetry.addData(">", "Logitech F310: Ensure LED is ON for X-Input mode.");
        telemetry.update();

        waitForStart();
        shooter.startIdle();

        while (opModeIsActive()) {

            // --- Read Gamepad Inputs ---
            // Forward/Backward from triggers
            double forwardPower = gamepad1.left_trigger; // 0.0 to 1.0
            double backwardPower = gamepad1.right_trigger; // 0.0 to 1.0
            // Net forward/backward. Positive is forward, negative is backward.
            double drive = (forwardPower - backwardPower);

            // Strafing from Left Joystick X-axis
            // Left stick X is -1.0 (left) to 1.0 (right).
            // Set to negative because left and right was swapped.
            double strafe = -(gamepad1.left_stick_x);

            // Rotation from Right Joystick X-axis
            // Right stick X is -1.0 (left/counter-clockwise) to 1.0 (right/clockwise).
            // Set to negative because left and right was swapped.
            double twist = (gamepad1.right_stick_x);
            twist *= 1;

            // --- Mecanum Drive Kinematics ---
            // These formulas allow for combining ySpeed, xSpeed, and rotationSpeed.

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1].
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);
            denominator *= 1;
//            double denominator = 1;



            double frontLeftPower = (drive + strafe + twist) / denominator;
            double backLeftPower = (drive - strafe + twist) / denominator;
            double frontRightPower = (drive - strafe - twist) / denominator;
            double backRightPower = (drive + strafe - twist) / (denominator);
            targetid = 20;
            //Intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,P = 0.8, D = 0, I = 0, F = 0.8);
            if (gamepad2.rightBumperWasPressed()) {

                shooter.shoot(this);
            }
            if (gamepad2.rightBumperWasReleased()) {
                shooter.finishShooting();
            }
            // --- Set Motor Powers ---
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
            BL.setPower(backLeftPower);
            FL.setPower(frontLeftPower);

            // -- ROBOT DIGESTIVE SYSTEM CONTROLS --
            /*if (gamepad2.dpad_up) {
                Intake.setVelocity(500);
                telemetry.update();
            }
            if (gamepad2.dpad_down){
                Intake.setVelocity(0);
                telemetry.update();
            }
            if (gamepad2.dpad_left){
                Intake.setVelocity(-50);
                telemetry.update();
            }*/

            //toggle intake
//            if (gamepad2.aWasReleased()){
//                if (IntakeOn == true){
//                    IntakeOn = false;
//                }
//                if (IntakeOn == false){
//                    IntakeOn = true;
//                }
//            }
//            int IntakeCurVel = 0;
//            if (gamepad2.aWasPressed()){
//                IntakeCurVel = INTAKESPD;
//            } else if (gamepad2.bWasPressed()){
//                IntakeCurVel = -(INTAKESPD/2);
//            if (gamepad2.aWasReleased()){
//                IntakeCurVel = 0;
//            }
            if (gamepad2.aWasPressed()) {
                intake.setVelocity(500);
            }
            if (gamepad2.bWasPressed()){
                intake.setVelocity(-500);
            }

            if (gamepad2.aWasReleased()){
                intake.setVelocity(0);
            }
            if (gamepad2.bWasReleased()){
                intake.setVelocity(0);
            }





//          flywheel.setVelocity(OuttakeSpeed);
                //if (gamepad2.left_bumper!=0) {
                //  flywheel.setVelocity(-50);
                //}

                // --          Flywheel Speed AprilTag           --
                //if (gamepad1.right_bumper) {
                //    aprilTagAutoAim.shootAtAprilTag(targetid,Camera,flywheel);
                //}
                //while (gamepad1.right_bumper){
                //  flywheel.setVelocity(gamepad1.right_trigger*5800);
                // while (gamepad1.left_bumper){
                //    Intake.setVelocity(gamepad1.left_trigger*2900);
                //    }
                //}
                //while (gamepad1.left_bumper){
                //   Intake.setVelocity(gamepad1.left_trigger*2900);
                //    while (gamepad1.right_bumper){
                //       flywheel.setVelocity(gamepad1.right_trigger*5800);
                //}}
                //flywheel.setVelocity(0);
                // --- Telemetry ---
                //telemetry.addData("Intake Power", Intake.getPower());
                telemetry.addData("FR Encoders", FR.getCurrentPosition());
                telemetry.addData("BL Encoders", BL.getCurrentPosition());
                telemetry.addData("BR Encoders", BR.getCurrentPosition());
                telemetry.addData("FL Encoders", FL.getCurrentPosition());
                telemetry.addData("Forward Trigger", "%.2f", forwardPower);
                telemetry.addData("Backward Trigger", "%.2f", backwardPower);
                telemetry.addData("YSpeed (Fwd/Rev)", "%.2f", drive);
                telemetry.addData("XSpeed (Strafe LX)", "%.2f", strafe);
                telemetry.addData("Rotation (Rotate RX)", "%.2f", twist);
                telemetry.addData("Denominator", "%.2f", denominator);
                telemetry.addData("FR Velocity", "%.2f", FR.getVelocity());
                telemetry.addData("BL Velocity", "%.2f", BL.getVelocity());
                telemetry.addData("BR Velocity", "%.2f", BR.getVelocity());
                telemetry.addData("FL Velocity", "%.2f", FL.getVelocity());
                telemetry.addData("Intake Power", intake.getPower());
                shooter.updateTelemetry(telemetry);
                telemetry.update();
            }
            // Stop all motors once the OpMode is no longer active
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
        }
    }