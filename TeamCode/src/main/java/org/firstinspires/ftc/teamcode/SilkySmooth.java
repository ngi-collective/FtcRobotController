package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.FlyWheelShooter;

import java.util.Arrays;

@TeleOp(name = "Silky Smooth")
public class SilkySmooth extends LinearOpMode {
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

        // One side of the robot needs to be reversed for mecanum wheels to work correctly
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        int targetid;
        //IuseArchbtw
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set zero power behavior - BRAKE provides much smoother, more precise stops
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            // Negating Y because Up on the stick is naturally -1.0
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;

            // --- SLOW MODE LOGIC ---
            // If Left Trigger is held down, cut movement speed by half
            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.1) {
                speedMultiplier = 0.2;
            }

            // Variables kept for your telemetry requirements
            double forwardPower = gamepad1.left_stick_y;
            double backwardPower = gamepad1.left_stick_y;

            // --- Mecanum Drive Kinematics ---
            // Denominator ensures motor powers stay within the -1.0 to 1.0 range
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);

            // Applying the speedMultiplier only to the drive motors
            double frontLeftPower  = ((drive + strafe + twist) / denominator) * speedMultiplier;
            double backLeftPower   = ((drive - strafe + twist) / denominator) * speedMultiplier;
            double frontRightPower = ((drive - strafe - twist) / denominator) * speedMultiplier;
            double backRightPower  = ((drive + strafe - twist) / denominator) * speedMultiplier;

            targetid = 20;
            //Intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,P = 0.8, D = 0, I = 0, F = 0.8);
            shooter.getShootSpeed();
            if (gamepad1.rightBumperWasPressed()) {

                shooter.shoot(this);
            }
            if (gamepad1.rightBumperWasReleased()) {
                shooter.finishShooting();
            }

            // --- Set Motor Powers ---
            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

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

//            if (gamepad1.aWasPressed()) {
//                intake.setVelocity(500);
//            }
//            if (gamepad1.bWasPressed()){
//                intake.setVelocity(-500);
//            }
//
//            if (gamepad1.aWasReleased()){
//                intake.setVelocity(0);
//            }
//            if (gamepad1.bWasReleased()){
//                intake.setVelocity(0);
//            }

            // Intake Power logic - Remains full speed
            if (gamepad1.a) {
                intake.setVelocity(500);
            } else if (gamepad1.b) {
                intake.setVelocity(-500);
            } else {
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
            telemetry.addData("Slow Mode", gamepad1.left_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("FR Encoders", FR.getCurrentPosition());
            telemetry.addData("BL Encoders", BL.getCurrentPosition());
            telemetry.addData("BR Encoders", BR.getCurrentPosition());
            telemetry.addData("FL Encoders", FL.getCurrentPosition());
            telemetry.addData("Forward Stick", "%.2f", forwardPower);
            telemetry.addData("Backward Stick", "%.2f", backwardPower);
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