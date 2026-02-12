package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.FlyWheelShooter;

import java.util.Arrays;

@TeleOp(name = "Mecanum IMU Control YX")
public class MecanumIMUControlYX extends LinearOpMode {

    // Diagnostic Timers
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime methodTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"Intake");
        boolean IntakeOn = false;
        final int INTAKESPD = 500;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        FlyWheelShooter shooter = new FlyWheelShooter(hardwareMap);

        // Monitor for brownouts
        VoltageSensor batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // --- SHOOTING SERVOS ---
        Servo ShootingServo1 = hardwareMap.get(Servo.class, "Shooting Servo 1");
        Servo ShootingServo2 = hardwareMap.get(Servo.class, "Shooting Servo 2");

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

        // --- IMU INITIALIZATION ---
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Vertical mount: Logo facing BACKWARD, USB ports facing UP
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // One side of the robot needs to be reversed for mecanum wheels to work correctly
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        int targetid;
        //IuseArchbtw
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            // Using RUN_WITHOUT_ENCODER to rule out PID jitter
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set zero power behavior - BRAKE provides much smoother, more precise stops
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to begin");
        telemetry.addData(">", "Movement: Triggers (Fwd/Rev), Left Stick X (Strafe), Right Stick X (Rotate)");
        telemetry.addData(">", "Reset Heading: PRESS TOUCHPAD");
        telemetry.update();

        waitForStart();
        shooter.startIdle();

        while (opModeIsActive()) {
            // Start of loop diagnostic
            loopTimer.reset();

            // --- Read Gamepad Inputs ---
            double yInput = -gamepad1.left_stick_y;
            double xInput = gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;

            // --- FIELD CENTRIC RESET (TOUCHPAD) ---
            if (gamepad1.touchpad) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // --- FIXED FIELD CENTRIC MATH ---
            double drive = yInput * Math.cos(-botHeading) - xInput * Math.sin(-botHeading);
            double strafe = -(yInput * Math.sin(-botHeading) + xInput * Math.cos(-botHeading));

            // --- SLOW MODE LOGIC ---
            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.1) {
                speedMultiplier = 0.2;
            }

            double forwardPower = gamepad1.left_stick_y;
            double backwardPower = gamepad1.left_stick_y;

            // --- Mecanum Drive Kinematics ---
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);

            double frontLeftPower  = ((drive + strafe + twist) / denominator) * speedMultiplier;
            double backLeftPower   = ((drive - strafe + twist) / denominator) * speedMultiplier;
            double frontRightPower = ((drive - strafe - twist) / denominator) * speedMultiplier;
            double backRightPower  = ((drive + strafe - twist) / denominator) * speedMultiplier;

            targetid = 20;

            // --- SHOOTER COMPONENT TIMING ---
            methodTimer.reset();
            shooter.getShootSpeed();
            double shooterTime = methodTimer.milliseconds();

            // --- SERVO FEEDER LOGIC ---
            if (gamepad1.right_bumper) {
                shooter.shoot(this);
                ShootingServo1.setPosition(1.0);
                ShootingServo2.setPosition(0.0);
            } else {
                shooter.finishShooting();
                ShootingServo1.setPosition(0.5);
                ShootingServo2.setPosition(0.5);
            }

            // --- Set Motor Powers ---
            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            // -- ROBOT DIGESTIVE SYSTEM CONTROLS (Kept exactly as requested) --
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

            // Intake Power logic
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

            // --- DIAGNOSTICS & TELEMETRY ---
            double volt = batteryVoltage.getVoltage();
            double ms = loopTimer.milliseconds();

            telemetry.addData("Loop Time", "%.1f ms", ms);
            telemetry.addData("Shooter Method Time", "%.1f ms", shooterTime);
            telemetry.addData("Voltage", "%.2f V", volt);
            telemetry.addData("Heading (Deg)", Math.toDegrees(botHeading));
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotation", "%.2f", twist);
            shooter.updateTelemetry(telemetry);
            telemetry.update();
        }

        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        ShootingServo1.setPosition(0.5);
        ShootingServo2.setPosition(0.5);
    }
}