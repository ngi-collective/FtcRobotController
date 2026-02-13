package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.FlyWheelShooter;

import java.util.Arrays;

@TeleOp(name = "Mecanum IMU Universal")
public class MecanumIMUControlXY extends LinearOpMode {
    @Override
    public void runOpMode() {
        // --- INTAKE SETUP (REVERTED TO OLD VERSION) ---
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"Intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FlyWheelShooter shooter = new FlyWheelShooter(hardwareMap);

        // --- SHOOTING SERVOS ---
        Servo ShootingServo1 = hardwareMap.get(Servo.class, "Shooting Servo 1");
        Servo ShootingServo2 = hardwareMap.get(Servo.class, "Shooting Servo 2");

        // --- DRIVETRAIN MOTORS ---
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotorEx BL = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx BR = hardwareMap.get(DcMotorEx.class, "BR");
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class, "FL");

        // --- IMU INITIALIZATION ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // Drivetrain Directions
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Keep FLOAT behavior for smoothness
        for (DcMotor dcMotor : Arrays.asList(FR, BL, BR, FL)) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        shooter.startIdle();

        while (opModeIsActive()) {

            // --- Read Gamepad Inputs ---
            double yInput = -gamepad1.left_stick_y;
            double xInput = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            if (gamepad1.touchpad) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // --- UNIVERSAL MOVEMENT MATH ---
            double drive = yInput * Math.cos(-botHeading) - xInput * Math.sin(-botHeading);
            double strafe = (yInput * Math.sin(-botHeading) + xInput * Math.cos(-botHeading));

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);

            double frontLeftPower = (drive + strafe + twist) / denominator;
            double backLeftPower = (drive - strafe + twist) / denominator;
            double frontRightPower = (drive - strafe - twist) / denominator;
            double backRightPower = (drive + strafe - twist) / denominator;

            // --- SHOOTER & SERVO FEEDER ---
            shooter.getShootSpeed();
            if (gamepad1.rightBumperWasPressed()) {
                shooter.shoot(this);
                ShootingServo1.setPosition(1.0);
                ShootingServo2.setPosition(0.0);
            }
            if (gamepad1.rightBumperWasReleased()) {
                shooter.finishShooting();
                ShootingServo1.setPosition(0.5);
                ShootingServo2.setPosition(0.5);
            }

            // --- MOTOR POWER ASSIGNMENT ---
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
            BL.setPower(backLeftPower);
            FL.setPower(frontLeftPower);

            // --- INTAKE/OUTTAKE CONTROLS (REVERTED) ---
            if (gamepad1.a) {
                intake.setPower(-0.45);
            } else if (gamepad1.b) {
                intake.setPower(0.45);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Heading (Deg)", Math.toDegrees(botHeading));
            shooter.updateTelemetry(telemetry);
            telemetry.update();
        }

        FR.setPower(0); BL.setPower(0); BR.setPower(0); FL.setPower(0);
        intake.setVelocity(0);
    }
}