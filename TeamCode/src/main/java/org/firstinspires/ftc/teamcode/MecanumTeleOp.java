package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This OpMode provides a basic mecanum drive control for a robot.
 * It assumes four DC motors are configured as "frontLeftMotor", "backLeftMotor",
 * "frontRightMotor", and "backRightMotor" in the Control Hub's hardware configuration.
 *
 * Control Scheme:
 * - Left Stick Y: Forward/Backward movement
 * - Left Stick X: Strafe Left/Right movement
 * - Right Stick X: Rotate Left/Right movement
 */
@TeleOp(name = "MecanumDrive") // This name will appear on the Driver Hub
public class MecanumTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {
        // Display status on Driver Hub telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. The strings here must match the names assigned
        // during the robot configuration on the Control Hub.
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");

        // Set motor directions. This is crucial for mecanum drive.
        // Directions depend on how motors are physically mounted and which way wheels spin.
        // Adjust these based on testing to ensure desired movement.
        // A common configuration for mecanum:
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Often reversed for proper strafing
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Often reversed for proper strafing
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY on Driver Hub)
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        // Loop continuously until the OpMode is stopped (driver presses STOP)
        while (opModeIsActive()) {
            // Read joystick inputs from gamepad1.
            // Y-axis is typically reversed on gamepads, so multiply by -1 for intuitive control.
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe Left/Right (1.1 factor for imperfect strafing compensation [8])
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate the denominator for motor power normalization.
            // This ensures that no motor power exceeds 1.0 (or goes below -1.0) while preserving
            // the proportional relationship between the motor powers.
            // Math.max ensures the denominator is at least 1 to avoid division by zero
            // and to prevent scaling up powers if all inputs are small. [8]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // Apply mecanum drive equations to calculate individual motor powers. [8]
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Set the calculated powers to the motors.
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Optional: Send motor powers to Driver Hub telemetry for debugging
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}