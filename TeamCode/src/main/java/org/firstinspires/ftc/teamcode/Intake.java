package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Intake {
    public static DcMotor Intake; // Front Left
    public static Telemetry telemetry;

    Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        org.firstinspires.ftc.teamcode.Intake.telemetry = telemetry;

        // Initialize motors from hardware map
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void start() {
        Intake.setPower(100);
        telemetry.addData("Starting intake", "%.2f", Intake.getPower());
        telemetry.update();
    }
    public static void stop() {
        Intake.setPower(0);
        telemetry.addData("Stopping intake", "%.2f", Intake.getPower());
        telemetry.update();
    }
}
