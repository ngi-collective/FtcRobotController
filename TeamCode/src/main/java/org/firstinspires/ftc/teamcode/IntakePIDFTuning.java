package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class IntakePIDFTuning extends OpMode {
    public DcMotorEx intake;
    public double velocity = 700;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    public void init(){
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 8, 9, 5);
        intake.setPIDFCoefficients(DcMotor.RunMode. RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine( "Init complete");
    }
    public void loop(){
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes [stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F+= stepSizes [stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes [stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes [stepIndex];
        }
        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients (P, 0,  0, F);
        intake.setPIDFCoefficients (DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        // set velocity
        intake.setVelocity (velocity);
        double error = velocity - intake.getVelocity();
        telemetry.addData( "Target Velocity", velocity);
        telemetry.addData("Current Velocity", "%.2f", intake.getVelocity());
        telemetry.addData( "Error",  "%.2f", error);
        telemetry.addLine( ". -----------------------------------------------");
        telemetry.addData( "Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData( "Tuning F",  "%.4f (D-Pad L/R)", F);
        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes [stepIndex]);
    }
}