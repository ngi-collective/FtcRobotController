package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Simple AprilTag shooter function - add to your existing OpMode class
 *
 * USAGE: Just call shootAtAprilTag(targetId) from your opmode!
 */
public class aprilTagAutoAim{
    // Constants - ADJUST THESE FOR YOUR ROBOT
    CameraName Camera = hardwareMap.get(CameraName.class,"Camera");
    private static final double RPM_TOLERANCE = 5.0;
    // Create the AprilTag processor and assign it to a variable.
    public static AprilTagProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    static ElapsedTime timer = new ElapsedTime();
    public static AprilTagDetection tagDetection;
    VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(Camera,myAprilTagProcessor);

    /**
     * Main function - Call this from your OpMode!
     * Assumes your OpMode has: flywheelMotor, stagingServo, aprilTag defined
     *
     * @param targetTagId Which AprilTag ID to shoot at
     * @param Camera
     * @param flywheel
     * @return true if successful, false if tag not found
     */
    public static boolean shootAtAprilTag(int targetTagId, WebcamName Camera, DcMotorEx flywheel) {
        // Find the AprilTag
        AprilTagDetection tag = findTag(targetTagId, 5.0);
        if (tag == null) {
            opMode.telemetry.addLine("Tag not found!");
            opMode.telemetry.update();
            return false;
        }

        // Get distance and calculate RPM
        double range = tag.ftcPose.range;

        opMode.telemetry.addData("Tag ID", tag.id);
        opMode.telemetry.addData("Range", "%.1f in", range);
        double targetRPM = 0;
        opMode.telemetry.addData("Target RPM", "%.0f", targetRPM);
        opMode.telemetry.update();

        // Spin up flywheel
        targetRPM = setFlywheelRPM(flywheel, Camera);
        // Set speed and wait for it.
        waitForSpeed(targetRPM, flywheel);

        // Shoot!
        opMode.telemetry.addLine("FIRING!");
        opMode.telemetry.update();

        opMode.telemetry.addLine("Shot complete!");
        opMode.telemetry.update();

        return true;
    }

    private static AprilTagDetection findTag(int targetId, double timeout) {
        while (timer.seconds() < timeout) {
            ArrayList<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == 20) {
                    return detection;
                }
                if (detection.id == 24) {
                    return detection;
                }
            }

        }

        return null;
    }

    private static double setFlywheelRPM(DcMotorEx flywheel, CameraName Camera) {
// Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        double targetRPM = tagDetection.ftcPose.range;
        flywheel.setVelocity(tagDetection.ftcPose.range);
        return targetRPM;
    }

    private static void waitForSpeed(double targetRPM, DcMotorEx flywheel) {
        while (flywheel.getVelocity() != targetRPM) {

            if (Math.abs(targetRPM - flywheel.getVelocity()) < RPM_TOLERANCE) {
                return;
            }

            opMode.telemetry.addData("Current RPM", "%.0f", flywheel.getVelocity());
            opMode.telemetry.addData("Target RPM", "%.0f", targetRPM);
            opMode.telemetry.update();
        }
    }
}

/* ========== PASTE THIS INTO YOUR EXISTING OPMODE ========== */

/*
// STEP 1: Make sure your OpMode has these hardware variables declared:
public DcMotorEx flywheelMotor;
public Servo stagingServo;
public AprilTagProcessor aprilTag;

// STEP 2: Initialize them in runOpMode():
flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

stagingServo = hardwareMap.get(Servo.class, "staging_servo");

aprilTag = new AprilTagProcessor.Builder().build();
visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTag)
    .build();

// STEP 3: Call the function whenever you want to shoot:
AprilTagShooter.shootAtAprilTag(this, 1);  // Shoot at AprilTag ID 1

// That's it!
*/