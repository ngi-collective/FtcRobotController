/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.components;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class FlyWheelShooter {
    private final double INTAKE_SPEED = 600;
    private final double SHOOTING_INTAKE_SPEED = 700;   //rpm speed
    private final double IDLE_RPM = 1000;
    private final double MAX_RPM = 2250;
    private final double MIN_RPM = 1700;

    //Positions of Servos (when opening & closing)
    private final double SERVO1_OPEN_POS = .80;
    private final double SERVO1_CLOSE_POS = .65;
    private final double SERVO2_OPEN_POS = .70;
    private final double SERVO2_CLOSE_POS = .55;
    private double MAX_EXPECTED_DISTANCE_IN = 119;
    private double MIN_EXPECTED_DISTANCE_IN = 42;

// Parts Defined
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo shootingServo1;
    private Servo shootingServo2;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private WebcamName webcam;


    boolean isShooting = false;
    int speed = 0;
    public FlyWheelShooter(HardwareMap hardwareMap) {
        shootingServo1 = hardwareMap.get(Servo.class,"Shooting Servo 1");
        shootingServo2 = hardwareMap.get(Servo.class,"Shooting Servo 2");
        flywheel = hardwareMap.get(DcMotorEx.class,"Flywheel");
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        shootingServo1.setDirection(Servo.Direction.REVERSE);
        shootingServo2.setDirection(Servo.Direction.FORWARD);
        initAprilTag();

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel Velocity",flywheel.getVelocity());
        telemetry.addData("Servo 1 Position", shootingServo1.getPosition());
        telemetry.addData("Servo 2 Position", shootingServo2.getPosition());
        telemetryAprilTag(telemetry);
        telemetry.update();

    }

    //Resets Flywheel & Intake to Lower Speed
    public void startIdle() {
        flywheel.setVelocity(IDLE_RPM);
        intake.setVelocity(0);
    }
// Starts Intake, not to be confused with setIntakeSpeed()
    public void startIntake() {
        setIntakeSpeed(INTAKE_SPEED);
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    //rpm of outtake
    public void setOuttakeSpeedRaw(double speed) {
        flywheel.setVelocity(speed);
    }

    public void setOuttakeSpeedPid(double speed) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(180, 0, 0, 10.4);
        flywheel.setPIDFCoefficients(DcMotor.RunMode. RUN_USING_ENCODER, pidfCoefficients);
        flywheel.setVelocity(speed);

    }

    //changes Intake speed (not startIntake() )
    public void setIntakeSpeed(double speed) {
        intake.setVelocity(speed);
    }

    public void shoot(LinearOpMode opmode) {
        if (isShooting) {
            return;
        }
        isShooting = true;

        // Detect distance
        double distance = getDetectedRange();
        double normalized_distance = distance - MIN_EXPECTED_DISTANCE_IN;
        double range = MAX_EXPECTED_DISTANCE_IN - MIN_EXPECTED_DISTANCE_IN;
        double pct = normalized_distance / range;
        double rpmRange = MAX_RPM - MIN_RPM;
        double rpmAdd = rpmRange * pct;
        double desiredRpm = MIN_RPM + rpmAdd;

        // Spin up motor
        setOuttakeSpeedPid(desiredRpm);


        // WAIT FOR RPM TO HIT DESIRED RPM
       /* int flywheelTimer = 0;
        while ((flywheel.getVelocity() <= desiredRpm) && (flywheelTimer <= 200)){
            flywheelTimer = flywheelTimer + 1;
            sleep(10);
            updateTelemetry(opmode.telemetry);
            if (!opmode.opModeIsActive()){
                return;
            }
        }
        openServos();
        setIntakeSpeed(SHOOTING_INTAKE_SPEED);
*/
        sleep(1500);
        setIntakeSpeed(SHOOTING_INTAKE_SPEED);
        sleep(500);
        openServos();
        updateTelemetry(opmode.telemetry);

    }

    //uses apriltag to get distance
    private double getDetectedRange() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20 || detection.id == 24) {
                return detection.ftcPose.range;
            }
        }
        return 0;
    }

    public void openServos() {
        shootingServo1.setPosition(SERVO1_OPEN_POS);
        shootingServo2.setPosition(SERVO2_OPEN_POS);
    }

    public void closeServos() {
        shootingServo1.setPosition(SERVO1_CLOSE_POS);
        shootingServo2.setPosition(SERVO2_CLOSE_POS);
    }

    public void finishShooting() {
        isShooting = false;
        startIdle();
        closeServos();
        // turn off Intake
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(webcam);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
