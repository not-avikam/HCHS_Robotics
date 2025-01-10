/* Copyright (c) 2021 FIRST. All rights reserved.
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


import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.io.File;

@TeleOp(name="award_red_basket-side", group="League Tournament")
public class hypothetical_red_basket extends LinearOpMode {
    private Follower follower;
    private final ElapsedTime runtime = new ElapsedTime();
    private final String soundPath = "/FIRST/blocks/sounds";
    private final File Alert  = new File("/sdcard" + soundPath + "/gold.wav");
    static final boolean FIELD_CENTRIC = false;
    private final Pose startPose = new Pose(9, 105, Math.toRadians(0));
    private final Pose observationZone = new Pose(144, 144);
    private final Pose basket = new Pose(144, 0);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        follower = new Follower(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312)
        );
        MotorEx vSlideLeft = hardwareMap.get(MotorEx.class, "VSL");
        MotorEx vSlideRight = hardwareMap.get(MotorEx.class, "VSR");
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        ServoEx claw = new SimpleServo(hardwareMap, "claw", 0, 180, AngleUnit.RADIANS);
        ServoEx clawAdjust = hardwareMap.get(ServoEx.class, "cA");
        ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 270, AngleUnit.RADIANS);
        ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 270, AngleUnit.RADIANS);
        ServoEx intakeRotateLeft = hardwareMap.get(ServoEx.class, "iRL");
        ServoEx intakeRotateRight = hardwareMap.get(ServoEx.class, "iRR");
        RevIMU imu = new RevIMU(hardwareMap);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        imu.init();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        vSlides.stopAndResetEncoder();


        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        boolean AlertFound   = Alert.exists();
        telemetry.addData("Alert sound",   AlertFound ?   "Found" : "NOT Found \nCopy alert.wav to " + soundPath  );

        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setInverted(true);
        claw.setInverted(true);
        vSlides.setInverted(true);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower.startTeleopDrive();
        follower.setStartingPose(startPose);

        waitForStart();
        runtime.reset();

        while (opModeInInit()) {
            telemetry.addLine("Read carefully: Align the robot such that it is aligned with the line facing the submersible on the tile right next to the net zone");
        }

        while (opModeIsActive()) {

            if (gamepad1.left_stick_button) {
                while (opModeIsActive()) {
                    drive.driveFieldCentric(
                            driverOp.getLeftX(),
                            driverOp.getLeftY(),
                            driverOp.getRightX(),
                            imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                            false
                    );
                }
            } else if (gamepad1.right_stick_button) {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
            follower.update();

            telemetry.addLine("To use field centric mode, press left stick.");
            telemetry.addLine("To use robot centric mode, press right stick.");

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double out = gamepad1.right_trigger;
            double in = gamepad1.left_trigger;
            double clawUp = gamepad2.right_trigger;
            double clawDown = gamepad2.left_trigger;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double horizontalPower = in - out;
            double clawVerticalPower = clawUp - clawDown;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(horizontalPower), Math.abs(clawVerticalPower));

            if (max > 1.0) {
                horizontalPower /= max;
                clawVerticalPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setInverted() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            linSlideLeft.setPower(horizontalPower);
            linSlideRight.setPower(horizontalPower);
            vSlides.set(clawVerticalPower);

            if (gamepad2.dpad_up){
                claw.setPosition(0);
                clawRotateLeft.turnToAngle(135);
                clawRotateRight.turnToAngle(135);;
                clawAdjust.setPosition(.75);
                telemetry.addLine("Sample scoring");
                vSlides.setTargetDistance(5);
                vSlides.set(-1);
                telemetry.addLine("Adjusting viper slides automatically");
            } else if (gamepad2.dpad_down){
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                telemetry.addLine("Resetting claw to intake");
            } else if (gamepad2.dpad_left) {
                clawRotateLeft.setPosition(1);
                clawRotateRight.setPosition(1);
                clawAdjust.setPosition(1);
                vSlides.setTargetPosition(1);
                vSlides.set(-1);
                claw.setPosition(1);
                telemetry.addLine("Specimen pickup");
            }

            if (gamepad2.y) {
                claw.setPosition(1);
                telemetry.addLine("Claw opened all the way | MANUAL OPERATION OF CLAW");
            } else if (gamepad2.b) {
                claw.setPosition(0);
                telemetry.addLine("Claw closed | MANUAL OPERATION OF CLAW");
            }

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            if (gamepad1.x) {
                intakeRotateLeft.setPosition(.15);
                intakeRotateRight.setPosition(.15);
            } else if (gamepad1.x && result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
                intakeRotateLeft.setPosition(.15);
                intakeRotateRight.setPosition(.15);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                telemetry.addLine("Intake in position for sample pickup");
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                claw.setPosition(1);
                telemetry.addLine("Adjusting claw automatically");
            } else if (gamepad1.x && result.closestSwatch == PredominantColorProcessor.Swatch.RED) {
                intakeRotateLeft.setPosition(.15);
                intakeRotateRight.setPosition(.15);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                telemetry.addLine("Intake in position for sample pickup");
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                claw.setPosition(1);
                telemetry.addLine("Adjusting claw automatically");
            } else if (gamepad1.x && AlertFound && result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                telemetry.addLine("WRONG COLOR!");
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, Alert);
            } else if (gamepad1.a) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
                intakeRotateLeft.setPosition(.1);
                intakeRotateRight.setPosition(.1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeRotateLeft.setPosition(0);
                intakeRotateRight.setPosition(0);
                telemetry.addLine("In position for claw pickup");
            }

            if (gamepad1.touchpad_finger_1) {
                intakeRotateRight.setPosition(.15);
                intakeRotateRight.setPosition(.15);
            } if (gamepad1.touchpad_finger_2) {
                intakeRotateLeft.setPosition(0);
                intakeRotateRight.setPosition(0);
            }

            if (follower.getPose().getX() > (observationZone.getX() - 25) && follower.getPose().getY() > (observationZone.getY() - 31)) {
                clawRotateLeft.setPosition(1);
                clawRotateRight.setPosition(1);
                clawAdjust.setPosition(1);
                vSlides.setTargetPosition(1);
                vSlides.set(-1);
                claw.setPosition(1);
                telemetry.addLine("Specimen pickup");
            }

            if (follower.getPose().getX() > (basket.getX() - 24) && follower.getPose().getY() <  (basket.getY())) {
                claw.setPosition(0);
                clawRotateLeft.turnToAngle(135);
                clawRotateRight.turnToAngle(135);;
                clawAdjust.setPosition(.75);
                telemetry.addLine("Sample scoring");
                vSlides.setTargetDistance(5);
                vSlides.set(-1);
                telemetry.addLine("Adjusting viper slides automatically");
            }

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            follower.update();
        }
    }}
