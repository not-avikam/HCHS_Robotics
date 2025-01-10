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
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.io.File;

@TeleOp(name="blue", group="LM3")
public class blue extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final String soundPath = "/FIRST/blocks/sounds";
    private final File Alert  = new File( soundPath + "/Alert.wav");
    static final boolean FIELD_CENTRIC = false;
    private final Pose startPose = new Pose(8, 19, Math.toRadians(0));
    private final Pose observationZone = new Pose(0, 0);
    private final Pose basket = new Pose(0, 144);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312)
        );
        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        ServoEx claw = new SimpleServo(hardwareMap, "claw", 0, 180, AngleUnit.DEGREES);
        ServoEx clawAdjust = new SimpleServo(hardwareMap, "cA", 0, 180, AngleUnit.DEGREES);
        ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 270, AngleUnit.DEGREES);
        ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 270, AngleUnit.DEGREES);
        ServoEx intakeRotateLeft = new SimpleServo(hardwareMap, "iRL", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeRotateRight = new SimpleServo(hardwareMap, "iRR", 0, 300, AngleUnit.DEGREES);
        //BNO055IMUNew imu = hardwareMap.get(BNO055IMUNew.class, "imu");
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx clawOp = new GamepadEx(gamepad2);
        RevIMU imu = new RevIMU(hardwareMap, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );

        imu.init();

        /*
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

         */

        boolean AlertFound   = Alert.exists();
        telemetry.addData("Alert sound",   AlertFound ?   "Found" : "NOT Found \nCopy alert.wav to " + soundPath  );

        ToggleButtonReader fieldCentricReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader robotCentricReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON
        );

        ToggleButtonReader hangModeRight = new ToggleButtonReader(
                clawOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader hangModeLeft = new ToggleButtonReader(
                clawOp, GamepadKeys.Button.LEFT_STICK_BUTTON
        );

        ButtonReader intakeOnReader = new ButtonReader(
                driverOp, GamepadKeys.Button.X
        );

        TriggerReader vSlideUpReader = new TriggerReader(
                clawOp, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        TriggerReader vSlideDownReader = new TriggerReader(
                clawOp, GamepadKeys.Trigger.LEFT_TRIGGER
        );

        //TODO: Adjust the vSlides parameters
        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        vSlides.setRunMode(Motor.RunMode.PositionControl);
        vSlides.setPositionCoefficient(0.05);
        vSlides.setDistancePerPulse(0.015);
        vSlides.stopAndResetEncoder();

        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRotateLeft.setInverted(true);
        clawRotateLeft.setInverted(true);
        claw.setInverted(true);
        vSlides.setInverted(true);
        vSlideRight.setInverted(false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower.startTeleopDrive();
        follower.setStartingPose(startPose);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button) {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                follower.update();
            } else if (gamepad1.right_stick_button){
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            }

            telemetry.addLine("To use field centric mode, press left stick.");
            telemetry.addLine("To use robot centric mode, press right stick.");

            follower.update();

            double in = gamepad1.left_trigger;
            double out = gamepad1.right_trigger;

            double max;

            double horizontalPower = out - in;

            //max = Math.max(Math.abs(horizontalPower), Math.abs(clawVerticalPower));

            /*
            if (max > 1.0) {
                horizontalPower /= max;
                //clawVerticalPower /= max;
            }

             */

            // Send calculated power
            linSlideLeft.setPower(horizontalPower);
            linSlideRight.setPower(horizontalPower);

            if (gamepad2.dpad_up){
                claw.setPosition(0);
                clawRotateLeft.turnToAngle(135);
                clawRotateRight.turnToAngle(135);
                clawAdjust.setPosition(.75);
                telemetry.addLine("Sample scoring");
                vSlides.setTargetPosition(5);
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
                claw.setPosition(1);
                telemetry.addLine("Specimen pickup");
            } else if (clawOp.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                vSlides.set(0);
            } else if (clawOp.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                vSlides.set(0);
            } else if (clawOp.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                vSlides.set(0);
            }

            if (gamepad2.right_trigger != 0) {
                vSlides.setTargetPosition(5);
                vSlides.set(gamepad2.right_trigger-gamepad2.left_trigger);
            } else if (gamepad2.left_trigger != 0) {
                vSlides.setTargetPosition(-5);
                vSlides.set(gamepad2.right_trigger-gamepad2.left_trigger);
            } else if (vSlideUpReader.wasJustReleased()) {
                vSlides.set(0);
            } else if (vSlideDownReader.wasJustReleased()) {
                vSlides.set(0);
            }

            if (gamepad2.y) {
                claw.setPosition(1);
                telemetry.addLine("Claw opened all the way | MANUAL OPERATION OF CLAW");
            } else if (gamepad2.b) {
                claw.setPosition(0);
                telemetry.addLine("Claw closed | MANUAL OPERATION OF CLAW");
            }

            //PredominantColorProcessor.Result result = colorSensor.getAnalysis();


            if (gamepad1.x) {
                intakeRotateLeft.setPosition(.15);
                intakeRotateRight.setPosition(.15);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                claw.setPosition(1);
            }  else if (gamepad1.a) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
                intakeRotateLeft.setPosition(.1);
                intakeRotateRight.setPosition(.1);
            } else{
                intakeRotateLeft.setPosition(0);
                intakeRotateRight.setPosition(0);
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                telemetry.addLine("Intake in position for claw pickup");
            }

            /*
            if (result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                telemetry.addLine("Intake in position for sample pickup");
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                claw.setPosition(1);
                telemetry.addLine("Adjusting claw automatically");
            } else if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                telemetry.addLine("Intake in position for sample pickup");
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.25);
                claw.setPosition(1);
                telemetry.addLine("Adjusting claw automatically");
            } else if (AlertFound && result.closestSwatch == PredominantColorProcessor.Swatch.RED) {
                telemetry.addLine("WRONG COLOR!");
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, Alert);
            }
             */

            if (follower.getPose().getX() < (observationZone.getX() + 25) && follower.getPose().getY() < (observationZone.getY() + 31)) {
                clawRotateLeft.setPosition(1);
                clawRotateRight.setPosition(1);
                clawAdjust.setPosition(1);
                vSlides.setTargetPosition(1);
                claw.setPosition(1);
                telemetry.addLine("Specimen pickup");
            }

            if (follower.getPose().getX() < (basket.getX() + 24) && follower.getPose().getY() > (basket.getY()) - 24) {
                claw.setPosition(0);
                clawRotateLeft.turnToAngle(135);
                clawRotateRight.turnToAngle(135);
                clawAdjust.setPosition(.75);
                telemetry.addLine("Sample scoring");
                vSlides.setTargetPosition(5);
                telemetry.addLine("Adjusting viper slides automatically");
            }

            if (hangModeRight.getState() && hangModeLeft.getState()) {
                while (opModeIsActive()) {
                    vSlides.setRunMode(Motor.RunMode.RawPower);
                    vSlides.set(clawOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-clawOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                }
            }

            while (!vSlides.atTargetPosition()) {
                vSlides.set(1);
            }

            //telemetry.addData("Vslides position", "%.2f", vSlides.getCurrentPosition());
            //telemetry.addData("Vslides distance", "%.2f", vSlides.getDistance());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            follower.update();
        }
    }
}