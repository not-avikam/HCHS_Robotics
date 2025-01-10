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

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="sexperimental", group="Linear OpMode")
public class avikamsexperimentalcode extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    //private final String soundPath = "/FIRST/blocks/sounds";
    //private final File Alert   = new File("/sdcard" + soundPath + "/gold.wav");

    private final Pose startPose = new Pose(8, 19, Math.toRadians(0));

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx vSlideLeft = hardwareMap.get(DcMotorEx.class, "VSL");
        DcMotorEx vSlideRight = hardwareMap.get(DcMotorEx.class, "VSR");
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo clawAdjust = hardwareMap.get(Servo.class, "cA");
        Servo clawRotateLeft = hardwareMap.get(Servo.class, "cRL");
        Servo clawRotateRight = hardwareMap.get(Servo.class, "cRR");
        Servo intakeRotateLeft = hardwareMap.get(Servo.class, "iRL");
        Servo intakeRotateRight = hardwareMap.get(Servo.class, "iRR");

        vSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        vSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        vSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        //boolean AlertFound   = Alert.exists();
        //telemetry.addData("Alert sound",   AlertFound ?   "Found" : "NOT Found \nCopy alert.wav to " + soundPath  );

        intakeRotateRight.setDirection(Servo.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        vSlideRight.setDirection(DcMotorEx.Direction.REVERSE);

        follower.startTeleopDrive();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // input motors exactly as shown below
            telemetry.addLine("To use field centric mode, press right stick.");
            telemetry.addLine("To use robot centric mode, press left stick.");
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            telemetry.addLine("ROBOT CENTRIC MODE ACTIVE");

            follower.update();

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
            //      the setDirection() calls above.
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

            if (gamepad2.dpad_up){
                clawRotateLeft.setPosition(.5);
                clawRotateRight.setPosition(.5);
                clawAdjust.setPosition(.75);
                claw.setPosition(0);
                telemetry.addLine("Sample scoring");
                vSlideLeft.setPower(1);
                vSlideRight.setPower(1);
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
                telemetry.addLine("Specimen scoring");
            }

            if (gamepad2.y) {
                claw.setPosition(1);
                telemetry.addLine("Claw opened all the way | MANUAL OPERATION OF CLAW");
            } else if (gamepad2.b) {
                claw.setPosition(0);
                telemetry.addLine("Claw closed | MANUAL OPERATION OF CLAW");
            }

            /*
            if (gamepad1.x && result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
                intake.setPower(1);
            } else if (gamepad1.x && result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                intake.setPower(1);
            } else if (gamepad1.x && AlertFound && result.closestSwatch == PredominantColorProcessor.Swatch.RED) {
                telemetry.addLine("WRONG COLOR!");
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, Alert);
            } else if (gamepad1.a) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
             */

            if (gamepad1.x) {
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

            if (gamepad2.right_trigger != 0) {
                vSlideLeft.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                vSlideRight.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            } else if (gamepad2.left_trigger != 0) {
                vSlideLeft.setPower(-gamepad2.right_trigger+gamepad2.left_trigger);
                vSlideRight.setPower(-gamepad2.right_trigger+gamepad2.left_trigger);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Position for viper slide", "Position:", vSlideLeft.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
