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

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="Scrimage Blue", group="Scrimmage")
public class ScrimmageBlue extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    //private final String soundPath = "/FIRST/blocks/sounds";
    //private final File Alert   = new File("/sdcard" + soundPath + "/gold.wav");

    private final Pose startPose = new Pose(8, 19, Math.toRadians(0));
    private DcMotorEx vSlideLeft = null;
    private DcMotorEx vSlideRight = null;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assign
        // ed during the robot configuration step on the DS or RC devices.
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        vSlideLeft = hardwareMap.get(DcMotorEx.class, "VSL");
        vSlideRight = hardwareMap.get(DcMotorEx.class, "VSR");
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo clawAdjust = hardwareMap.get(Servo.class, "cA");
        ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 300, AngleUnit.DEGREES);
        ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 300, AngleUnit.DEGREES);
        Servo intakeRotateLeft = hardwareMap.get(Servo.class, "iRL");
        Servo intakeRotateRight = hardwareMap.get(Servo.class, "iRR");

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
        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setInverted(false);
        clawRotateRight.setInverted(true);
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

            if (gamepad1.left_bumper) {
                follower.setTeleOpMovementVectors((-gamepad1.left_stick_y*.5), (-gamepad1.left_stick_x*.5), (-gamepad1.right_stick_x*.5), false);
            }

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
            //      the setDirection() calls above.s
            // Once the correct motors move in the correct direction re-comment this code.



            // Send calculated power to wheels
            vSlideLeft.setPower(clawVerticalPower);
            vSlideRight.setPower(clawVerticalPower);
            //linSlideLeft.setPower(horizontalPower);
            //linSlideRight.setPower(horizontalPower);

            if (gamepad2.dpad_up){
                clawRotateLeft.setPosition(.833);
                clawRotateRight.setPosition(.833);
                clawAdjust.setPosition(.75);
                claw.setPosition(0);
                telemetry.addLine("Sample scoring");
                vSlideLeft.setPower(-1);
                vSlideRight.setPower(-1);
                telemetry.addLine("Adjusting viper slides automatically");
            } else if (gamepad2.dpad_down){
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                //clawAdjust.setPosition(O.25)
                clawAdjust.setPosition(.12-.0277);
                telemetry.addLine("Resetting claw to intake");
            } else if (gamepad2.dpad_left) {
                clawRotateLeft.setPosition(.11);
                clawRotateRight.setPosition(.11);
                clawAdjust.setPosition(0.5);
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
                intakeRotateLeft.setPosition(.025);
                intakeRotateRight.setPosition(.17);
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

            if (gamepad1.right_trigger !=0) {
                linSlideRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
                linSlideLeft.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            } else if (gamepad1.left_trigger !=0) {
                linSlideRight.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
                linSlideLeft.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            } else {
                linSlideRight.setPower(0);
                linSlideLeft.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Position for viper slide", "Position:", vSlideLeft.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
