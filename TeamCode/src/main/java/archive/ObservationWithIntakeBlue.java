package archive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Disabled
@Autonomous(name = "Blue Autonomous Observation with Intake - Silver Knight", group = "Silver Knight")
public class ObservationWithIntakeBlue extends OpMode{
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private MotorEx vSlideRight = null;
    private MotorEx vSlideLeft = null;
    private final Pose startPose = new Pose(10, 56, Math.toRadians(0));  // Starting position
    private final Pose scorePose1 = new Pose(37, 70, Math.toRadians(180));
    private final Pose pickupPose1 = new Pose(34, 24, Math.toRadians(0));
    private final Pose pickupPose1ControlPose1 = new Pose(22, 44);
    private final Pose dropOff1 = new Pose(13, 27, Math.toRadians(180));
    private final Pose pickUpPose2 = new Pose(34, 12.5, Math.toRadians(0));
    private final Pose dropOff2 = new Pose(18, 11, Math.toRadians(180));
    private final Pose pickUpPose3 = new Pose(63, 13.5, Math.toRadians(180));
    private final Pose dropOff3 = new Pose(11, 9.5, Math.toRadians(180));
    private final Pose dropOff3ControlPose1 = new Pose(90, 6);

    private final Pose pickupSpecimenPose = new Pose(11, 28, Math.toRadians(180)); // First sample pickup
    private final Pose scorePose2 = new Pose(37, 74, Math.toRadians(180)); // Third sample pickup
    private final Pose scorePose3 = new Pose(37, 72, Math.toRadians(180));
    private final Pose scorePose4 = new Pose(37, 68, Math.toRadians(180));
    private final Pose scorePose5 = new Pose(37, 64, Math.toRadians(180));
    private final Pose parkPose = new Pose(18, 24, Math.toRadians(180));
    private PathChain park, dropOffSample3, score2return, score3return, score4return, score2, score3, score4, score5, pickupSpecimen, scorePreload, pickup1, pickUpSample3, dropOffSample1, dropOffSample2, pickUpSample2;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickupPose1ControlPose1), new Point(pickupPose1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickupPose1.getHeading())
                .build();

        dropOffSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose1), new Point(dropOff1)))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), dropOff1.getHeading())
                .build();

        pickUpSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropOff1), new Point(pickUpPose2)))
                .setLinearHeadingInterpolation(dropOff1.getHeading(), pickUpPose2.getHeading())
                .build();

        dropOffSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(dropOff2)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), dropOff2.getHeading())
                .build();

        pickUpSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOff2), new Point(pickUpPose3)))
                .setLinearHeadingInterpolation(dropOff2.getHeading(), pickUpPose3.getHeading())
                .build();

        dropOffSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpPose3), new Point(dropOff3ControlPose1), new Point(dropOff3)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), dropOff3.getHeading())
                .build();

        pickupSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOff3), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(dropOff3.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose2.getHeading())
                .build();

        score2return = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose3.getHeading())
                .build();

        score3return = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose4)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose4.getHeading())
                .build();

        score4return = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose4), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose5)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose5.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose5), new Point(parkPose)))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
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
        vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);

        //TODO: Adjust the vSlides parameters
//        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
//        vSlides.setRunMode(Motor.RunMode.PositionControl);
//        vSlides.setPositionCoefficient(0.05);
//        vSlides.setDistancePerPulse(0.015);
//        vSlides.stopAndResetEncoder();

        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRotateLeft.setInverted(true);
        clawRotateLeft.setInverted(true);
        claw.setInverted(true);
        vSlides.setInverted(true);
        vSlideRight.setInverted(false);

        double targetDistance = 0;

        switch (pathState) {
            case 0:
                setPathState(1);
                follower.followPath(scorePreload, true);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //targetDistance = 5;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(1);
                    intakeLeft.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    setPathState(3);
                    follower.followPath(dropOffSample1, true);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(-1);
                    intakeLeft.setPower(-1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.05);
                    intakeRotateRight.setPosition(.05);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickUpSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(1);
                    intakeLeft.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(dropOffSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(-1);
                    intakeLeft.setPower(-1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.05);
                    intakeRotateRight.setPosition(.05);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickUpSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(1);
                    intakeLeft.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(dropOffSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    intakeRight.setPower(-1);
                    intakeLeft.setPower(-1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.05);
                    intakeRotateRight.setPosition(.05);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickupSpecimen,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == 1) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //targetDistance = 5;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(score2return,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == 1) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score3,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    //targetDistance = 5;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(score3return,true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == 1) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score4, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    //targetDistance = 5;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(score4return,true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 22) {
                    //targetDistance = 0;
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    clawAdjust.setPosition(0.5);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == 1) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score5, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    //targetDistance = 5;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() == .5) {
                        claw.setPosition(1);
                    }
                    follower.followPath(park,true);
                    setPathState(16);
                    //Sets to a non existent pathstate so that it doesn't keep running
                    //-Avikam ;)
                }
                break;
        }
        PIDFController pidf = new PIDFController(0, 0, 0, 0);
        pidf.setSetPoint(targetDistance);
        while (!pidf.atSetPoint()) {
            double outputLeft = pidf.calculate(
                    vSlideLeft.getCurrentPosition()
            );

            double outputRight = pidf.calculate(
                    vSlideRight.getCurrentPosition()
            );

            vSlideLeft.setVelocity(outputLeft);
            vSlideRight.setVelocity(outputRight);
        }
        vSlideLeft.stopMotor();
        vSlideRight.stopMotor();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        if (pathTimer.getElapsedTime() > 22) {
            setPathState(14);
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        /*
        while (!vSlides.atTargetPosition()) {
            vSlides.set(1);
        }
         */

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addLine("Align on the field tile next to the observation zone, with the edge of the robot aligned with the edge of the field tile facing the observation zone");
        telemetry.addLine("There should be 3 tiles to the left of the robot and 2 to the right");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
