import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Submersible Test - Silver Knight", group = "Silver Knight")
public class SubmersibleTest extends OpMode{
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
    PredominantColorProcessor.Result result = colorSensor.getAnalysis();

    //hardware
    private MotorEx vSlideLeft = null;
    private MotorEx vSlideRight = null;

    //auto stuff
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(10, 104, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickupPose1 = new Pose(36, 121, Math.toRadians(0));
    private final Pose pickUpPose2 = new Pose(36, 131, Math.toRadians(0));
    private final Pose pushPickUpPose = new Pose(59, 131, Math.toRadians(45));
    private final Pose pushPickUpControl = new Pose(46, 117);
    private final Pose pushScorePose = new Pose(17, 131.5, Math.toRadians(45));
    private final Pose hangPose = new Pose(65, 97, Math.toRadians(90));
    private final Pose hangControlPose = new Pose(70, 128);
    private final Pose specimenScorePose = new Pose(37, 76, Math.toRadians(180));
    private final Pose submersiblePose1 = new Pose(59, 97, Math.toRadians(-90));
    private final Pose submersiblePose2 = new Pose(86, 97, Math.toRadians(-90));
    private PathChain scorePreLoad, pickUp1, score1, pickUp2, score2, pushPickUp, pushScore, hang, submersible, search, scoreInBasket;
    public void buildPaths() {
        scorePreLoad = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(specimenScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePose), new Point(pickupPose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupPose1.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose1), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), scorePose.getHeading())
                .build();

        pickUp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickUpPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpPose2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), scorePose.getHeading())
                .build();

        pushPickUp = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pushPickUpControl), new Point(pushPickUpPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pushPickUpPose.getHeading())
                .build();

        pushScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPickUpPose), new Point(pushScorePose)))
                .setLinearHeadingInterpolation(pushPickUpPose.getHeading(), pushScorePose.getHeading())
                .build();

        hang = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushScorePose), new Point(hangControlPose), new Point(hangPose)))
                .setLinearHeadingInterpolation(pushScorePose.getHeading(), hangPose.getHeading())
                .build();

        pushScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPickUpPose), new Point(pushScorePose)))
                .setLinearHeadingInterpolation(pushPickUpPose.getHeading(), pushScorePose.getHeading())
                .build();

        submersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushScorePose), new Point(submersiblePose1)))
                .setLinearHeadingInterpolation(pushScorePose.getHeading(), submersiblePose1.getHeading())
                .build();

        search = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePose1), new Point(submersiblePose2)))
                .setLinearHeadingInterpolation(submersiblePose1.getHeading(), submersiblePose2.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {

        vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        ServoEx claw = new SimpleServo(hardwareMap, "claw", 0, 180, AngleUnit.DEGREES);
        ServoEx clawAdjust = new SimpleServo(hardwareMap, "cA", 0, 180, AngleUnit.DEGREES);
        ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 270, AngleUnit.DEGREES);
        ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 270, AngleUnit.DEGREES);
        ServoEx intakeRotateLeft = new SimpleServo(hardwareMap, "iRL", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeRotateRight = new SimpleServo(hardwareMap, "iRR", 0, 300, AngleUnit.DEGREES);
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        vSlides.setRunMode(Motor.RunMode.PositionControl);
        vSlides.encoder.setDistancePerPulse(0.00102);
        vSlides.stopAndResetEncoder();

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
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                follower.followPath(scorePreLoad, true);
                telemetry.addLine("pathState 0");
                break;
            case 1:
                if (!follower.isBusy()) {
                    targetDistance = 2.16667-1.19500;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickUp1, true);
                    telemetry.addLine("pathState 1");
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    targetDistance = 0;
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    //clawAdjust.setPosition(O.25)
                    clawAdjust.setPosition(.12-.0277);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (clawRotateLeft.getPosition() == 0 && clawRotateRight.getPosition() == 0) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score1, true);
                    setPathState(3);
                    telemetry.addLine("pathState 2");
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    targetDistance = 3.58333333-1.19500;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    if (actionTimer.getElapsedTimeSeconds() >= .2) {
                        intakeRotateLeft.setPosition(.05);
                        intakeRotateRight.setPosition(.05);
                        intakeLeft.setPower(-1);
                        intakeRight.setPower(-1);
                    }
                    follower.followPath(pickUp2, true);
                    setPathState(4);
                    telemetry.addLine("pathState 3");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    targetDistance = 0;
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    //clawAdjust.setPosition(O.25)
                    clawAdjust.setPosition(.12-.0277);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (clawRotateLeft.getPosition() == 0 && clawRotateRight.getPosition() == 0) {
                        claw.setPosition(0);
                    }
                    follower.followPath(score2, true);
                    setPathState(5);
                    telemetry.addLine("pathState 4");
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    targetDistance = 3.58333333-1.19500;
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    if (actionTimer.getElapsedTimeSeconds() >= .2) {
                        intakeRotateLeft.setPosition(.05);
                        intakeRotateRight.setPosition(.05);
                        intakeLeft.setPower(-1);
                        intakeRight.setPower(-1);
                    }
                    follower.followPath(pushPickUp, true);
                    telemetry.addLine("pathState 5");
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    targetDistance = 0;
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    //clawAdjust.setPosition(O.25)
                    clawAdjust.setPosition(.12-.0277);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(0);
                    }
                    setPathState(7);
                    follower.followPath(pushScore, true);
                    telemetry.addLine("pathState 6");
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    targetDistance = 0;
                    intakeRotateLeft.setPosition(.05);
                    intakeRotateRight.setPosition(.05);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.75);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(0);
                    }
                    follower.followPath(submersible, false);
                    telemetry.addLine("pathState 7");
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(search, true);
                linSlideLeft.setPower(.5);
                linSlideRight.setPower(.5);
                clawRotateLeft.setPosition(.5);
                clawRotateRight.setPosition(.5);
                clawAdjust.setPosition(.5);
                intakeRotateLeft.setPosition(.025);
                intakeRotateRight.setPosition(.17);
                actionTimer.resetTimer();
                if (actionTimer.getElapsedTimeSeconds() == .5) {
                    claw.setPosition(1);
                }
                setPathState(11);
                telemetry.addLine("pathState 7");
                break;

            case 9:
                if (opmodeTimer.getElapsedTimeSeconds() > 27) {
                    setPathState(10);
                } else if (!follower.isBusy() && opmodeTimer.getElapsedTimeSeconds() < 27) {
                    targetDistance = 3.58333333-1.19500;
                    intakeRotateLeft.setPosition(.05);
                    intakeRotateRight.setPosition(.05);
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    setPathState(7);
                } break;

            case 10:
                hang = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(hangPose)))
                        .setLinearHeadingInterpolation(pushScorePose.getHeading(), hangPose.getHeading())
                        .build();
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                intakeRotateLeft.setPosition(.025);
                intakeRotateRight.setPosition(.17);
                targetDistance = 3.58333333-1.19500;
                clawRotateLeft.setPosition(.833);
                clawRotateRight.setPosition(.833);
                clawAdjust.setPosition(.75);
                if (actionTimer.getElapsedTimeSeconds() == 1) {
                    claw.setPosition(0);
                }
                follower.followPath(hang, true);
                setPathState(12);
                break;

            case 11:
                if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12-.0277);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    claw.setPosition(0);
                    scoreInBasket = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.getPose()), new Point(scorePose)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    follower.followPath(scoreInBasket, true);
                    setPathState(9);
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    linSlideLeft.setPower(-1);
                    linSlideRight.setPower(-1);
                    claw.setPosition(0);
                    scoreInBasket = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.getPose()), new Point(scorePose)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    follower.followPath(scoreInBasket, true);
                    setPathState(9);
                }
                break;
        }
        PIDFController pidf = new PIDFController(0, 0, 0, 0);
        pidf.setSetPoint(targetDistance);
        while (!pidf.atSetPoint()) {
            double outputLeft = pidf.calculate(
                    vSlideLeft.encoder.getPosition()
            );

            double outputRight = pidf.calculate(
                    vSlideRight.encoder.getPosition()
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

        // These loop the movements of the robot
        follower.update();
        //dashboardPoseTracker.update();
        autonomousPathUpdate();

        if (opmodeTimer.getElapsedTimeSeconds() > 27) {
            setPathState(9);
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("preview on/off", "... Camera Stream\n");
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

        telemetry.addLine("Align on the field tile next to the basket, with the edge of the robot aligned with the edge of the field tile facing the observation zone");
        telemetry.addLine("There should be 1 tile to the left of the robot and 4 to the right");
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
