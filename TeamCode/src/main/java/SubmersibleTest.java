

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Blue Net Sample - Silver Knight", group = "Silver Knight")
public class SubmersibleTest extends OpMode{
    //private PoseUpdater poseUpdater;
    //private DashboardPoseTracker dashboardPoseTracker;
    PIDFController pidf = new PIDFController(0, 0, 0, 0);
    //hardware
    //auto stuff
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(9, 104, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(14, 130, Math.toRadians(-45));
    private final Pose pickupPose1 = new Pose(36, 121, Math.toRadians(0));
    private final Pose pickUpPose2 = new Pose(36, 131, Math.toRadians(0));
    private final Pose pushPickUpPose = new Pose(59, 132, Math.toRadians(45));
    private final Pose pushPickUpControl = new Pose(46, 117);
    private final Pose pushScorePose = new Pose(17, 133, Math.toRadians(45));
    private final Pose hangPose = new Pose(59, 96, Math.toRadians(90));
    private final Pose submersiblePose = new Pose(58, 96, Math.toRadians(-90));
    private final Pose searchPose = new Pose (follower.getPose().getX()+1, follower.getPose().getY());
    private PathChain pickUp1, score1, pickUp2, score2, pushPickUp, pushScore, hang, submersible, search, scoreInBasket;
    public void buildPaths() {
        pickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pickupPose1)))
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
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(hangPose)))
                .setLinearHeadingInterpolation(pushScorePose.getHeading(), hangPose.getHeading())
                .build();

        submersible = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushScorePose), new Point(submersiblePose)))
                .setLinearHeadingInterpolation(pushScorePose.getHeading(), submersiblePose.getHeading())
                .build();

        search = follower.pathBuilder()
                .addPath(new BezierLine(new Point(submersiblePose), new Point(searchPose)))
                .setLinearHeadingInterpolation(submersiblePose.getHeading(), searchPose.getHeading())
                .build();

        scoreInBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(searchPose)))
                .setLinearHeadingInterpolation(submersiblePose.getHeading(), searchPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {

        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
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

        //TODO: Adjust the vSlides parameters
        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        vSlides.setRunMode(Motor.RunMode.PositionControl);
        vSlides.setPositionCoefficient(0.05);
        vSlides.setDistancePerPulse(0.015);
        vSlides.stopAndResetEncoder();

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        intakeRotateRight.setInverted(true);
        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setInverted(false);
        clawRotateRight.setInverted(true);
        claw.setInverted(true);
        vSlideRight.setInverted(true);

        double targetDistance = 0;

        //double power = pidf.calculate(vSlides.getCurrentPosition());  // Adjust power based on how fast you want to move

        //double currentDistance = vSlides.getCurrentPosition() /* *number for .setDistancePerPulse*/;
        //double distanceRemaining = targetDistance - currentDistance;
        pidf.setSetPoint(targetDistance);

        switch (pathState) {
            case 0:
                setPathState(1);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                follower.followPath(pickUp1, true);
                telemetry.addLine("pathState 0");
                break;
            case 1:
                if (!follower.isBusy()) {
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    claw.setPosition(1);
                    follower.followPath(score1, true);
                    setPathState(2);
                    telemetry.addLine("pathState 1");
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    //targetDistance = 5;
                    //vSlides.set(power);
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    claw.setPosition(0);
                    follower.followPath(pickUp2, true);
                    setPathState(3);
                    telemetry.addLine("pathState 2");
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                   // targetDistance = 0;
                   // vSlides.set(power);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    claw.setPosition(1);
                    follower.followPath(score2, true);
                    setPathState(4);
                    telemetry.addLine("pathState 3");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                   // targetDistance = 5;
                   // vSlides.set(power);
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    claw.setPosition(0);
                    follower.followPath(pushPickUp, true);
                    setPathState(5);
                    telemetry.addLine("pathState 4");
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                  //  targetDistance = 0;
                   // vSlides.set(power);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    follower.followPath(pushScore, true);
                    telemetry.addLine("pathState 5");
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    follower.followPath(submersible, true);
                    setPathState(7);
                    telemetry.addLine("pathState 6");
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    linSlideLeft.setPower(.7);
                    linSlideRight.setPower(.7);
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    claw.setPosition(1);
                    follower.followPath(search, true);
                    setPathState(8);
                    telemetry.addLine("pathState 7");
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 25) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    clawRotateLeft.setPosition(1);
                    clawRotateRight.setPosition(1);
                    clawAdjust.setPosition(1);
                    follower.followPath(hang, true);
                    setPathState(10);
                }

            case 9:
                intakeRotateLeft.setPosition(0);
                intakeRotateRight.setPosition(0);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                //targetDistance = 5;
                //vSlides.set(power);
                clawRotateLeft.setPosition(.833);
                clawRotateRight.setPosition(.833);
                clawAdjust.setPosition(.75);
                claw.setPosition(0);
                setPathState(7);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");

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

        // These loop the movements of the robot
        follower.update();
        //dashboardPoseTracker.update();
        autonomousPathUpdate();

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            setPathState(9);
        }


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
