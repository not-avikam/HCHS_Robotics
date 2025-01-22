

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Observation Zone debug help", group = "Silver Knight")
public class test extends OpMode{
    //private PoseUpdater poseUpdater;
    //private DashboardPoseTracker dashboardPoseTracker;
    PIDFController pidf = new PIDFController(0, 0, 0, 0);
    //hardware
    //auto stuff
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(8, 56, Math.toRadians(0));  // Starting position
    private final Pose scorePose1 = new Pose(38, 70, Math.toRadians(180));
    private final Pose pickupPose1 = new Pose(57, 25, Math.toRadians(90));
    private final Pose pickupPose1ControlPose1 = new Pose(0.5, 77, Math.toRadians(90));
    private final Pose pickupPose1ControlPose2 = new Pose(82, .8, Math.toRadians(90));
    private final Pose pickupPose1ControlPose3 = new Pose(57, 39, Math.toRadians(90));
    private final Pose dropOff1 = new Pose(13, 27, Math.toRadians(90));
    private final Pose pickUpPose2 = new Pose(51, 17, Math.toRadians(90));
    private final Pose pickUpPose2ControlPose1 = new Pose(90, 27, Math.toRadians(90));
    private final Pose dropOff2 = new Pose(15, 19, Math.toRadians(90));
    private final Pose dropOff3 = new Pose(16, 10.6, Math.toRadians(90));
    private final Pose dropOff3ControlPose1 = new Pose(140, 7);

    private final Pose pickupSpecimenPose = new Pose(10, 28, Math.toRadians(180)); // First sample pickup
    private final Pose scorePose2 = new Pose(38, 76, Math.toRadians(0)); // Third sample pickup
    private final Pose scorePose3 = new Pose(38, 74, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(38, 72, Math.toRadians(0));
    private final Pose scorePose5 = new Pose(38, 68, Math.toRadians(0));
    private final Pose parkPose = new Pose(18, 24, Math.toRadians(0));
    private PathChain park, score2return, score3return, score4return, score2, score3, score4, score5, pickupSpecimen, scorePreload, pickup1, pickUpSample3, dropOffSample1, dropOffSample2, pickUpSample2;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickupPose1ControlPose1), new Point(pickupPose1ControlPose2), new Point(pickupPose1ControlPose3), new Point(pickupPose1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickupPose1.getHeading())
                .build();

        dropOffSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose1), new Point(dropOff1)))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), dropOff1.getHeading())
                .build();

        pickUpSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropOff1), new Point(pickUpPose2ControlPose1), new Point(pickUpPose2)))
                .setLinearHeadingInterpolation(dropOff1.getHeading(), pickUpPose2.getHeading())
                .build();

        dropOffSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(dropOff2)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), dropOff2.getHeading())
                .build();

        pickUpSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropOff2),new Point(dropOff3ControlPose1), new Point(dropOff3)))
                .setLinearHeadingInterpolation(dropOff2.getHeading(), dropOff3.getHeading())
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

        //TODO: Adjust the vSlides parameters
        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        vSlides.setRunMode(Motor.RunMode.PositionControl);
        vSlides.setPositionCoefficient(0.05);
        vSlides.setDistancePerPulse(0.015);
        vSlides.stopAndResetEncoder();

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        intakeRotateRight.setInverted(true);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setInverted(false);
        clawRotateRight.setInverted(true);;
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
                follower.followPath(scorePreload, true);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //vSlides.setTargetPosition(5);
                    //vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    follower.followPath(pickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
//                    vSlides.setTargetPosition(0);
//                    vSlides.set(-1);
                    follower.followPath(dropOffSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(dropOffSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pickupSpecimen, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
//                    vSlides.setTargetPosition(1);
//                    vSlides.set(-1);
                    follower.followPath(score2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
//                    vSlides.setTargetPosition(5);
//                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(score2return, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
//                    vSlides.setTargetPosition(1);
//                    vSlides.set(-1);
                    follower.followPath(score3,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
//                    vSlides.setTargetPosition(5);
//                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(score3return,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
//                    vSlides.setTargetPosition(1);
//                    vSlides.set(-1);
                    follower.followPath(score4,true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
//                    vSlides.setTargetPosition(5);
//                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(score4return, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
//                    vSlides.setTargetPosition(1);
//                    vSlides.set(-1);
                    follower.followPath(score5,true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
//                    vSlides.setTargetPosition(5);
//                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(park, true);
                    //Sets to a non existent pathstate so that it doesn't keep running
                    //-Avikam ;)
                    setPathState(15);
                }
                break;
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

        // These loop the movements of the robot
        follower.update();
        //dashboardPoseTracker.update();
        autonomousPathUpdate();


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
