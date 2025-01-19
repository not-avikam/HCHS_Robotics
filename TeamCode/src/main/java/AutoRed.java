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
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//TODO: I still haven't coded anything different from Auto blue. I plan on doing it after testing blue.
@Autonomous(name = "Red Autonomous", group = "LM3")
public class AutoRed extends OpMode{
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private Follower follower;
    private DcMotorEx vSlideLeft = null;
    private DcMotorEx vSlideRight = null;
    private CRServo linSlideLeft = null;
    private CRServo linSlideRight = null;
    private CRServo intakeLeft = null;
    private CRServo intakeRight = null;
    private Servo clawRotateLeft = null;
    private Servo clawRotateRight = null;
    private Servo intakeRotateLeft = null;
    private Servo intakeRotateRight = null;
    private Servo claw = null;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(8.983, 100.139, Math.toRadians(0));
    private final Pose preloadScoreControlPose = new Pose(31.706, 100.404, Math.toRadians(0));
    private final Pose scorePose = new Pose(37, 72, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(37, 65, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(37, 79, Math.toRadians(180));
    private final Pose scorePose4 = new Pose(37, 62, Math.toRadians(180));
    private final Pose pickupPose = new Pose(10, 11, Math.toRadians(0));
    private final Pose pickup1ControlPose1 = new Pose(16.382, 67.640, Math.toRadians(0));
    private final Pose pickup1ControlPose2 = new Pose(85.343, 10.305, Math.toRadians(0));
    private final Pose pickup2ControlPose1 = new Pose(7.662, 32.235, Math.toRadians(0));
    private final Pose pickup2ControlPose2 = new Pose(70.811, 8.983, Math.toRadians(0));
    private final Pose pickup2ControlPose3 = new Pose(55.222, 11.097, Math.toRadians(0));
    private final Pose pickupWait = new Pose(25.000, 15.000, Math.toRadians(180));
    private final Pose parkPose = new Pose(8.983, 19.288, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12.154, 79.530, Math.toRadians(90));
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, score1, score2, pickupLast, score3, pickup, pickupBackUp;
    private int pathstate;

    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(preloadScoreControlPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickupPose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine (new Point(pickupPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup2ControlPose1), new Point(pickup2ControlPose2), new Point(pickup2ControlPose3), new Point(pickupPose)))
                .setTangentHeadingInterpolation()
                .build();
        pickupBackUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(pickupWait)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), pickupWait.getHeading())
                .build();
        pickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupWait), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickupWait.getHeading(), pickupPose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine (new Point(pickupPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine (new Point(pickupPose), new Point(scorePose4)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        pickupLast = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {

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

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */
                    vSlides.setTargetPosition(5);
                    vSlides.setTargetPosition(0);
                    claw.setPosition(.3);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Grab Sample */
                    vSlideLeft.setTargetPosition(0);
                    vSlideRight.setTargetPosition(0);
                    claw.setPosition(0);
                    clawRotateLeft.setPosition(.7);
                    clawRotateRight.setPosition(.7);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getX() > (scorePose2.getX() - 1) && follower.getPose().getY() > (scorePose2.getY() - 1)) {
                    /* Score Preload */
                    vSlides.setTargetPosition(5);
                    vSlides.setTargetPosition(0);
                    claw.setPosition(.3);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                } break;

            case 4:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                }

                follower.followPath(pickupBackUp,true);
                setPathState(5);
                break;

            case 5:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Grab Sample */
                    vSlideLeft.setTargetPosition(0);
                    vSlideRight.setTargetPosition(0);
                    claw.setPosition(0);
                    clawRotateLeft.setPosition(.7);
                    clawRotateRight.setPosition(.7);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(6);
                } break;

            case 6:
                if(follower.getPose().getX() > (scorePose3.getX() - 1) && follower.getPose().getY() > (scorePose3.getY() - 1)) {
                    /* Grab Sample */
                    vSlides.setTargetPosition(5);
                    vSlides.setTargetPosition(0);
                    claw.setPosition(.3);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pickupLast, true);
                    setPathState(7);
                }

            case 7:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Grab Sample */
                    vSlideLeft.setTargetPosition(0);
                    vSlideRight.setTargetPosition(0);
                    claw.setPosition(0);
                    clawRotateLeft.setPosition(.7);
                    clawRotateRight.setPosition(.7);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score3,true);
                    setPathState(8);
                } break;

            case 8:
                if(follower.getPose().getX() > (scorePose4.getX() - 1) && follower.getPose().getY() > (scorePose4.getY() - 1)) {
                    /* Score Preload */
                    vSlides.setTargetPosition(5);
                    vSlides.setTargetPosition(0);
                    claw.setPosition(.3);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park,true);
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
        autonomousPathUpdate();

        while (!vSlides.atTargetPosition()) {
            vSlides.set(1);
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