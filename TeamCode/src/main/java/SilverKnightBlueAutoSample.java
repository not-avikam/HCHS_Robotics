

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
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Blue Autonomous Net Sample - Silver Knight", group = "Silver Knight")
public class SilverKnightBlueAutoSample extends OpMode{
    //private PoseUpdater poseUpdater;
    //private DashboardPoseTracker dashboardPoseTracker;
    PIDFController pidf = new PIDFController(0, 0, 0, 0);
    //hardware
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
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
    private PathChain pickUp1, score1, pickUp2, score2, pushPickUp, pushScore, hang;
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
                .addPath(new BezierLine(new Point(pushScorePose), new Point(hangPose)))
                .setLinearHeadingInterpolation(pushScorePose.getHeading(), hangPose.getHeading())
                .build();
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

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        intakeRotateRight.setInverted(true);
        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
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
                follower.followPath(pickUp1, true);
                break;
            case 1:
                if (follower.getPose().getX() > (pickupPose1.getX() - 1) && follower.getPose().getY() > (pickupPose1.getY() - 1)) {
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
                }
                break;
            case 2:
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    //targetDistance = 5;
                    //vSlides.set(power);
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    claw.setPosition(0);
                    follower.followPath(pickUp2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() > (pickUpPose2.getX() - 1) && follower.getPose().getY() > (pickUpPose2.getY() - 1)) {
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
                }
                break;
            case 4:
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                   // targetDistance = 5;
                   // vSlides.set(power);
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    claw.setPosition(0);
                    follower.followPath(pushPickUp, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getPose().getX() > (pushPickUpPose.getX() - 1) && follower.getPose().getY() > (pushPickUpPose.getY() - 1)) {
                  //  targetDistance = 0;
                   // vSlides.set(power);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    follower.followPath(pushScore, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getX() > (pushScorePose.getX() - 1) && follower.getPose().getY() > (pushScorePose.getY() - 1)) {
                    follower.followPath(hang, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getPose().getX() > (hangPose.getX() - 1) && follower.getPose().getY() > (hangPose.getY() - 1)) {
                    clawRotateLeft.setPosition(1);
                    clawRotateRight.setPosition(1);
                    clawAdjust.setPosition(1);
                    setPathState(8);
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
//        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        //poseUpdater = new PoseUpdater(hardwareMap);

        //dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        // These loop the movements of the robot
        follower.update();
        //dashboardPoseTracker.update();
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
//        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();
        telemetry.update();

    }

    @Override
    public void init() {
//        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        //poseUpdater = new PoseUpdater(hardwareMap);

        //dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        //Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        //Drawing.sendPacket();

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
