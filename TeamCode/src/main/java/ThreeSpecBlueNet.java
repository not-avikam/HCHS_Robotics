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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "3+0 Blue Net", group = "Silver Knight")
public class ThreeSpecBlueNet extends OpMode{
    //private PoseUpdater poseUpdater;
    //private DashboardPoseTracker dashboardPoseTracker;
    PIDFController pidf = new PIDFController(0, 0, 0, 0);
    //hardware
    //auto stuff
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(10, 104, Math.toRadians(0));  // Starting position
    private final Pose scorePose1 = new Pose(38, 68, Math.toRadians(180));
    private final Pose pickupPose1 = new Pose(67, 22, Math.toRadians(90));
    private final Pose pickupPose1ControlPose1 = new Pose(3, 38);
    private final Pose pickupPose1ControlPose2 = new Pose(105, 5);
    private final Pose pickupPose1ControlPose3 = new Pose(38, 47);
    private final Pose dropOff1 = new Pose(19, 22, Math.toRadians(90));
    private final Pose pickupSpecimenPose = new Pose(9, 28, Math.toRadians(180)); // First sample pickup
    private final Pose scorePose2 = new Pose(38, 64, Math.toRadians(180)); // Third sample pickup
    private final Pose scorePose3 = new Pose(38, 72, Math.toRadians(180));
    private final Pose parkPose = new Pose(18, 24, Math.toRadians(0));
    private PathChain park, score2, score3, scorePreload, pickup1, dropOffSample1, pickupSpecimen1, pickupSpecimen2;
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

        pickupSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOff1), new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(dropOff1.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose2.getHeading())
                .build();

        pickupSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose2),new Point(pickupSpecimenPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickupSpecimenPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupSpecimenPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickupSpecimenPose.getHeading(), scorePose3.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading())
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
        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);

        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);

        //TODO: Adjust the vSlides parameters
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
                intakeRotateRight.setPosition(0);
                intakeRotateLeft.setPosition(0);
                follower.followPath(scorePreload, true);
                break;
            case 1:
                if (!follower.isBusy()) {
                    targetDistance = 2.16667-1.19500;
                    clawRotateRight.setPosition(.833);
                    clawRotateLeft.setPosition(.833);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    linSlideLeft.setPower(-.4);
                    linSlideRight.setPower(-.4);
                    clawAdjust.setPosition(.75);
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    targetDistance = 0;
                    clawRotateRight.setPosition(0);
                    clawRotateLeft.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    linSlideLeft.setPower(-.4);
                    linSlideRight.setPower(-.4);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    follower.followPath(dropOffSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    linSlideLeft.setPower(-.4);
                    linSlideRight.setPower(-.4);
                    follower.followPath(pickupSpecimen1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    clawAdjust.setPosition(0.5);
                    targetDistance = 0;
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        claw.setPosition(0);
                        follower.followPath(score2,true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //TODO: subtract distance of retracted slides
                    targetDistance = 2.16667-1.19500;
                    clawRotateRight.setPosition(.833);
                    clawRotateLeft.setPosition(.833);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    clawAdjust.setPosition(.75);
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    follower.followPath(pickupSpecimen2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    clawRotateLeft.setPosition(.09);
                    clawRotateRight.setPosition(.09);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    clawAdjust.setPosition(0.5);
                    targetDistance = 0;
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        claw.setPosition(0);
                        follower.followPath(score3,true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 25) {
                    targetDistance = 2.16667-1.19500;
                    clawRotateRight.setPosition(.833);
                    clawRotateLeft.setPosition(.833);
                    intakeRotateRight.setPosition(0);
                    intakeRotateLeft.setPosition(0);
                    clawAdjust.setPosition(.75);
                    actionTimer.resetTimer();
                    if (vSlideLeft.atTargetPosition() && vSlideRight.atTargetPosition()) {
                        claw.setPosition(1);
                    }
                    follower.followPath(park, true);
                    //Sets to a non existent pathstate so that it doesn't keep running
                    //-Avikam ;)
                    setPathState(8);
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
    public void stop() {}
}
