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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Blue Autonomous Observation - Silver Knight", group = "Silver Knight")
public class SilverKnightBlueAutoObservation extends OpMode{
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
    private final Pose startPose = new Pose(10, 87, Math.toRadians(0));  // Starting position
    private final Pose scorePose1 = new Pose(38, 70, Math.toRadians(90));
    private final Pose pickupPose1 = new Pose(57, 25, Math.toRadians(315));
    private final Pose pickupPose1ControlPose1 = new Pose(0.5, 77, Math.toRadians(90));
    private final Pose pickupPose1ControlPose2 = new Pose(82, .8, Math.toRadians(90));
    private final Pose pickupPose1ControlPose3 = new Pose(57, 39, Math.toRadians(90));
    private final Pose dropOff1 = new Pose(13, 27, Math.toRadians(90));
    private final Pose pickUpPose2 = new Pose(51, 17, Math.toRadians(90));
    private final Pose pickUpPose2ControlPose1 = new Pose(90, 27, Math.toRadians(90));
    private final Pose dropOff2 = new Pose(15, 19, Math.toRadians(90));
    private final Pose dropOff3 = new Pose(16, 10.6, Math.toRadians(90));
    private final Pose dropOff3ControlPose1 = new Pose(140, 7, Math.toRadians(90));

    private final Pose pickupSpecimenPose = new Pose(10, 28, Math.toRadians(180)); // First sample pickup
    private final Pose scorePose2 = new Pose(38, 76, Math.toRadians(0)); // Third sample pickup
    private final Pose scorePose3 = new Pose(38, 74, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(38, 72, Math.toRadians(0));
    private final Pose scorePose5 = new Pose(38, 68, Math.toRadians(0));
    private final Pose parkPose = new Pose(18, 24, Math.toRadians(0));
    private PathChain park, score2return, score3return, score4return, score5return, score2, score3, score4, score5, pickupSpecimen, scorePreload, pickup1, pickup2, pickUpSample3, dropOffSample1, dropOffSample2, pickUpSample2;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickupPose1ControlPose1), new Point(pickupPose1ControlPose2), new Point(pickupPose1ControlPose3), new Point(pickupPose1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickupPose1.getHeading())
                .build();

        dropOffSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose1), new Point(dropOff1)))
                .setConstantHeadingInterpolation(dropOff1.getHeading())
                .build();

        pickUpSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropOff1), new Point(pickUpPose2ControlPose1), new Point(pickUpPose2)))
                .setConstantHeadingInterpolation(pickUpPose2.getHeading())
                .build();

        dropOffSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(dropOff2)))
                .setConstantHeadingInterpolation(dropOff2.getHeading())
                .build();

        pickUpSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropOff2),new Point(dropOff3ControlPose1), new Point(dropOff3)))
                .setConstantHeadingInterpolation(dropOff3.getHeading())
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
                setPathState(1);
                follower.followPath(scorePreload, true);
                break;
            case 1:
                if (follower.getPose().getX() > (scorePose1.getX() - 1) && follower.getPose().getY() > (scorePose1.getY() - 1)) {
                    vSlides.setTargetPosition(5);
                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(pickup1, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (pickupPose1.getX() - 1) && follower.getPose().getY() > (pickupPose1.getY() - 1)) {
                    vSlides.setTargetPosition(0);
                    vSlides.set(-1);
                    follower.followPath(dropOffSample1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() > (dropOff1.getX() - 1) && follower.getPose().getY() > (dropOff1.getY() - 1)) {
                    follower.followPath(pickUpSample2, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() > (pickUpPose2.getX() - 1) && follower.getPose().getY() > (pickUpPose2.getY() - 1)) {
                    follower.followPath(dropOffSample2, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getPose().getX() > (dropOff2.getX() - 1) && follower.getPose().getY() > (dropOff2.getY() - 1)) {
                    follower.followPath(pickUpSample3, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getX() > (dropOff3.getX() - 1) && follower.getPose().getY() > (dropOff3.getY() - 1)) {
                    follower.followPath(pickupSpecimen, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.getPose().getX() > (pickupSpecimenPose.getX() - 1) && follower.getPose().getY() > (pickupSpecimenPose.getY() - 1)) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    vSlides.setTargetPosition(1);
                    vSlides.set(-1);
                    follower.followPath(score2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getPose().getX() > (scorePose2.getX() - 1) && follower.getPose().getY() > (scorePose2.getY() - 1)) {
                    vSlides.setTargetPosition(5);
                    vSlides.set(1);
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
                if (follower.getPose().getX() > (pickupSpecimenPose.getX() - 1) && follower.getPose().getY() > (pickupSpecimenPose.getY() - 1)) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    vSlides.setTargetPosition(1);
                    vSlides.set(-1);
                    follower.followPath(score3,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getPose().getX() > (scorePose3.getX() - 1) && follower.getPose().getY() > (scorePose3.getY() - 1)) {
                    vSlides.setTargetPosition(5);
                    vSlides.set(1);
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
                if (follower.getPose().getX() > (pickupSpecimenPose.getX() - 1) && follower.getPose().getY() > (pickupSpecimenPose.getY() - 1)) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    vSlides.setTargetPosition(1);
                    vSlides.set(-1);
                    follower.followPath(score4,true);
                    setPathState(12);
                }
                break;
            case 12:
                if (follower.getPose().getX() > (scorePose4.getX() - 1) && follower.getPose().getY() > (scorePose4.getY() - 1)) {
                    vSlides.setTargetPosition(5);
                    vSlides.set(1);
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
                if (follower.getPose().getX() > (pickupSpecimenPose.getX() - 1) && follower.getPose().getY() > (pickupSpecimenPose.getY() - 1)) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    vSlides.setTargetPosition(1);
                    vSlides.set(-1);
                    follower.followPath(score5,true);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.getPose().getX() > (scorePose5.getX() - 1) && follower.getPose().getY() > (scorePose5.getY() - 1)) {
                    vSlides.setTargetPosition(5);
                    vSlides.set(1);
                    clawRotateRight.setPosition(.65);
                    clawRotateLeft.setPosition(.65);
                    clawAdjust.setPosition(1);
                    //TODO: Measure the correct amount of time for this
                    /*
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        claw.setPosition(0);
                    }
                     */
                    follower.followPath(park, false);
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
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        ServoEx intakeRotateLeft = new SimpleServo(hardwareMap, "iRL", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeRotateRight = new SimpleServo(hardwareMap, "iRR", 0, 300, AngleUnit.DEGREES);

        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRotateLeft.setInverted(true);
        vSlides.setInverted(true);
        vSlideRight.setInverted(false);

        intakeRotateLeft.setPosition(0);
        intakeRotateRight.setPosition(0);

// TODO: This code is incorrect
//  it only calculates the newtons of force necessary
//  we need to convert the newtons to whatever is correct for the GoBilda 300 degree speed servos
//            if (gamepad1.right_trigger != 0) {
//                linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//                linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//            } else if (gamepad1.left_trigger != 0) {
//                linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//                linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//            } else {
//                linSlideLeft.setPower(-1*(abs((follower.getVelocityMagnitude()*follower.getVelocityMagnitude() /* * mass of intake*/ )/3)));
//                linSlideRight.setPower(-1*(abs((follower.getVelocityMagnitude()*follower.getVelocityMagnitude() /* * mass of intake*/ )/3)));
//            }

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
    public void init_loop() {
        ServoEx intakeRotateLeft = new SimpleServo(hardwareMap, "iRL", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeRotateRight = new SimpleServo(hardwareMap, "iRR", 0, 300, AngleUnit.DEGREES);
        intakeRotateLeft.setPosition(0);
        intakeRotateRight.setPosition(0);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
