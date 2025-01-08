
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "Autopilot", group = "Autonomous")
public class autoDrive extends OpMode{
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(8.983, 100.139, Math.toRadians(0));
    private final Pose preloadScoreControlPose = new Pose(31.706, 100.404, Math.toRadians(0));
    private final Pose scorePose = new Pose(37, 72, Math.toRadians(180));
    private final Pose pickupPose = new Pose(10, 11, Math.toRadians(0));
    private final Pose pickup1ControlPose1 = new Pose(16.382, 67.640, Math.toRadians(0));
    private final Pose pickup1ControlPose2 = new Pose(85.343, 10.305, Math.toRadians(0));
    private final Pose pickup2ControlPose1 = new Pose(7.662, 32.235, Math.toRadians(0));
    private final Pose pickup2ControlPose2 = new Pose(70.811, 8.983, Math.toRadians(0));
    private final Pose pickup2ControlPose3 = new Pose(55.222, 11.097, Math.toRadians(0));
    private final Pose pickupWait = new Pose(25.000, 15.000, Math.toRadians(0));
    private final Pose parkPose = new Pose(8.983, 19.288, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12.154, 79.530, Math.toRadians(90));
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, score1, score2, scorePickup3, pickup, pickupBackUp;

    public void buildpaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(preloadScoreControlPose), new Point(scorePose)));
        scorePreload.setTangentHeadingInterpolation();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickupPose.getHeading())
                .build();
        pickupBackUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(pickupWait)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), pickupWait.getHeading())
                .build();
        pickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupWait), new Point(pickupPose)))
                .setLinearHeadingInterpolation(pickupWait.getHeading(), pickupPose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine (new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup2ControlPose1), new Point(pickup2ControlPose2), new Point(pickup2ControlPose3), new Point(pickupPose)))
                .setTangentHeadingInterpolation()
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine (new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {

        follower = new Follower(hardwareMap);
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                } break;

            case 4:
                if(follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    setPathState(5);
                } break;

            case 5:
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park,true);
                    setPathState(6);
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
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
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
