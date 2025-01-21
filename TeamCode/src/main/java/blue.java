    import static java.lang.Math.abs;

    import android.util.Size;

    import com.arcrobotics.ftclib.controller.PIDFController;
    import com.arcrobotics.ftclib.drivebase.MecanumDrive;
    import com.arcrobotics.ftclib.gamepad.ButtonReader;
    import com.arcrobotics.ftclib.gamepad.GamepadEx;
    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
    import com.arcrobotics.ftclib.gamepad.TriggerReader;
    import com.arcrobotics.ftclib.hardware.ServoEx;
    import com.arcrobotics.ftclib.hardware.SimpleServo;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.arcrobotics.ftclib.hardware.motors.MotorEx;
    import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
    import com.qualcomm.ftccommon.SoundPlayer;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.ServoImplEx;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.opencv.ImageRegion;
    import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.localization.Pose;
    import com.pedropathing.util.Constants;

    import pedroPathing.constants.FConstants;
    import pedroPathing.constants.LConstants;

    import java.io.File;

    @TeleOp(name="blue-experimental", group="Silver Knight")
    public class blue extends LinearOpMode {
        PIDFController pidf = new PIDFController(1, 2, 3, 4);
        private final ElapsedTime runtime = new ElapsedTime();
        private final String soundPath = "/FIRST/blocks/sounds";
        private final File Alert  = new File( soundPath + "/alert.wav");

        //TODO: Make this correct
        private final Pose startPose = new Pose(64, 80, Math.toRadians(0));
        private final Pose observationZone = new Pose(0, 0);
        private final Pose basket = new Pose(0, 144);
        private Follower follower;

        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap);
            follower.startTeleopDrive();
            follower.setStartingPose(startPose);
            MecanumDrive drive = new MecanumDrive(
                    new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312),
                    new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312),
                    new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312),
                    new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312)
            );
            MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
            MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
            MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
            CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
            CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
            CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
            CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
            ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");
            ServoImplEx clawAdjust = hardwareMap.get(ServoImplEx.class, "cA");
            ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 300, AngleUnit.DEGREES);
            ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 300, AngleUnit.DEGREES);
            ServoImplEx intakeRotateLeft = hardwareMap.get(ServoImplEx.class, "iRL");
            ServoImplEx intakeRotateRight = hardwareMap.get(ServoImplEx.class, "iRR");
            //BNO055IMUNew imu = hardwareMap.get(BNO055IMUNew.class, "imu");
            GamepadEx driverOp = new GamepadEx(gamepad1);
            GamepadEx clawOp = new GamepadEx(gamepad2);


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

            boolean AlertFound   = Alert.exists();
            telemetry.addData("Alert sound",   AlertFound ?   "Found" : "NOT Found \nCopy alert.wav to " + soundPath  );

            ToggleButtonReader robotCentricReader = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON
            );

            ToggleButtonReader hangModeRight = new ToggleButtonReader(
                    clawOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
            );

            ToggleButtonReader hangModeLeft = new ToggleButtonReader(
                    clawOp, GamepadKeys.Button.LEFT_STICK_BUTTON
            );

            ToggleButtonReader slowMode = new ToggleButtonReader(
                    driverOp, GamepadKeys.Button.LEFT_BUMPER
            );

            ButtonReader intakeOnReader = new ButtonReader(
                    driverOp, GamepadKeys.Button.X
            );

            TriggerReader vSlideUpReader = new TriggerReader(
                    clawOp, GamepadKeys.Trigger.RIGHT_TRIGGER
            );

            TriggerReader vSlideDownReader = new TriggerReader(
                    clawOp, GamepadKeys.Trigger.LEFT_TRIGGER
            );

            //TODO: Adjust the vSlides parameters
            vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            vSlides.setRunMode(Motor.RunMode.PositionControl);
            vSlides.setPositionCoefficient(0.05);
            vSlides.setDistancePerPulse(0.015);
            vSlides.stopAndResetEncoder();

            vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            vSlideRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

            intakeRotateRight.setDirection(ServoImplEx.Direction.REVERSE);
            linSlideLeft.setDirection(CRServo.Direction.REVERSE);
            intakeLeft.setDirection(CRServo.Direction.REVERSE);
            clawRotateLeft.setInverted(false);
            clawRotateRight.setInverted(true);;
            claw.setDirection(ServoImplEx.Direction.REVERSE);
            vSlideRight.setInverted(true);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            follower.startTeleopDrive();
            follower.setStartingPose(startPose);

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                if (robotCentricReader.wasJustPressed()) {
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                    follower.update();
                } else if (slowMode.wasJustPressed()) {
                    follower.setTeleOpMovementVectors((-gamepad1.left_stick_y * .5), (-gamepad1.left_stick_x * .5), (-gamepad1.right_stick_x * .5), false);
                    follower.update();
                } else if (robotCentricReader.wasJustPressed() && (slowMode.wasJustPressed())) {
                    follower.setTeleOpMovementVectors((-gamepad1.left_stick_y * .5), (-gamepad1.left_stick_x * .5), (-gamepad1.right_stick_x * .5), true);
                    follower.update();
                } else {
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                    follower.update();
                }

                telemetry.addLine("To use field centric mode, press left stick.");

                follower.update();

                if (gamepad1.right_trigger != 0) {
                    linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                } else if (gamepad1.left_trigger != 0) {
                    linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                } else {
                    linSlideLeft.setPower(-1 * ((Math.abs(Math.pow(follower.getVelocityMagnitude(), 2) * .2 / .55)/1.79) * 100));
                    linSlideRight.setPower(-1 * ((Math.abs(Math.pow(follower.getVelocityMagnitude(), 2) * .2 / .55)/1.79) * 100));
                }

                double targetDistance = 0;
                //TODO
                //double power = pidf.calculate(vSlides.getCurrentPosition());  // Adjust power based on how fast you want to move

                if (gamepad2.dpad_up) {
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    telemetry.addLine("Sample scoring");
                    //TODO
//                    targetDistance = 5;
//                    vSlides.set(power);
                    telemetry.addLine("Adjusting viper slides automatically");
                } else if (gamepad2.dpad_down) {
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    telemetry.addLine("Resetting claw to intake");
                } else if (gamepad2.dpad_left) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    telemetry.addLine("Specimen pickup");
                }

                if (gamepad2.right_trigger != 0) {
                    targetDistance = 5;
                    vSlides.set(gamepad2.right_trigger - gamepad2.left_trigger);
                } else if (gamepad2.left_trigger != 0) {
                    targetDistance = -5;
                    vSlides.set(gamepad2.right_trigger - gamepad2.left_trigger);
                } else if (vSlideUpReader.wasJustReleased()) {
                    vSlides.set(0);
                } else if (vSlideDownReader.wasJustReleased()) {
                    vSlides.set(0);
                }

                if (gamepad2.y) {
                    claw.setPosition(1);
                    telemetry.addLine("Claw opened all the way | MANUAL OPERATION OF CLAW");
                } else if (gamepad2.b) {
                    claw.setPosition(0);
                    telemetry.addLine("Claw closed | MANUAL OPERATION OF CLAW");
                }

                PredominantColorProcessor.Result result = colorSensor.getAnalysis();


                if (gamepad1.x) {
                    intakeRotateLeft.setPosition(.025);
                    intakeRotateRight.setPosition(.17);
                    //intakeLeft.setPower(1);
                    //intakeRight.setPower(1);
                    telemetry.addLine("Intake in position for sample pickup");
                    clawRotateLeft.setPosition(0);
                    clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    telemetry.addLine("Adjusting claw automatically");
                } else if (gamepad1.a) {
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                    intakeRotateLeft.setPosition(.1);
                    intakeRotateRight.setPosition(.1);
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    telemetry.addLine("Intake on");
                    //clawRotateLeft.setPosition(0);
                    //clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.12 - .0277);
                    claw.setPosition(1);
                    telemetry.addLine("Adjusting claw automatically");
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    telemetry.addLine("Intake on");
                    //clawRotateLeft.setPosition(0);
                    //clawRotateRight.setPosition(0);
                    clawAdjust.setPosition(.25);
                    claw.setPosition(1);
                    telemetry.addLine("Adjusting claw automatically");
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {
                    telemetry.addLine("WRONG COLOR!");
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                }  else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    intakeRotateLeft.setPosition(0);
                    intakeRotateRight.setPosition(0);
                    telemetry.addLine("In position for claw pickup");
                }

                if (follower.getPose().getX() < (observationZone.getX() + 25) && follower.getPose().getY() < (observationZone.getY() + 31)) {
                    clawRotateLeft.setPosition(.11);
                    clawRotateRight.setPosition(.11);
                    clawAdjust.setPosition(0.5);
                    targetDistance = 1;
                    telemetry.addLine("Specimen pickup");
                }

                if (follower.getPose().getX() < (basket.getX() + 24) && follower.getPose().getY() > (basket.getY()) - 24) {
                    clawRotateLeft.setPosition(.833);
                    clawRotateRight.setPosition(.833);
                    clawAdjust.setPosition(.75);
                    telemetry.addLine("Sample scoring");
                    targetDistance = 5;
                    telemetry.addLine("Adjusting viper slides automatically");
                }

                if (hangModeRight.wasJustReleased() && hangModeLeft.wasJustReleased()) {
                    vSlides.setRunMode(Motor.RunMode.RawPower);
                    vSlides.set(clawOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - clawOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                    linSlideLeft.setPower(-.053);
                    linSlideRight.setPower(-.053);
                    claw.setPwmDisable();
                    //TODO
                    //clawRotateLeft.setPwmDisable();
                    //clawRotateRight.setPwmDisable();
                    clawAdjust.setPwmDisable();
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    intakeRotateLeft.setPwmDisable();
                    intakeRotateRight.setPwmDisable();
                    telemetry.addLine("Hang mode");
                }

                //TODO
                //double currentDistance = vSlides.getCurrentPosition() /* *number for .setDistancePerPulse*/;
                //double distanceRemaining = targetDistance - currentDistance;
                pidf.setSetPoint(targetDistance);

                //telemetry.addData("Vslides position", "%.2f", vSlides.getCurrentPosition());
                //telemetry.addData("Vslides distance", "%.2f", vSlides.getDistance());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();
                follower.update();
            }
            }
        }