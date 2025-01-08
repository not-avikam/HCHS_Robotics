import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="intakeTest", group="opps")
public class IntakeTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");
        Servo intakeRotateLeft = hardwareMap.get(Servo.class, "iRL");
        Servo intakeRotateRight = hardwareMap.get(Servo.class, "iRR");

        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRotateLeft.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.x) {
                intakeRotateLeft.setPosition(.15);
                intakeRotateRight.setPosition(.15);
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.a) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
                intakeRotateLeft.setPosition(.1);
                intakeRotateRight.setPosition(.1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeRotateLeft.setPosition(0);
                intakeRotateRight.setPosition(0);
                telemetry.addLine("In position for claw pickup");
            }

            telemetry.update();
        }
    }}
