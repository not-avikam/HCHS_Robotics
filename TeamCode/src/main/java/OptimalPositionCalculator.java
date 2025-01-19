import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Servo Position calculator", group="Silver Knight")
public class OptimalPositionCalculator extends LinearOpMode {
    @Override
    public void runOpMode() {
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");
        ServoImplEx clawAdjust = hardwareMap.get(ServoImplEx.class, "cA");
        ServoImplEx clawRotateLeft = hardwareMap.get(ServoImplEx.class, "cRL");
        ServoImplEx clawRotateRight = hardwareMap.get(ServoImplEx.class, "cLL");
        ServoImplEx intakeRotateLeft = hardwareMap.get(ServoImplEx.class, "iRL");
        ServoImplEx intakeRotateRight = hardwareMap.get(ServoImplEx.class, "iRR");
        CRServo intakeLeft = hardwareMap.get(CRServo.class, "iL");
        CRServo intakeRight = hardwareMap.get(CRServo.class, "iR");

        intakeRotateRight.setDirection(ServoImplEx.Direction.REVERSE);
        clawRotateRight.setDirection(ServoImplEx.Direction.REVERSE);;
        claw.setDirection(ServoImplEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                clawRotateLeft.setPosition(clawRotateLeft.getPosition()+.01);
                clawRotateRight.setPosition(clawRotateRight.getPosition()+.01);
                telemetry.update();
                telemetry.addData("clawRotateLeft", clawRotateLeft.getPosition());
                telemetry.addData("clawRotateRight", clawRotateRight.getPosition());
                telemetry.update();
            } else if (gamepad2.dpad_down) {
                clawRotateLeft.setPosition(clawRotateLeft.getPosition()-.01);
                clawRotateRight.setPosition(clawRotateRight.getPosition()-.01);
                telemetry.update();
                telemetry.addData("clawRotateLeft", clawRotateLeft.getPosition());
                telemetry.addData("clawRotateRight", clawRotateRight.getPosition());
                telemetry.update();
            }

            if (gamepad2.dpad_left) {
                clawAdjust.setPosition(clawAdjust.getPosition()-.01);
                telemetry.update();
                telemetry.addData("clawAdjust", clawAdjust.getPosition());
                telemetry.update();
            } else if (gamepad2.dpad_right) {
                clawAdjust.setPosition(clawAdjust.getPosition()+.01);
                telemetry.update();
                telemetry.addData("clawAdjust", clawAdjust.getPosition());
                telemetry.update();
            }

            if (gamepad1.dpad_up) {
                intakeRotateLeft.setPosition(intakeRotateLeft.getPosition()+.01);
                intakeRotateRight.setPosition(intakeRotateRight.getPosition()+.01);
                telemetry.update();
                telemetry.addData("intakeRotateLeft", intakeRotateLeft.getPosition());
                telemetry.addData("intakeRotateRight", intakeRotateRight.getPosition());
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                intakeRotateLeft.setPosition(intakeRotateLeft.getPosition()-.01);
                intakeRotateRight.setPosition(intakeRotateRight.getPosition()-.01);
                telemetry.update();
                telemetry.addData("intakeRotateLeft", intakeRotateLeft.getPosition());
                telemetry.addData("intakeRotateRight", intakeRotateRight.getPosition());
                telemetry.update();
            }

            if (gamepad2.y) {
                claw.setPosition(1);
            } else if (gamepad2.b) {
                claw.setPosition(0);
            }

            if (gamepad1.x) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.a) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            telemetry.update();
        }
    }
}