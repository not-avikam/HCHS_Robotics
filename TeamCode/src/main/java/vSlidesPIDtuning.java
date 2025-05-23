/*
Encoder Type: Relative, Quadrature:
Relative: This means the encoder measures changes in position, not absolute position. You'll need to track the starting position and count the changes.
Quadrature: This refers to how the encoder signals are interpreted. Quadrature encoders have two output signals (A and B) that are 90 degrees out of phase. This allows you to determine the direction of rotation.
Encoder Sensor Type: Magnetic (Hall Effect):
This means the encoder uses a magnetic sensor (Hall Effect sensor) to detect changes in position.
Encoder Voltage Range: 3.3 - 5VDC:
This is the voltage range that the encoder can operate within. Make sure your power supply is within this range.
Encoder Resolution: 384.5 PPR at the Output Shaft:
PPR: Pulses Per Revolution. This is the same as CPR (Counts Per Revolution) for most practical purposes.
Output Shaft: This is the resolution after the gearbox.
384.5 PPR: This is a very specific number, which we'll explain below.
Encoder Resolution Formula: ((((1+(46/17))) * (1+(46/17))) * 28):
This formula is used to calculate the PPR. Let's break it down:
(1+(46/17)): This is the gear ratio of the first stage of the gearbox.
(1+(46/17)): This is the gear ratio of the second stage of the gearbox.
28: This is the CPR of the motor.
((((1+(46/17))) * (1+(46/17))) * 28): This is the formula to calculate the PPR.
Gear Ratio: The encoder resolution formula tells us that the motor has a gear ratio of (1+(46/17)) * (1+(46/17)).
Motor CPR: The motor has a CPR of 28.
Gearbox: The gearbox has a gear ratio of (1+(46/17)) * (1+(46/17)).
Output Shaft: The output shaft has a PPR of 384.5.
Encoder Resolution: The encoder resolution is 384.5 PPR at the output shaft.
Encoder Resolution Formula: The encoder resolution formula is ((((1+(46/17))) * (1+(46/17))) * 28).

Encoder Type Relative, Quadrature Encoder Sensor Type Magnetic (Hall Effect) Encoder Voltage Range 3.3 - 5VDC Encoder Resolution 384.5 PPR at the Output Shaft Encoder Resolution Formula ((((1+(46/17))) * (1+(46/17))) * 28)
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class vSlidesPIDtuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 383.6/360;
    DcMotorEx vSlideLeft;
    DcMotorEx vSlideRight;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        vSlideLeft = hardwareMap.get(DcMotorEx.class, "VSL");
        vSlideRight = hardwareMap.get(DcMotorEx.class, "VSR");

        vSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        vSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        vSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        vSlideRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int vSlideLeftPos = vSlideLeft.getCurrentPosition();
        int vSlideRightPos = vSlideRight.getCurrentPosition();
        double pidLeft = controller.calculate(vSlideLeftPos, target);
        double pidRight = controller.calculate(vSlideRightPos, target);
        double ff  = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double powerLeft = pidLeft + ff;
        double powerRight = pidRight + ff;

        vSlideLeft.setPower(powerLeft);
        vSlideRight.setPower(powerRight);

        telemetry.addData("Left pos ", vSlideLeftPos);
        telemetry.addData("Right pos ", vSlideRightPos);
        telemetry.addData("target", target);
        telemetry.addData("PID Left", pidLeft);
        telemetry.addData("PID Right", pidRight);
        telemetry.addData("Feedforward", ff);
        telemetry.addData("Power Left", powerLeft);
        telemetry.addData("Power Right", powerRight);
        telemetry.update();
        telemetry.update();
    }
}
