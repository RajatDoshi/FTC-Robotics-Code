package net.kno3.season.relicrecovery.nerva.program.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;

/**
 * Created by robotics on 10/28/2017.
 */

@TeleOp(group = "testing", name = "runlifttest")
public class RunLiftTest extends OpMode {
    private DcMotor motor;
    private Servo holder;
    private double hc, ho;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get(Nerva.GLYPH_LIFT_KEY);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        holder = hardwareMap.servo.get(Nerva.GLYPH_LIFT_HOLDER_KEY);

        RobotSettings settings = new RobotSettings("Nerva");

        hc = settings.getDouble("glyph_lift_holder_closed");
        ho = settings.getDouble("glyph_lift_holder_open");
    }

    @Override
    public void loop() {
        double mpow = -gamepad1.left_stick_y;
        if(mpow < 0) {
            mpow *= 0.25;
        }
        motor.setPower(mpow);
        holder.setPosition(gamepad1.a ? hc : ho);
        telemetry.addData("Pos", -motor.getCurrentPosition());
    }
}
