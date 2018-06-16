package net.kno3.season.relicrecovery.nerva.program.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import net.kno3.util.*;

/**
 * Created by robotics on 2/8/2018.
 */
@TeleOp(name = "Drive")
public class Drive extends OpMode {
    private DcMotor frontLeft, rearLeft, frontRight, rearRight;
    private AdafruitIMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("flm");
        frontRight = hardwareMap.dcMotor.get("frm");
        rearLeft = hardwareMap.dcMotor.get("rlm");
        rearRight = hardwareMap.dcMotor.get("rrm");

        imu = new AdafruitIMU(hardwareMap, "imu", true);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double w = gamepad1.right_stick_x;

        x *= 0.8;
        y *= 0.8;
        w *= 0.7;

        driveFieldOriented(x, y, w);

    }

    public void driveFieldOriented(double x, double y, double w) {
        Mecanum.fieldOriented(y, x, w, getHeading(), frontLeft, frontRight, rearLeft, rearRight);
    }

    public double getHeading() {
        return AngleUtil.normalize(imu.getHeading());
    }


}
