package net.kno3.util;

/**
 * Created by robotics on 10/9/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * Methods for the Mecanum multi-directional drive train
 * <p/>
 * WARNING: This class was ported from RobotC, but not retested. Use with caution.
 */
public final class Mecanum {
    /**
     * Implements the Arcade drive train with three axis and four motors.
     *
     * @param y          The y-axis of the controller, forward/rev
     * @param x          The x-axis of the controller, strafe
     * @param w          The spin axis of the controller
     * @param leftFront  The motor on the front left
     * @param rightFront The motor on the front right
     * @param leftBack   The motor on the back left
     * @param rightBack  The motor on the back right
     */
    public static void robotOriented(double y, double x, double w, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + w;
        double rightFrontVal = y - x - w;
        double leftBackVal = y - x + w;
        double rightBackVal = y + x - w;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(coerce(-1, 1, leftFrontVal));
        rightFront.setPower(coerce(-1, 1, rightFrontVal));
        leftBack.setPower(coerce(-1, 1, leftBackVal));
        rightBack.setPower(coerce(-1, 1, rightBackVal));
    }

    /**
     * Implements the Arcade drive train with field orientation based on Gyro input
     *
     * @param y           The y-axis of the controller, forward/rev
    .* @param w           The spin axis of the controller
     * @param gyroheading The current normalized gyro heading (between 0 and 360)
     * @param leftFront   The motor on the front left
     * @param rightFront  The motor on the front right
     * @param leftBack    The motor on the back left
     * @param rightBack   The motor on the back right
     */
    public static void fieldOriented(double y, double x, double w, double gyroheading, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double cosA = Math.cos(Math.toRadians(normalize(gyroheading)));
        double sinA = Math.sin(Math.toRadians(normalize(gyroheading)));
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        robotOriented(yOut, xOut, w, leftFront, rightFront, leftBack, rightBack);
    }

    /**
     * Forces a numerical value to be between a min
     * and a max.
     *
     * @param min   If less than min, returns min
     * @param max   If greater than max, returns max
     * @param value Value to test
     * @return Coerced value
     */
    private static double coerce(double min, double max, double value) {
        return (value > max) ? max : (value < min) ? min : value;
    }

    /**
     * Normalize Gyroscope bounds to within 0 and 360
     *
     * @param heading The current Gyroscope value
     * @return The normalized Gyroscope value, between 0 and 360.
     */
    private static double normalize(double heading) {
        if (heading < 0) {
            return 360 - (Math.abs(heading) % 360);
        } else {
            return (heading % 360);
        }
    }
}