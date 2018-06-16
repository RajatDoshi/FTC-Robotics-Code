package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.google.common.base.Supplier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.Robot;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;
import net.kno3.season.relicrecovery.nerva.robot.NervaDriveSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaPersist;
import net.kno3.util.AdafruitIMU;
import net.kno3.util.AngleUtil;
import net.kno3.util.Mecanum;
import net.kno3.util.SynchronousPID;
import net.kno3.util.Threading;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by robotics on 10/29/2017.
 */

public class TestingNervaDT {
    public double kDriveFor_p;
    public double kDriveForX_p;
    public double kDriveForFO_p;
    public double kTurnPIDslow_maxTurn, kTurnPIDslow_kp, kTurnPIDslow_ki, kTurnPIDslow_kd, kTurnPIDslow_tolerance;
    public double kTurnPIDfast_maxTurn, kTurnPIDfast_kp, kTurnPIDfast_ki, kTurnPIDfast_kd, kTurnPIDfast_tolerance;
    public double kTurnPIDsuperfast_maxTurn, kTurnPIDsuperfast_kp, kTurnPIDsuperfast_ki, kTurnPIDsuperfast_kd, kTurnPIDsuperfast_tolerance;
    public double kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_maxTurn,
            kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_tolerance;
    public double kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_maxTurn,
            kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_tolerance;
    public double kDriveWithCorrectionsSuperfast_p, kDriveWithCorrectionsSuperfast_maxTurn,
            kDriveWithCorrectionsSuperfast_kp, kDriveWithCorrectionsSuperfast_ki, kDriveWithCorrectionsSuperfast_kd, kDriveWithCorrectionsSuperfast_tolerance;
    private double xSpeed = 0.7; // max speed of driveForX
    private double ySpeed = 0.5; // max speed of driveForY
    private double xAcc = 0.5; // accuracy of driveFor in inches
    private double yAcc = 0.5;

    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    private AdafruitIMU imu;
    private double imuZeroHeading;
    private DcMotor encoder_y, encoder_x;
    private int encZeroX, encZeroY;

    private boolean slowLockout = false, foLockout = false, slow = false, fieldOriented = true;

    Telemetry telemetry;

    public TestingNervaDT(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.dcMotor.get(Nerva.DRIVE_FL_KEY);
        frontRight = hardwareMap.dcMotor.get(Nerva.DRIVE_FR_KEY);
        rearLeft = hardwareMap.dcMotor.get(Nerva.DRIVE_RL_KEY);
        rearRight = hardwareMap.dcMotor.get(Nerva.DRIVE_RR_KEY);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        modeVoltage();
        brakeMode();

        Threading.startThread(() -> {
            imu = new AdafruitIMU(hardwareMap, Nerva.IMU_KEY, !NervaPersist.lastWasAuto);
            telemetry.addData("IMU initialized", true);
            telemetry.addData("heading", () -> imu.getHeading());
            telemetry.update();
        });
        this.encoder_x = hardwareMap.dcMotor.get(Nerva.ENCODER_X_KEY);
        this.encoder_y = hardwareMap.dcMotor.get(Nerva.ENCODER_Y_KEY);
    }


    public void floatMode() {
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakeMode() {
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void modeSpeed() {
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void modeVoltage() {
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getHeading() {
        return imu.getHeading() - imuZeroHeading;
    }

    public void zeroHeading() {
        this.imuZeroHeading = imu.getHeading();
    }

    public int getEncX() {
        return -(encoder_x.getCurrentPosition() - this.encZeroX);
    }

    public int getEncY() {
        return -(encoder_y.getCurrentPosition() - this.encZeroY);
    }

    public void resetEncX() {
        this.encZeroX = encoder_x.getCurrentPosition();
    }

    public void resetEncY() {
        this.encZeroY = encoder_y.getCurrentPosition();
    }


    public void driveRobotOriented(double x, double y, double w) {
        Mecanum.robotOriented(y, x, w, frontLeft, frontRight, rearLeft, rearRight);
    }

    public void driveFieldOriented(double x, double y, double w) {
        Mecanum.fieldOriented(y, x, w, getHeading(), frontLeft, frontRight, rearLeft, rearRight);
    }


    public boolean driveFor(double maxSpeed, double p, double inches, double accuracy, boolean isX, double timeOut, Supplier<Boolean> stopCase) {
        resetEncX();
        resetEncY();

        int targetPulses = inchesToEncoderPulses(inches);
        int accuracyPulses = inchesToEncoderPulses(accuracy);
        SynchronousPID pid = new SynchronousPID(p, 0, 0);
        pid.setOutputRange(-maxSpeed, maxSpeed);
        pid.setSetpoint(targetPulses);

        /*
         * Last system time at which the PID loop was definitely NOT stable at the solution.
         */
        long lastBad = System.currentTimeMillis();
        long startTime = System.currentTimeMillis();


        /*
         * Loop until either:
         *   (1) Autonomous is over.
         *   (2) The PID loop becomes stable at the solution for at least 25 ms.
         */
        while(true) {

            try { Thread.sleep(5); }
            catch (InterruptedException ex) { }

            /*
             * Stop if supplier returns true
             */
            if(stopCase != null && stopCase.get()) {
                return true;
            }


            /*
             * Check if autonomous is over.
             */
            if(!Threading.isOpModeActive()) {
                return false;
            }

            /*
             * Check if loop is timed out
             */
            if((System.currentTimeMillis() - startTime) > (long)(timeOut*1000)) {
                return true;
            }

            /*
             * Update the PID.
             */
            int encoderPos;
            if(isX) { encoderPos = getEncX(); }
            else { encoderPos = getEncY(); }
            double speed = pid.calculate(encoderPos);


            /*
             * Ensure that we never try to send the motor a speed so low it won't do anything.
             */
            if(isX) {
                if ((speed > 0) && (speed < .3)) speed = 0.3;
                else if((speed < 0) && (speed > -0.3)) speed = -0.3;
            }
            else {
                if ((speed > 0) && (speed < .2)) speed = 0.2;
                else if ((speed < 0) && (speed > -0.2)) speed = -0.2;
            }

            /*
             * If the PID loop is NOT stable at the solution (error is too large) then update
             * the time.
             */
            if(Math.abs(encoderPos - targetPulses) > accuracyPulses) lastBad = System.currentTimeMillis();


            /*
             * If the PID loop IS stable for the past 25 ms then we're done.
             */
            if (System.currentTimeMillis() - lastBad > 25) return true;


            /*
             * Looks like we're not stable yet. Update the motors.
             */
            if(isX) driveRobotOriented(speed, 0, 0);
            else driveRobotOriented(0, speed, 0);
        }
    }

    public boolean driveForX(double inches) {
        return driveFor(xSpeed, kDriveForX_p, inches, xAcc, true, 30, null);
    }

    public boolean driveForY(double inches) {
        return driveFor(ySpeed, kDriveFor_p, inches, yAcc, false, 30, null);
    }

    public boolean driveForXTime(double inches, double timeOut) {
        return driveFor(xSpeed, kDriveForX_p, inches, xAcc, true, timeOut, null);
    }
    public boolean driveForYTime(double inches, double timeOut) {
        return driveFor(ySpeed, kDriveFor_p, inches, yAcc, false, timeOut, null);
    }

    public boolean driveForX(double inches, Supplier<Boolean> stopCase) {
        return driveFor(xSpeed, kDriveForX_p, inches, xAcc, true, 30, stopCase);
    }

    public boolean driveForY(double inches, Supplier<Boolean> stopCase) {
        return driveFor(ySpeed, kDriveFor_p, inches, yAcc, false, 30, stopCase);
    }

    public boolean driveForXTime(double inches, double timeOut, Supplier<Boolean> stopCase) {
        return driveFor(xSpeed, kDriveForX_p, inches, xAcc, true, timeOut, stopCase);
    }
    public boolean driveForYTime(double inches, double timeOut, Supplier<Boolean> stopCase) {
        return driveFor(ySpeed, kDriveFor_p, inches, yAcc, false, timeOut, stopCase);
    }


    public void turnPID(double maxTurn, double kp, double ki, double kd, double angle, double turnTolerance, double delay) {
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnTolerance);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            double heading = getHeading();
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(0, 0, turnFactor);
            telemetry.addData("turn fac", turnFactor);
            telemetry.addData("error", turnPID.getError());
            telemetry.update();
            return Math.abs(turnPID.getError()) < turnTolerance;
        });

        stop();
    }

    public void turnPIDslow(double angle) {
        turnPID(kTurnPIDslow_maxTurn, kTurnPIDslow_kp, kTurnPIDslow_ki, kTurnPIDslow_kd, angle, kTurnPIDslow_tolerance, 0.05);
    }

    public void turnPIDfast(double angle) {
        turnPID(kTurnPIDfast_maxTurn, kTurnPIDfast_kp, kTurnPIDfast_ki, kTurnPIDfast_kd, angle, kTurnPIDfast_tolerance, 0.05);
    }

    public void turnPIDsuperfast(double angle) {
        turnPID(kTurnPIDsuperfast_maxTurn, kTurnPIDsuperfast_kp, kTurnPIDsuperfast_ki, kTurnPIDsuperfast_kd, angle, kTurnPIDsuperfast_tolerance, 0.05);
    }

    public void driveWithCorrection(double maxSpeed, double p, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, 0, 0);
        drivePID.setOutputRange(-maxSpeed, maxSpeed);
        drivePID.setSetpoint(targetPulses);
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnDeadband);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            int encPos = getEncY();
            double heading = getHeading();
            double out = drivePID.calculate(encPos);
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(0, out, turnFactor);
            if (additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if (pass) {
                if (targetPulses > 0) {
                    return encPos > targetPulses;
                } else {
                    return encPos < targetPulses;
                }
            } else {
                return Math.abs(encPos - targetPulses) < 25;
            }
        });

        stop();
    }

    public void driveWithCorrectionFloating(double maxSpeed, double minSpeed, double p, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, 0, 0);
        drivePID.setOutputRange(-maxSpeed, maxSpeed);
        drivePID.setSetpoint(targetPulses);
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnDeadband);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            int encPos = getEncY();
            double heading = getHeading();
            double out = drivePID.calculate(encPos);
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(0, out, turnFactor);
            if (additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if (pass) {
                if (targetPulses > 0) {
                    return encPos > targetPulses;
                } else {
                    return encPos < targetPulses;
                }
            } else {
                return Math.abs(encPos - targetPulses) < 25;
            }
        });
    }

    public void driveWithCorrectionFast(double inches, double angle) {
        driveWithCorrectionFast(inches, angle, 0.05, null);
    }

    public void driveWithCorrectionSlow(double inches, double angle) {
        driveWithCorrectionSlow(inches, angle, 0.05, null);
    }

    public void driveWithCorrectionFast(double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, 0.1, angle, kDriveWithCorrectionsFast_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionSlow(double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle) {
        driveWithCorrectionFastFloating(minSpeed, inches, angle, 0.05, null);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle) {
        driveWithCorrectionSlowFloating(minSpeed, inches, angle, 0.05, null);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsFast_maxSpeed, minSpeed, kDriveWithCorrectionsFast_p, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsSlow_maxSpeed, minSpeed, kDriveWithCorrectionsSlow_p, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionFast(double inches, double angle, boolean pass) {
        driveWithCorrectionFast(inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionSlow(double inches, double angle, boolean pass) {
        driveWithCorrectionSlow(inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionFast(double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionSlow(double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle, boolean pass) {
        driveWithCorrectionFastFloating(minSpeed, inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionSuperFastFloating(double minSpeed, double maxSpeed, double inches, double angle, boolean pass) {
        driveWithCorrectionFloating(maxSpeed, minSpeed, kDriveWithCorrectionsSuperfast_p, inches, kDriveWithCorrectionsSuperfast_kp, kDriveWithCorrectionsSuperfast_ki, kDriveWithCorrectionsSuperfast_kd, kDriveWithCorrectionsSuperfast_maxTurn, angle, kDriveWithCorrectionsSuperfast_tolerance, 0.05, pass, null);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle, boolean pass) {
        driveWithCorrectionSlowFloating(minSpeed, inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsFast_maxSpeed, minSpeed, kDriveWithCorrectionsFast_p, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsSlow_maxSpeed, minSpeed, kDriveWithCorrectionsSlow_p, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, pass, additionalStopParameter);
    }

    public void stop() {
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.rearLeft.setPower(0);
        this.rearRight.setPower(0);
    }

    public static int inchesToEncoderPulses(double inches) {
        return (int) (inches / (4 * Math.PI) * 1440.0);
    }
    public void driveForFO(double maxSpeed, double p, double inches, boolean pass) { // made some changes not sure wheter works or not. PLEASE TEST
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID pid = new SynchronousPID(p, 0, 0);
        pid.setOutputRange(-maxSpeed, maxSpeed);
        pid.setSetpoint(targetPulses);

        Threading.waitFor(() -> {
            int encoderPos = getEncY();
            double speed = pid.calculate(encoderPos);
            double thetaError = AngleUtil.normalize180(getHeading());
            double x = Math.sin(Math.toRadians(thetaError)) * speed;
            double y = Math.cos(Math.toRadians(thetaError)) * speed;

            driveFieldOriented(x, y, 0);

            telemetry.addData("encPos", encoderPos);
            telemetry.addData("driveFor_error", pid.getError());
            telemetry.addData("speed", speed);
            telemetry.update();

            if(pass) {
                if(targetPulses > 0) {
                    return encoderPos > targetPulses;
                } else {
                    return encoderPos < targetPulses;
                }
            } else {
                return Math.abs(encoderPos - targetPulses) < 25;
            }
        });
    }

    public void driveForFO(double maxSpeed, double inches) {
        driveForFO(maxSpeed, kDriveFor_p, inches, false);
    }

    public void driveForFO(double inches) {
        driveForFO(0.5, inches);
    }
}

