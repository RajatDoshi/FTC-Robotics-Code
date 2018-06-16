package net.kno3.season.relicrecovery.nerva.robot;

import com.google.common.base.Supplier;
import com.google.common.util.concurrent.AtomicDouble;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.kno3.robot.Robot;
import net.kno3.robot.SubSystem;
import net.kno3.util.AdafruitIMU;
import net.kno3.util.AngleUtil;
import net.kno3.util.Color;
import net.kno3.util.Mecanum;
import net.kno3.util.SynchronousPID;
import net.kno3.util.Threading;


/**
 * Created by robotics on 10/28/2017.
 */

public class NervaDriveSystem extends SubSystem {
    public double kDriveFor_p;
    public double kDriveForX_p;

    public double kTurnPIDslow_maxTurn, kTurnPIDslow_kp, kTurnPIDslow_ki, kTurnPIDslow_kd, kTurnPIDslow_tolerance;
    public double kTurnPIDfast_maxTurn, kTurnPIDfast_kp, kTurnPIDfast_ki, kTurnPIDfast_kd, kTurnPIDfast_tolerance;
    public double kTurnPIDsuperfast_maxTurn, kTurnPIDsuperfast_kp, kTurnPIDsuperfast_ki, kTurnPIDsuperfast_kd, kTurnPIDsuperfast_tolerance;

    public double kDriveForYSlow_maxSpeed, kDriveForYSlow_kp, kDriveForYSlow_ki, kDriveForYSlow_kd, kDriveForYSlow_tolerance;
    public double kDriveForXSlow_maxSpeed, kDriveForXSlow_kp, kDriveForXSlow_ki, kDriveForXSlow_kd, kDriveForXSlow_tolerance;

    public double kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, kDriveWithCorrectionsSlow_maxTurn,
            kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_tolerance;
    public double kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_i, kDriveWithCorrectionsFast_d, kDriveWithCorrectionsFast_maxTurn,
            kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_tolerance;
    public double kDriveWithCorrectionsSuperfast_p, kDriveWithCorrectionsSuperfast_i, kDriveWithCorrectionsSuperfast_d, kDriveWithCorrectionsSuperfast_maxTurn,
            kDriveWithCorrectionsSuperfast_kp, kDriveWithCorrectionsSuperfast_ki, kDriveWithCorrectionsSuperfast_kd, kDriveWithCorrectionsSuperfast_tolerance;

    public double kDriveWithCorrectionsSlowX_maxSpeed, kDriveWithCorrectionsSlowX_p, kDriveWithCorrectionsSlowX_i, kDriveWithCorrectionsSlowX_d, kDriveWithCorrectionsSlowX_maxTurn,
            kDriveWithCorrectionsSlowX_kp, kDriveWithCorrectionsSlowX_ki, kDriveWithCorrectionsSlowX_kd, kDriveWithCorrectionsSlowX_tolerance;
    public double kDriveWithCorrectionsFastX_maxSpeed, kDriveWithCorrectionsFastX_p, kDriveWithCorrectionsFastX_i, kDriveWithCorrectionsFastX_d, kDriveWithCorrectionsFastX_maxTurn,
            kDriveWithCorrectionsFastX_kp, kDriveWithCorrectionsFastX_ki, kDriveWithCorrectionsFastX_kd, kDriveWithCorrectionsFastX_tolerance;
    public double kDriveWithCorrectionsSuperfastX_p, kDriveWithCorrectionsSuperfastX_i, kDriveWithCorrectionsSuperfastX_d, kDriveWithCorrectionsSuperfastX_maxTurn,
            kDriveWithCorrectionsSuperfastX_kp, kDriveWithCorrectionsSuperfastX_ki, kDriveWithCorrectionsSuperfastX_kd, kDriveWithCorrectionsSuperfastX_tolerance;

    public double xSpeed = 1; // max speed of driveForX
    public double ySpeed = 1; // max speed of driveForY
    public double xAcc = 0.35; // accuracy of driveFor in inches
    public double yAcc = 0.35;

    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    private Servo wedge;

    private Thread positionTracker;
    private double currX, currY;
    private int lastEncX, lastEncY;
    private boolean resetPositionTracker = false;

    private AdafruitIMU imu, rev;
    public double imuZeroHeading;
    public DcMotor encoder_y, encoder_x;
    public int encZeroX, encZeroY;
    private boolean isIMUReady = false;

    private boolean foLockout = false, slow = false, fieldOriented = true;

    private ColorSensor leftColor, rightColor;


    public NervaDriveSystem(Robot robot) {
        super(robot);


        kDriveFor_p = robot.settings.getDouble("kDriveFor_p");
        kDriveForX_p = robot.settings.getDouble("kDriveForX_p");

        kTurnPIDslow_maxTurn = robot.settings.getDouble("kTurnPIDslow_maxTurn");
        kTurnPIDslow_kp = robot.settings.getDouble("kTurnPIDslow_kp");
        kTurnPIDslow_ki = robot.settings.getDouble("kTurnPIDslow_ki");
        kTurnPIDslow_kd = robot.settings.getDouble("kTurnPIDslow_kd");
        kTurnPIDslow_tolerance = robot.settings.getDouble("kTurnPIDslow_tolerance");

        kTurnPIDfast_maxTurn = robot.settings.getDouble("kTurnPIDfast_maxTurn");
        kTurnPIDfast_kp = robot.settings.getDouble("kTurnPIDfast_kp");
        kTurnPIDfast_ki = robot.settings.getDouble("kTurnPIDfast_ki");
        kTurnPIDfast_kd = robot.settings.getDouble("kTurnPIDfast_kd");
        kTurnPIDfast_tolerance = robot.settings.getDouble("kTurnPIDfast_tolerance");

        kTurnPIDsuperfast_maxTurn = robot.settings.getDouble("kTurnPIDsuperfast_maxTurn");
        kTurnPIDsuperfast_kp = robot.settings.getDouble("kTurnPIDsuperfast_kp");
        kTurnPIDsuperfast_ki = robot.settings.getDouble("kTurnPIDsuperfast_ki");
        kTurnPIDsuperfast_kd = robot.settings.getDouble("kTurnPIDsuperfast_kd");
        kTurnPIDsuperfast_tolerance = robot.settings.getDouble("kTurnPIDsuperfast_tolerance");

        kDriveForYSlow_maxSpeed = robot.settings.getDouble("kDriveForYSlow_maxSpeed");
        kDriveForYSlow_kp = robot.settings.getDouble("kDriveForYSlow_kp");
        kDriveForYSlow_ki = robot.settings.getDouble("kDriveForYSlow_ki");
        kDriveForYSlow_kd = robot.settings.getDouble("kDriveForYSlow_kd");
        kDriveForYSlow_tolerance = robot.settings.getDouble("kDriveForYSlow_tolerance");

        kDriveForXSlow_maxSpeed = robot.settings.getDouble("kDriveForXSlow_maxSpeed");
        kDriveForXSlow_kp = robot.settings.getDouble("kDriveForXSlow_kp");
        kDriveForXSlow_ki = robot.settings.getDouble("kDriveForXSlow_ki");
        kDriveForXSlow_kd = robot.settings.getDouble("kDriveForXSlow_kd");
        kDriveForXSlow_tolerance = robot.settings.getDouble("kDriveForXSlow_tolerance");

        kDriveWithCorrectionsSlow_maxSpeed = robot.settings.getDouble("kDriveWithCorrectionsSlow_maxSpeed");
        kDriveWithCorrectionsSlow_p = robot.settings.getDouble("kDriveWithCorrectionsSlow_p");
        kDriveWithCorrectionsSlow_i = robot.settings.getDouble("kDriveWithCorrectionsSlow_i");
        kDriveWithCorrectionsSlow_d = robot.settings.getDouble("kDriveWithCorrectionsSlow_d");
        kDriveWithCorrectionsSlow_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsSlow_maxTurn");
        kDriveWithCorrectionsSlow_kp = robot.settings.getDouble("kDriveWithCorrectionsSlow_kp");
        kDriveWithCorrectionsSlow_ki = robot.settings.getDouble("kDriveWithCorrectionsSlow_ki");
        kDriveWithCorrectionsSlow_kd = robot.settings.getDouble("kDriveWithCorrectionsSlow_kd");
        kDriveWithCorrectionsSlow_tolerance = robot.settings.getDouble("kDriveWithCorrectionsSlow_tolerance");

        kDriveWithCorrectionsFast_maxSpeed = robot.settings.getDouble("kDriveWithCorrectionsFast_maxSpeed");
        kDriveWithCorrectionsFast_p = robot.settings.getDouble("kDriveWithCorrectionsFast_p");
        kDriveWithCorrectionsFast_i = robot.settings.getDouble("kDriveWithCorrectionsFast_i");
        kDriveWithCorrectionsFast_d = robot.settings.getDouble("kDriveWithCorrectionsFast_d");
        kDriveWithCorrectionsFast_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsFast_maxTurn");
        kDriveWithCorrectionsFast_kp = robot.settings.getDouble("kDriveWithCorrectionsFast_kp");
        kDriveWithCorrectionsFast_ki = robot.settings.getDouble("kDriveWithCorrectionsFast_ki");
        kDriveWithCorrectionsFast_kd = robot.settings.getDouble("kDriveWithCorrectionsFast_kd");
        kDriveWithCorrectionsFast_tolerance = robot.settings.getDouble("kDriveWithCorrectionsFast_tolerance");

        kDriveWithCorrectionsSuperfast_p = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_p");
        kDriveWithCorrectionsSuperfast_i = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_i");
        kDriveWithCorrectionsSuperfast_d = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_d");
        kDriveWithCorrectionsSuperfast_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_maxTurn");
        kDriveWithCorrectionsSuperfast_kp = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_kp");
        kDriveWithCorrectionsSuperfast_ki = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_ki");
        kDriveWithCorrectionsSuperfast_kd = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_kd");
        kDriveWithCorrectionsSuperfast_tolerance = robot.settings.getDouble("kDriveWithCorrectionsSuperfast_tolerance");

        kDriveWithCorrectionsSlowX_maxSpeed = robot.settings.getDouble("kDriveWithCorrectionsSlowX_maxSpeed");
        kDriveWithCorrectionsSlowX_p = robot.settings.getDouble("kDriveWithCorrectionsSlowX_p");
        kDriveWithCorrectionsSlowX_i = robot.settings.getDouble("kDriveWithCorrectionsSlowX_i");
        kDriveWithCorrectionsSlowX_d = robot.settings.getDouble("kDriveWithCorrectionsSlowX_d");
        kDriveWithCorrectionsSlowX_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsSlowX_maxTurn");
        kDriveWithCorrectionsSlowX_kp = robot.settings.getDouble("kDriveWithCorrectionsSlowX_kp");
        kDriveWithCorrectionsSlowX_ki = robot.settings.getDouble("kDriveWithCorrectionsSlowX_ki");
        kDriveWithCorrectionsSlowX_kd = robot.settings.getDouble("kDriveWithCorrectionsSlowX_kd");
        kDriveWithCorrectionsSlowX_tolerance = robot.settings.getDouble("kDriveWithCorrectionsSlowX_tolerance");

        kDriveWithCorrectionsFastX_maxSpeed = robot.settings.getDouble("kDriveWithCorrectionsFastX_maxSpeed");
        kDriveWithCorrectionsFastX_p = robot.settings.getDouble("kDriveWithCorrectionsFastX_p");
        kDriveWithCorrectionsFastX_i = robot.settings.getDouble("kDriveWithCorrectionsFastX_i");
        kDriveWithCorrectionsFastX_d = robot.settings.getDouble("kDriveWithCorrectionsFastX_d");
        kDriveWithCorrectionsFastX_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsFastX_maxTurn");
        kDriveWithCorrectionsFastX_kp = robot.settings.getDouble("kDriveWithCorrectionsFastX_kp");
        kDriveWithCorrectionsFastX_ki = robot.settings.getDouble("kDriveWithCorrectionsFastX_ki");
        kDriveWithCorrectionsFastX_kd = robot.settings.getDouble("kDriveWithCorrectionsFastX_kd");
        kDriveWithCorrectionsFastX_tolerance = robot.settings.getDouble("kDriveWithCorrectionsFastX_tolerance");

        kDriveWithCorrectionsSuperfastX_p = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_p");
        kDriveWithCorrectionsSuperfastX_i = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_i");
        kDriveWithCorrectionsSuperfastX_d = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_d");
        kDriveWithCorrectionsSuperfastX_maxTurn = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_maxTurn");
        kDriveWithCorrectionsSuperfastX_kp = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_kp");
        kDriveWithCorrectionsSuperfastX_ki = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_ki");
        kDriveWithCorrectionsSuperfastX_kd = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_kd");
        kDriveWithCorrectionsSuperfastX_tolerance = robot.settings.getDouble("kDriveWithCorrectionsSuperfastX_tolerance");
    }

    @Override
    public void init() {
        frontLeft = hardwareMap().dcMotor.get(Nerva.DRIVE_FL_KEY);
        frontRight = hardwareMap().dcMotor.get(Nerva.DRIVE_FR_KEY);
        rearLeft = hardwareMap().dcMotor.get(Nerva.DRIVE_RL_KEY);
        rearRight = hardwareMap().dcMotor.get(Nerva.DRIVE_RR_KEY);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        modeSpeed();
        brakeMode();



        Threading.startThread(() -> {
            imu = new AdafruitIMU(hardwareMap(), Nerva.IMU_KEY, true);
            //!NervaPersist.lastWasAuto);
            if(NervaPersist.lastWasAuto) {
                imuZeroHeading = -NervaPersist.lastAngle;
                telemetry().addData("IMU Zero", () -> imuZeroHeading);
                telemetry().update();
            }
            telemetry().addData("IMU initialized", true);

            isIMUReady = true;

        });

        this.encoder_x = hardwareMap().dcMotor.get(Nerva.ENCODER_X_KEY);
        this.encoder_y = hardwareMap().dcMotor.get(Nerva.ENCODER_Y_KEY);

        leftColor = hardwareMap().colorSensor.get("left color");
        rightColor = hardwareMap().colorSensor.get("right color");

        /*
        this.positionTracker = Threading.startThread(() -> {
            while(Threading.isOpModeActive() && !isIMUReady) {
                Threading.delay(0.2);
            }
            while(Threading.isOpModeActive()) {
                if(resetPositionTracker) {
                    currX = 0;
                    currY = 0;
                    resetPositionTracker = false;
                }

                int encX = getEncX();
                int encY = getEncY();
                int deltaX = encX - lastEncX;
                int deltaY = encY - lastEncY;
                double heading = getHeading();

                currX += deltaX * Math.cos(Math.toRadians(heading)) + deltaY * Math.cos(Math.toRadians(AngleUtil.normalize(90-heading)));
                currY += deltaX * Math.cos(Math.toRadians(AngleUtil.normalize(90+heading))) + deltaY * Math.cos(Math.toRadians(heading));

                lastEncX = encX;
                lastEncY = encY;
                Threading.delay(0.002);
            }
        });
        */
    }

    @Override
    public void handle() {
        double x = gamepad1().left_stick_x;
        double y = -gamepad1().left_stick_y;
        double w = gamepad1().right_stick_x;



        if(gamepad1().right_stick_button) {
            slow = false;
        }

        if(gamepad1().left_stick_button) {
            slow = true;
        }


        if(!foLockout && gamepad1().dpad_up) {
            fieldOriented = !fieldOriented;
            foLockout = true;
        }
        if(foLockout && !gamepad1().dpad_up) {
            foLockout = false;
        }


        if(gamepad1().start) zeroHeading();


        if(slow) {
            x *= 0.4;
            y *= 0.4;
            w *= 0.3;
        }
        else {
            x *= 0.8;
            y *= 0.8;
            w *= 0.7;
        }


        if(fieldOriented) {
            driveFieldOriented(x, y, w);
        } else {
            driveRobotOriented(x, y, w);
        }

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
        return AngleUtil.normalize(imu.getHeading() - imuZeroHeading);
    }

    public void zeroHeading() {
        this.imuZeroHeading = imu.getHeading();
    }

    public void zeroHeading(double heading) {
        this.imuZeroHeading = heading;
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

    public double getPosX() {
        return encoderPulsesToInches(currX);
    }

    public double getPosY() {
        return encoderPulsesToInches(currY);
    }

    public void resetPos() {
        this.resetPositionTracker = true;
    }


    public void driveRobotOriented(double x, double y, double w) {
        Mecanum.robotOriented(y, x, w, frontLeft, frontRight, rearLeft, rearRight);
    }

    public void driveFieldOriented(double x, double y, double w) {
        Mecanum.fieldOriented(y, x, w, getHeading(), frontLeft, frontRight, rearLeft, rearRight);
    }

    public void driveOffStone() {
        while(leftColor.red() > 25 || rightColor.red() > 25) {
            driveFieldOriented(1, 0, 0);
        }
        stop();
    }


    public boolean driveFor(double maxSpeed, double p, double i, double d, double inches, double accuracy, boolean isX, double timeOut) {
        resetEncX();
        resetEncY();

        int targetPulses = inchesToEncoderPulses(inches);
        int accuracyPulses = inchesToEncoderPulses(accuracy);
        SynchronousPID pid = new SynchronousPID(p, i, d);
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
             * Check if autonomous is over.
             */
            if(!Threading.isOpModeActive()) {
                return false;
            }

            /*
             * Check if loop is timed out
             */
            if((System.currentTimeMillis() - startTime) > (long)(timeOut*1000)) {
                return false;
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
            /*if(isX) {
                if ((speed > 0) && (speed < .7)) speed = 0.7;
                else if((speed < 0) && (speed > -0.7)) speed = -0.7;
            }
            else {
                if ((speed > 0) && (speed < .7)) speed = 0.7;
                else if ((speed < 0) && (speed > -0.7)) speed = -0.7;
            }*/

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
            telemetry().addData("encoder position", encoderPos);
            telemetry().addData("y raw", encoder_y.getCurrentPosition());
            telemetry().addData("target pulses", targetPulses);
            telemetry().addData("speed", speed);
            telemetry().update();
        }
    }

    public boolean driveForX(double inches) {
        return driveFor(kDriveForXSlow_maxSpeed, kDriveForXSlow_kp, kDriveForXSlow_ki, kDriveForXSlow_kd, inches, kDriveForXSlow_tolerance, true, 30);
    }

    public boolean driveForY(double inches) {
        return driveFor(kDriveForYSlow_maxSpeed, kDriveForYSlow_kp, kDriveForYSlow_ki, kDriveForYSlow_kd, inches, kDriveForYSlow_tolerance, false, 30);
    }

    public boolean driveForXTime(double inches, double timeOut) {
        return driveFor(xSpeed, kDriveForX_p, 0, 0, inches, xAcc, true, timeOut);
    }

    public void driveForYOld(double maxSpeed, double p, double i, double d, double inches, double tolerance, boolean pass) { // made some changes not sure wheter works or not. PLEASE TEST
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID pid = new SynchronousPID(p, i, d);
        pid.setOutputRange(-maxSpeed, maxSpeed);
        pid.setSetpoint(targetPulses);

        Threading.waitFor(() -> {
            int encoderPos = getEncY();
            double speed = pid.calculate(encoderPos);
            driveRobotOriented(0, speed, 0);

            telemetry().addData("encPos", encoderPos);
            telemetry().addData("driveFor_error", pid.getError());
            telemetry().addData("speed", speed);
            telemetry().update();

            if(pass) {
                if(targetPulses > 0) {
                    return encoderPos > targetPulses;
                } else {
                    return encoderPos < targetPulses;
                }
            } else {
                return Math.abs(encoderPos - targetPulses) < tolerance;
            }
        });
    }

    public void driveForYOld(double inches) {
        driveForYOld(kDriveForYSlow_maxSpeed, kDriveForYSlow_kp, kDriveForYSlow_ki, kDriveForYSlow_kd, inches, kDriveForYSlow_tolerance, false);
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

            telemetry().addData("encPos", encoderPos);
            telemetry().addData("driveFor_error", pid.getError());
            telemetry().addData("speed", speed);
            telemetry().update();

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
            telemetry().addData("turnPID_heading", heading);
            telemetry().addData("turnPID_error", turnPID.getError());
            telemetry().addData("turnPID_factor", turnFactor);
            telemetry().update();
            return Math.abs(turnPID.getError()) < turnTolerance;
        });

        stop();
    }

    public void turnPIDslow(double angle) {
        turnPID(kTurnPIDslow_maxTurn, kTurnPIDslow_kp, kTurnPIDslow_ki, kTurnPIDslow_kd, angle, kTurnPIDslow_tolerance, 0.05);
    }

    public void driveWithCorrection(double maxSpeed, double p, double i, double d, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        int zeroEnc = getEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, i, d);
        drivePID.setOutputRange(-maxSpeed, maxSpeed);
        drivePID.setSetpoint(targetPulses);
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnDeadband);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            int encPos = getEncY() - zeroEnc;
            double heading = getHeading();
            double out = drivePID.calculate(encPos);
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(0, out, turnFactor);
            if(additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if(pass) {
                if(targetPulses > 0) {
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

    public void driveWithCorrectionXY(double maxSpeed, double xp, double xi, double xd, double xInches, double p, double i, double d, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        int zeroEncY = getEncY();
        int zeroEncX = getEncX();
        int targetPulsesY = inchesToEncoderPulses(inches);
        int targetPulsesX = inchesToEncoderPulses(xInches);

        SynchronousPID driveYPID = new SynchronousPID(p, i, d);
        SynchronousPID driveXPID = new SynchronousPID(xp, xi, xd);
        driveXPID.setOutputRange(-maxSpeed, maxSpeed);
        driveXPID.setSetpoint(targetPulsesX);
        driveYPID.setOutputRange(-maxSpeed, maxSpeed);
        driveYPID.setSetpoint(targetPulsesY);
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnDeadband);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            int encPosY = getEncY() - zeroEncY;
            int encPosX = getEncX() - zeroEncX;
            double heading = getHeading();
            double outY = driveYPID.calculate(encPosY);
            double outX = driveXPID.calculate(encPosX);
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(outX, outY, turnFactor);
            if(additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if(pass) {
                if(targetPulsesY > 0 && targetPulsesX > 0) {
                    return encPosY > targetPulsesY && encPosX > targetPulsesX;
                } else {
                    return encPosY < targetPulsesY && encPosX < targetPulsesX;
                }
            } else {
                return Math.abs(encPosY - targetPulsesY) < 25 && (encPosX - targetPulsesX) < 25;
            }
        });

        stop();
    }
    public void driveWithCorrectionXY(double inchesY, double inchesX, double angle) {
        driveWithCorrectionXY(kDriveForXSlow_maxSpeed, kDriveForXSlow_kp, kDriveForXSlow_ki, kDriveForXSlow_kd, inchesX, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, inchesY, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, 0.05, false, null);
    }

    public void driveWithCorrectionFloating(double maxSpeed, double minSpeed, double p, double i, double d, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, i, d);
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
            if(additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if(pass) {
                if(targetPulses > 0) {
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
        driveWithCorrection(kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_i, kDriveWithCorrectionsFast_d, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, 0.1, angle, kDriveWithCorrectionsFast_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionSlow(double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle) {
        driveWithCorrectionFastFloating(minSpeed, inches, angle, 0.05, null);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle) {
        driveWithCorrectionSlowFloating(minSpeed, inches, angle, 0.05, null);
    }

    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsFast_maxSpeed, minSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_i, kDriveWithCorrectionsFast_d, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsSlow_maxSpeed, minSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionFast(double inches, double angle, boolean pass) {
        driveWithCorrectionFast(inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionSlow(double inches, double angle, boolean pass) {
        driveWithCorrectionSlow(inches, angle, 0.05, pass, null);
    }

    public void driveWithCorrectionFast(double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsFast_maxSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_i, kDriveWithCorrectionsFast_d, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionSlow(double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrection(kDriveWithCorrectionsSlow_maxSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, pass, additionalStopParameter);
    }


    public void driveWithCorrectionFastFloating(double minSpeed, double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsFast_maxSpeed, minSpeed, kDriveWithCorrectionsFast_p, kDriveWithCorrectionsFast_i, kDriveWithCorrectionsFast_d, inches, kDriveWithCorrectionsFast_kp, kDriveWithCorrectionsFast_ki, kDriveWithCorrectionsFast_kd, kDriveWithCorrectionsFast_maxTurn, angle, kDriveWithCorrectionsFast_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionSlowFloating(double minSpeed, double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionFloating(kDriveWithCorrectionsSlow_maxSpeed, minSpeed, kDriveWithCorrectionsSlow_p, kDriveWithCorrectionsSlow_i, kDriveWithCorrectionsSlow_d, inches, kDriveWithCorrectionsSlow_kp, kDriveWithCorrectionsSlow_ki, kDriveWithCorrectionsSlow_kd, kDriveWithCorrectionsSlow_maxTurn, angle, kDriveWithCorrectionsSlow_tolerance, delay, pass, additionalStopParameter);
    }

    public void driveWithCorrectionX(double maxSpeed, double p, double i, double d, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        int zeroEnc = getEncX();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, i, d);
        drivePID.setOutputRange(-maxSpeed, maxSpeed);
        drivePID.setSetpoint(targetPulses);
        SynchronousPID turnPID = new SynchronousPID(kp, ki, kd);
        turnPID.setOutputRange(-maxTurn, maxTurn);
        turnPID.setSetpoint(angle);
        turnPID.setDeadband(turnDeadband);

        Threading.waitFor(() -> {
            Threading.delay(delay);
            int encPos = getEncX() - zeroEnc;
            double heading = getHeading();
            double out = drivePID.calculate(encPos);
            double turnFactor = turnPID.calculateGivenError(AngleUtil.normalize180(angle - heading));
            driveRobotOriented(out, 0, turnFactor);
            if(additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if(pass) {
                if(targetPulses > 0) {
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

    public void driveWithCorrectionFloatingX(double maxSpeed, double minSpeed, double p, double i, double d, double inches, double kp, double ki, double kd, double maxTurn, double angle, double turnDeadband, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        resetEncX();
        resetEncY();
        int targetPulses = inchesToEncoderPulses(inches);
        SynchronousPID drivePID = new SynchronousPID(p, i, d);
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
            driveRobotOriented(out, 0, turnFactor);
            if(additionalStopParameter != null && additionalStopParameter.get()) {
                return true;
            }
            if(pass) {
                if(targetPulses > 0) {
                    return encPos > targetPulses;
                } else {
                    return encPos < targetPulses;
                }
            } else {
                return Math.abs(encPos - targetPulses) < 25;
            }
        });
    }


    public void driveWithCorrectionSlowX(double inches, double angle) {
        driveWithCorrectionSlowX(inches, angle, 0.05, null);
    }

    public void driveWithCorrectionSlowX(double inches, double angle, double delay, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionX(kDriveWithCorrectionsSlowX_maxSpeed, kDriveWithCorrectionsSlowX_p, kDriveWithCorrectionsSlowX_i, kDriveWithCorrectionsSlowX_d, inches, kDriveWithCorrectionsSlowX_kp, kDriveWithCorrectionsSlowX_ki, kDriveWithCorrectionsSlowX_kd, kDriveWithCorrectionsSlowX_maxTurn, angle, kDriveWithCorrectionsSlowX_tolerance, delay, false, additionalStopParameter);
    }

    public void driveWithCorrectionSlowX(double inches, double angle, boolean pass) {
        driveWithCorrectionSlowX(inches, angle, 0.05, pass, null);
    }


    public void driveWithCorrectionSlowX(double inches, double angle, double delay, boolean pass, Supplier<Boolean> additionalStopParameter) {
        driveWithCorrectionX(kDriveWithCorrectionsSlowX_maxSpeed, kDriveWithCorrectionsSlowX_p, kDriveWithCorrectionsSlowX_i, kDriveWithCorrectionsSlowX_d, inches, kDriveWithCorrectionsSlowX_kp, kDriveWithCorrectionsSlowX_ki, kDriveWithCorrectionsSlowX_kd, kDriveWithCorrectionsSlowX_maxTurn, angle, kDriveWithCorrectionsSlowX_tolerance, delay, pass, additionalStopParameter);
    }





    @Override
    public void stop() {
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.rearLeft.setPower(0);
        this.rearRight.setPower(0);
    }


    public static int inchesToEncoderPulses(double inches) {
        return (int) ((inches *1440.0) / (4*Math.PI));
    }

    public static double encoderPulsesToInches(double pulses) {
        return pulses * (4*Math.PI) / 1440.0;
    }


}