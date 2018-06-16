package net.kno3.season.relicrecovery.nerva.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.Robot;
import net.kno3.robot.SubSystem;
import net.kno3.util.Threading;
import net.kno3.opMode.DriverControlledProgram;

/**
 * Created by robotics on 10/28/2017.
 */

public class NervaRelicSystem extends SubSystem {
    private DcMotor armMotor;
    private Servo clampServo, rotServo;

    private int armEncZero;

    private boolean bLock = false;
    private boolean clampOpen = false;

    public boolean limits = true;


    public NervaRelicSystem(Robot robot) {
        super(robot);

        RotPosition.UP.updateSettings(robot);
        RotPosition.DOWN.updateSettings(robot);
        RotPosition.IDLE.updateSettings(robot);
        ClampPosition.CLAMPED.updateSettings(robot);
        ClampPosition.OPEN.updateSettings(robot);
        ClampPosition.IDLE.updateSettings(robot);
    }

    @Override
    public void init() {
        this.armMotor = hardwareMap().dcMotor.get(Nerva.RELIC_ARM_KEY);
        this.clampServo = hardwareMap().servo.get(Nerva.RELIC_CLAMP_KEY);
        this.rotServo = hardwareMap().servo.get(Nerva.RELIC_ROT_KEY);

        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        zeroArmEnc();
        setClampState(ClampPosition.IDLE);
        setRotState(RotPosition.IDLE);
    }

    @Override
    public void handle() {
        double armPower = -gamepad2().left_stick_y;
        if(Math.abs(armPower) >= 0.1) {
            setArmPower(armPower*armPower*armPower*armPower * Math.signum(armPower));
        } else {
            setArmPower(0);
        }

        if(gamepad2().a) {
            setRotState(RotPosition.DOWN);
        }

        if(gamepad2().b && !bLock) {
            clampOpen = !clampOpen;
            if(clampOpen) {
                setClampState(ClampPosition.OPEN);
            } else {
                setClampState(ClampPosition.CLAMPED);
            }
            bLock = true;
        }
        if(!gamepad2().b && bLock) {
            bLock = false;
        }

        if(gamepad2().y) {
            setRotState(RotPosition.UP);
        }
    }



    public int getArmEnc() {
        return -armMotor.getCurrentPosition() - armEncZero;
    }

    public void zeroArmEnc() {
        this.armEncZero = -armMotor.getCurrentPosition();
    }


    public void setClampState(ClampPosition clampState) {
        this.clampServo.setPosition(clampState.getPosition());
    }

    public void setRotState(RotPosition rotState) {
        this.rotServo.setPosition(rotState.getPosition());
    }

    //TODO check the encoder value
    public void setArmPower(double power) {
        if(limits) {
            if (getArmEnc() <= 0 && power < 0) {
                power = 0;
            }
            if (getArmEnc() > 12250 && power > 0) {
                power = 0;
            }
        }

        this.armMotor.setPower(power);
    }



    @Override
    public void stop() {
        setArmPower(0);
    }

    public enum RotPosition {
        UP("relic_rot_up"),
        DOWN("relic_rot_down"),
        IDLE("relic_rot_idle");

        private String key;
        private double pos;

        RotPosition(String key) {
            this.key = key;
        }

        public double getPosition() {
            return pos;
        }

        public void updateSettings(Robot robot) {
            this.pos = robot.settings.getDouble(key);
        }
    }

    public enum ClampPosition {
        CLAMPED("relic_clamp_closed"),
        OPEN("relic_clamp_open"),
        IDLE("relic_clamp_idle");

        private String key;
        private double pos;

        ClampPosition(String key) {
            this.key = key;
        }

        public double getPosition() {
            return pos;
        }

        public void updateSettings(Robot robot) {
            this.pos = robot.settings.getDouble(key);
        }
    }
}
