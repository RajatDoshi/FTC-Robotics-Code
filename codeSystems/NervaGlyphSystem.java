package net.kno3.season.relicrecovery.nerva.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import net.kno3.robot.Robot;
import net.kno3.robot.SubSystem;
import net.kno3.util.Threading;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotics on 10/28/2017.
 */

public class NervaGlyphSystem extends SubSystem {
    public DcMotor liftMotor;
    private Servo liftHolder;
    public int liftEncZero;
    private int state = 0;
    private Servo bottomLeft, bottomRight, topLeft, topRight;

    private DistanceSensor glyph1;

    private ColorSensor topColor, bottomColor;
    private ColorSensor colorLeft, colorRight;

    private TopClampState topClampState;
    private BottomClampState bottomClampState;
    private Thread changeState = null;

    private boolean rightBumperLock, leftBumperLock;
    private boolean dpadUpLock, dpadDownLock, dpadRightLock, dpadLeftLock, backLock;
    private boolean firstRun = true;
    private boolean tLED = false;
    private boolean bLED = false;

    // Used to determine how often to refresh servo values
    // We'll refresh the servo values every <refreshTimeReset> loops through handler().
    private final int refreshTimeReset = 2;
    private int refreshTime = refreshTimeReset;

    public NervaGlyphSystem(Robot robot) {
        super(robot);

        GlyphLiftHolderState.OPEN.updateSettings(robot);
        GlyphLiftHolderState.CLOSED.updateSettings(robot);
        BottomClampState.OPEN.updateSettings(robot);
        BottomClampState.CLOSED.updateSettings(robot);
        BottomClampState.IDLE.updateSettings(robot);
        BottomClampState.HALF.updateSettings(robot);
        TopClampState.OPEN.updateSettings(robot);
        TopClampState.CLOSED.updateSettings(robot);
        TopClampState.IDLE.updateSettings(robot);
        TopClampState.HALF.updateSettings(robot);
    }

    @Override
    public void init() {
        liftMotor = hardwareMap().dcMotor.get(Nerva.GLYPH_LIFT_KEY);
        liftHolder = hardwareMap().servo.get(Nerva.GLYPH_LIFT_HOLDER_KEY);
        bottomLeft = hardwareMap().servo.get(Nerva.GLYPH_CLAMP_BOTTOM_LEFT_KEY);
        bottomRight = hardwareMap().servo.get(Nerva.GLYPH_CLAMP_BOTTOM_RIGHT_KEY);
        topLeft = hardwareMap().servo.get(Nerva.GLYPH_CLAMP_TOP_LEFT_KEY);
        topRight = hardwareMap().servo.get(Nerva.GLYPH_CLAMP_TOP_RIGHT_KEY);
        glyph1 = hardwareMap().get(DistanceSensor.class, "glyph_dist_right");
        //glyph2 = hardwareMap().get(DistanceSensor.class, "glyph_dist_left");
        topColor = hardwareMap().colorSensor.get("tColor");
        bottomColor = hardwareMap().colorSensor.get("bColor");

        //liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(NervaPersist.lastWasAuto) {
            this.liftEncZero = NervaPersist.lastLiftEncZero;
        } else {
            zeroLiftEnc();
        }

        setBottomClamp(BottomClampState.OPEN);
        setTopClamp(TopClampState.OPEN);
        setHolderState(GlyphLiftHolderState.CLOSED);
        bottomColor.enableLed(bLED);
        topColor.enableLed(tLED);
    }

    @Override
    public void handle() {
        //if(firstRun) {
        //    setTopClamp(TopClampState.OPEN);
        //   setBottomClamp(BottomClampState.OPEN);
        //    firstRun = false;
        //}
        TopClampState topState = this.topClampState;
        BottomClampState botState = this.bottomClampState;


        //changed liftPow to 0.7
        double liftPow = gamepad2().right_stick_y;
        if(liftPow > 0)
            liftPow *= 0.7;

        setLiftPower(liftPow);

        if (!dpadUpLock && gamepad2().dpad_up) {
            if(topClampState != TopClampState.CLOSED) {
                topState = TopClampState.CLOSED;
            } else {
                topState = TopClampState.OPEN;
            }
            dpadUpLock = true;
        }
        if (dpadUpLock && !gamepad2().dpad_up) {
            dpadUpLock = false;
        }

        if (!dpadDownLock && gamepad2().dpad_down) {
            if(bottomClampState != BottomClampState.CLOSED) {
                botState = BottomClampState.OPEN;
            } else {
                botState= BottomClampState.OPEN;
            }
            dpadDownLock = true;
        }
        if (dpadDownLock && !gamepad2().dpad_down) {
            dpadDownLock = false;
        }

        if (!dpadRightLock && gamepad2().dpad_right) {
            topState = TopClampState.OPEN;
            botState = BottomClampState.OPEN;
            dpadRightLock = true;
        }
        if (dpadRightLock && !gamepad2().dpad_right) {
            dpadRightLock = false;
        }

        if (!dpadLeftLock && gamepad2().dpad_left) {
            topState = TopClampState.HALF;
            botState= BottomClampState.HALF;
            dpadLeftLock = true;
        }
        if (dpadLeftLock && !gamepad2().dpad_left) {
            dpadLeftLock = false;
        }


        if(gamepad2().left_bumper) {
            setHolderState(GlyphLiftHolderState.CLOSED);
        } else {
            setHolderState(GlyphLiftHolderState.OPEN);
        }

        if(gamepad2().back) {
            backLock = true;
        } else if(backLock) {
            backLock = false;
            zeroLiftEnc();
        }


        /*
         * Refresh the servo values, sometimes they "float" over time.
         */
        setTopClamp(topState);
        setBottomClamp(botState);

    }

    public int getLiftEnc() {
        return liftMotor.getCurrentPosition() - liftEncZero;
    }

    public void zeroLiftEnc() {
        this.liftEncZero = liftMotor.getCurrentPosition();
    }

    public void setLiftPower(double power) {
        if(getLiftEnc() <= 0 && power > 0 && !gamepad2().back) {
            power = 0;
        }
        if(getLiftEnc() > 4750 && power < 0 && !gamepad2().back) {
            power = 0;
        }

        this.liftMotor.setPower(power);
    }

    public boolean hasGlyph() {
        return glyph1.getDistance(DistanceUnit.INCH) < 4;
    }
    public void liftPosMode() {
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftRunMode() {
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftPos(int position) {
        this.liftMotor.setTargetPosition(position);
        this.liftMotor.setPower(-0.7);
    }

    public void setHolderState(GlyphLiftHolderState state) {
        this.liftHolder.setPosition(state.getPosition());
    }

    public void setBottomClamp(BottomClampState state) {
        this.bottomClampState = state;
        if(state == BottomClampState.CLOSED) bLED = true;
        else bLED = false;
        bottomLeft.setPosition(state.getLeft());
        bottomRight.setPosition(state.getRight());
        bottomColor.enableLed(bLED);
    }

    public void setTopClamp(TopClampState state) {
        this.topClampState = state;
        if(state == TopClampState.CLOSED) tLED = true;
        else tLED = false;
        topLeft.setPosition(state.getLeft());
        topRight.setPosition(state.getRight());
        topColor.enableLed(tLED);
    }

    @Override
    public void stop() {
        this.liftMotor.setPower(0);
    }

    public enum GlyphLiftHolderState {
        OPEN("glyph_lift_holder_open"),
        CLOSED("glyph_lift_holder_closed");

        private String key;
        private double pos;

        GlyphLiftHolderState(String key) {
            this.key = key;
        }

        public double getPosition() {
            return pos;
        }

        public void updateSettings(Robot robot) {
            this.pos = robot.settings.getDouble(key);
        }
    }

    public enum BottomClampState {
        OPEN("glyph_clamp_bottom_left_open", "glyph_clamp_bottom_right_open"),
        CLOSED("glyph_clamp_bottom_left_closed", "glyph_clamp_bottom_right_closed"),
        IDLE("glyph_clamp_bottom_left_idle", "glyph_clamp_bottom_right_idle"),
        HALF("glyph_clamp_bottom_left_half", "glyph_clamp_bottom_right_half");

        private String left, right;
        private double leftP, rightP;

        BottomClampState(String left, String right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return leftP;
        }

        public double getRight() {
            return rightP;
        }

        public void updateSettings(Robot robot) {
            leftP = robot.settings.getDouble(left);
            rightP = robot.settings.getDouble(right);
        }
    }

    public enum TopClampState {
        OPEN("glyph_clamp_top_left_open", "glyph_clamp_top_right_open"),
        CLOSED("glyph_clamp_top_left_closed", "glyph_clamp_top_right_closed"),
        IDLE("glyph_clamp_top_left_idle", "glyph_clamp_top_right_idle"),
        HALF("glyph_clamp_top_left_half", "glyph_clamp_top_right_half");

        private String left, right;
        private double leftP, rightP;

        TopClampState(String left, String right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return leftP;
        }

        public double getRight() {
            return rightP;
        }

        public void updateSettings(Robot robot) {
            leftP = robot.settings.getDouble(left);
            rightP = robot.settings.getDouble(right);
        }
    }

}
