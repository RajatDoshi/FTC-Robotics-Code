package net.kno3.season.relicrecovery.nerva.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import net.kno3.robot.Robot;
import net.kno3.util.SimpleColor;

/**
 * Created by robotics on 10/28/2017.
 */

public class Nerva extends Robot {
    public static final String DRIVE_FL_KEY = "flm";
    public static final String DRIVE_FR_KEY = "frm";
    public static final String DRIVE_RL_KEY = "rlm";
    public static final String DRIVE_RR_KEY = "rrm";
    public static final String IMU_KEY = "imu";
    public static final String ENCODER_X_KEY = "encX";
    public static final String ENCODER_Y_KEY = "encY";
    public static final String COLOR_LEFT_KEY = "color left";
    public static final String COLOR_RIGHT_KEY = "color right";

    public static final String GLYPH_LIFT_KEY = "glyft motor";
    public static final String GLYPH_LIFT_HOLDER_KEY = "Glyft Clamp";
    public static final String GLYPH_CLAMP_BOTTOM_LEFT_KEY = "Gripper BL";
    public static final String GLYPH_CLAMP_BOTTOM_RIGHT_KEY = "Gripper BR";
    public static final String GLYPH_CLAMP_TOP_LEFT_KEY = "Gripper TL";
    public static final String GLYPH_CLAMP_TOP_RIGHT_KEY = "Gripper TR";

    public static final String RELIC_ARM_KEY = "relic motor";
    public static final String RELIC_CLAMP_KEY = "Relic Grabber";
    public static final String RELIC_ROT_KEY = "Relic Rot";

    public static final String JEWEL_ARM_LEFT_KEY = "Left Jewel";
    public static final String JEWEL_ARM_RIGHT_KEY = "Right Jewel";
    public static final String JEWEL_KNOCKER_LEFT_KEY = "Left Knocker";
    public static final String JEWEL_SENSOR_LEFT_KEY = "Left Jewel Color";

    public static final String INTAKE_LEFT_ARM_KEY = "Intake Left Arm";
    public static final String INTAKE_RIGHT_ARM_KEY = "Intake Right Arm";

    public static SimpleColor ALLIANCE_COLOR;


    public Nerva(OpMode opMode, SimpleColor allianceColor) {
        super(opMode);

        ALLIANCE_COLOR = allianceColor;

        putSubSystem("drive", new NervaDriveSystem(this));
        putSubSystem("glyph", new NervaGlyphSystem(this));
        putSubSystem("jewel", new NervaJewelSystem(this));
        putSubSystem("relic", new NervaRelicSystem(this));
        putSubSystem("intake", new NervaIntakeSystem(this));
    }
}
