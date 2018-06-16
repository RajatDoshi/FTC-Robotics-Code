package net.kno3.season.relicrecovery.nerva.program.calibration;

import com.qualcomm.robotcore.hardware.HardwareMap;

import net.kno3.robot.RobotSettings;
import net.kno3.season.relicrecovery.nerva.robot.Nerva;

/**
 * Created by robotics on 10/28/2017.
 */

public class ServoDefaults {
    public static void resetAllServos(HardwareMap hardwareMap, RobotSettings settings) {
        hardwareMap.servo.get(Nerva.GLYPH_CLAMP_BOTTOM_LEFT_KEY).setPosition(settings.getDouble("glyph_clamp_bottom_left_open"));
        hardwareMap.servo.get(Nerva.GLYPH_CLAMP_TOP_LEFT_KEY).setPosition(settings.getDouble("glyph_clamp_top_left_open"));
        hardwareMap.servo.get(Nerva.GLYPH_CLAMP_BOTTOM_RIGHT_KEY).setPosition(settings.getDouble("glyph_clamp_bottom_right_open"));
        hardwareMap.servo.get(Nerva.GLYPH_CLAMP_TOP_RIGHT_KEY).setPosition(settings.getDouble("glyph_clamp_top_right_open"));
        hardwareMap.servo.get(Nerva.JEWEL_ARM_LEFT_KEY).setPosition(settings.getDouble("jewel_arm_left_up"));
        //hardwareMap.servo.get(Nerva.JEWEL_ARM_RIGHT_KEY).setPosition(settings.getDouble("jewel_arm_right_up"));
        hardwareMap.servo.get(Nerva.RELIC_CLAMP_KEY).setPosition(settings.getDouble("relic_clamp_idle"));
        hardwareMap.servo.get(Nerva.RELIC_ROT_KEY).setPosition(settings.getDouble("relic_rot_idle"));
        hardwareMap.servo.get(Nerva.INTAKE_LEFT_ARM_KEY).setPosition(settings.getDouble("intake_arm_left_out"));
        hardwareMap.servo.get(Nerva.INTAKE_RIGHT_ARM_KEY).setPosition(settings.getDouble("intake_arm_right_out"));
    }
}
