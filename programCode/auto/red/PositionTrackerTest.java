package net.kno3.season.relicrecovery.nerva.program.auto.red;

/**
 * Created by robotics on 10/29/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import net.kno3.season.relicrecovery.nerva.program.auto.NervaAutoRed;
import net.kno3.season.relicrecovery.nerva.program.auto.VuforiaScanner;
import net.kno3.season.relicrecovery.nerva.robot.NervaGlyphSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaIntakeSystem;
import net.kno3.season.relicrecovery.nerva.robot.NervaPersist;
import net.kno3.util.AngleUtil;
import net.kno3.util.AutoTransitioner;
import net.kno3.util.Threading;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by robotics on 10/29/2017.
 */

@Autonomous(name = "driveForXYTest")
public class PositionTrackerTest extends NervaAutoRed {

    @Override
    public void postInit() {
        super.postInit();
    }

    @Override
    public void main() {
        drive.driveWithCorrectionXY(24, 24, 0);
    }
}