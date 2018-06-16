package net.kno3.season.relicrecovery.nerva.program.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import net.kno3.util.LCR;
import net.kno3.util.Threading;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by robotics on 10/29/2017.
 */

public class VuforiaScanner {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private boolean initialized;
    private RelicRecoveryVuMark lastValid = RelicRecoveryVuMark.UNKNOWN;

    public VuforiaScanner(HardwareMap hardwareMap, Telemetry telemetry) {
        Threading.async(() -> {
            telemetry.addData("vuin", "Initializing Vuforia...");
            telemetry.update();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AVXTiEb/////AAAAGewh2CgozE5ajHRzFN4e/xlI5pr4xPKYRr5SI5UTblDbfQ6TAbYmT4n+1vW0/pQsHXAh1WXHx17E2sqsES2XiY0y3t22g3XvsSJXI2apOhpmfxxTYnkWxNjOS97q2SwdXXL1JjdHHyLLkyiCdU3cDUMdhy2hXcDCRAz0p/KYI5SpIwlltPfAz+IHX+9bydMDlhCGQN/mHmqj8F0EqMNfEU+KMl0nZqK7my3xYbNxbUpF/BwWVw7xyOFQtRn/4y0gDtqV7KcEgVTVvE2WIw5tDJCvTlAkZQFCTyn/9VL3E+NOQsyFfG5JFO2ePW06dr9kke2z0M8vayJp6Y+LJSVfenCtKO8ihV9H1E2OV/nXdmir";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
            relicTrackables.activate();
            initialized = true;
            telemetry.addData("vuin", "Vuforia Ready!");
            telemetry.update();
        });
    }

    public boolean isInitialized() {
        return initialized;
    }

    public RelicRecoveryVuMark scan() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if(vuMark != RelicRecoveryVuMark.UNKNOWN)
            lastValid = vuMark;
        return vuMark;
    }

    public RelicRecoveryVuMark getLastValid() {
        return lastValid;
    }
}
