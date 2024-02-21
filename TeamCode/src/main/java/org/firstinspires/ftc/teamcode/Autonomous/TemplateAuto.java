package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Pipeline;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
@Disabled
public class TemplateAuto extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvWebcam webcam;
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern pipelineError = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineBroken = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineNotReady = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineReady = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //blinkin.
        blinkin.setPattern(pipelineNotReady);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Blue"), cameraMonitorViewId); //TODO: Change webcam name
        Pipeline detector = new Pipeline(telemetry, StartPosition.RED_STAGE, blinkin); //TODO: Change start position
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // this camera supports 1280x800, 1280x720, 800x600, 640x480, and 320x240
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
            }
            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        detector.toggleTelemetry();
        telemetry.clearAll();

        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();
            blinkin.setPattern(pipelineError);

            while(detector.getPropLocation() == null && totalTimeWaited < 7000 && !isStopRequested()) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
                blinkin.setPattern(pipelineBroken);
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
            blinkin.setPattern(pipelineReady);
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //detector.toggleTelemetry();
        //telemetry.clearAll();

        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " RED_STAGE" + location); //TODO: Change name in telemetry
        telemetry.update();

        switch(location) {
            case LEFT:
                sleep(5000);
                break;
            case CENTER:
                sleep(5000);
                break;
            case RIGHT:
                sleep(5000);
                break;
            default:
                throw new IllegalArgumentException("The code is most certainly severely screwed up.");
        }
    }
}