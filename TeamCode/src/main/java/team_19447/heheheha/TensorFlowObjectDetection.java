package team_19447.heheheha;

import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Totally my code
 */

@TeleOp
public class TensorFlowObjectDetection extends LinearOpMode {

    int gameobjpos = 0;
    // guide to ^^^ gameobjpos
    // if camera1 then = 1 if camera2 then = 2 if none then = 0

    private static final String TFOD_MODEL_ASSET = "lesser_comp.tflite";

    private static final String[] LABELS = {
            "redobject",
            "blueobject",
    };

    /* The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    ///private TfodProcessor tfod2;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    ///private VisionPortal visionPortal2;

    @Override
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        ///List<Recognition> currentRecognitions2 = tfod2.getRecognitions();
        ///telemetry.addData("# Objects Detected 2", currentRecognitions2.size());

        ///if (currentRecognitions.size() > 0) {
        ///    gameobjpos = 1;
        ///    telemetry.addData("game object position", "");
        ///} else if (currentRecognitions2.size() > 0) {
        ///    gameobjpos = 2;
        ///    telemetry.addData("game object position", "");
        ///} else {
        ///    gameobjpos = 0;
        ///    telemetry.addData("game object position", "");
        ///}

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double imagecentreX = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double imagecentreY = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData("Camera 1"," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", imagecentreX, imagecentreY);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        ///for (Recognition recognition2 : currentRecognitions2) {
        ///    double imagecentreX2 = (recognition2.getLeft() + recognition2.getRight()) / 2 ;
        ///    double imagecentreY2 = (recognition2.getTop()  + recognition2.getBottom()) / 2 ;

        ///    telemetry.addData("Camera 2"," ");
        ///    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition2.getLabel(), recognition2.getConfidence() * 100);
        ///    telemetry.addData("- Position", "%.0f / %.0f", imagecentreX2, imagecentreY2);
        ///    telemetry.addData("- Size", "%.0f x %.0f", recognition2.getWidth(), recognition2.getHeight());
        ///}   // end for() loop

    }   // end method telemetryTfod()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        ///tfod2 = TfodProcessor.easyCreateWithDefaults();

        ///tfod2 = new TfodProcessor.Builder()
        ///        .setModelAssetName(TFOD_MODEL_ASSET)
        ///        .setModelLabels(LABELS)
        ///        .build();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        ///visionPortal2 = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), tfod);

    }   // end method initTfod()
}   // end class
