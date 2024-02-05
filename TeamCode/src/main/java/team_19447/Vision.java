package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
public class Vision extends LinearOpMode {
    public static double[] averagePixelValues = new double[3];
    public OpenCvCamera webcam;
    MyPipeline myPipeline;  // Create an instance of MyPipeline


    public void runOpMode() {
        // Set up the camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamName"));
        myPipeline = new MyPipeline();  // Initialize the MyPipeline instance
        webcam.setPipeline(myPipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(900, 720, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            // Get the average pixel values

            telemetry.addData("Average Red Value", averagePixelValues[0]);
            telemetry.addData("Average Green Value", averagePixelValues[1]);
            telemetry.addData("Average Blue Value", averagePixelValues[2]);
            telemetry.update();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public static class MyPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Calculate the average pixel values
            Scalar mean = Core.mean(input);

            // Extract red, green, and blue components
            double red = mean.val[0];
            double green = mean.val[1];
            double blue = mean.val[2];

            // Set the values in the array
            averagePixelValues[0] = red;
            averagePixelValues[1] = green;
            averagePixelValues[2] = blue;

            return input;
        }
    }
}
