package team_19447;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
SECOND CAMERA CODE IS WITH 3 SLASHES ///
 */

/*The alliances closest to the back board are '1'
 * The rest of the two are '2'
 *   Blue1Auto - closest to back board
 *   Blue2Auto - furtherest to back board
 *    Red1Auto -  closest to backboard
 *    Red2 Auto - furthurest to backboard
 * */
@Autonomous
public class Blue2Autotfod extends LinearOpMode {
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

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public static final double forwardTicks = 47.63;
    public static final double strafeTicks = 49.05;

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;

    public boolean canContinue = false;

    double integralSum = 0;
    double Kp = 0.05;
    double Ki = 0;
    double Kd = 0.01;
    double Kf = 0.2;

    int visionintX=0;
    int visionintY=0;

    ElapsedTime timer = new ElapsedTime();
    public static double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //Expansion hub 3
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");  //Expansion hub 2
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");  //Control hub 3
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight"); //Control hub 2


        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); // Expansion hub 1
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); // Control hub 0
        Servo Wrist = hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 2



        // set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Sliders.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Sliders.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Wrist.setPosition(0.79);
        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");   --> Done
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); --> Done
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); for the robot to hang --> Done
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); for the robot to hang --> Done

        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "AirplaneLauncher"); --> Still have to work on //Servo Port 3

        Climbing1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climbing2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        int visiontimer = 120;

        //Reverse left side motors, as they start out reversed
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        Sliders.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        timer.reset();

        initTfod();

        while (visiontimer > 0) {

            telemetryTfod();

            // Push telemetry to the Driver Station.
            telemetry.update();

            visiontimer --;
            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        ///visionPortal2.close();

        //-------------Auto code goes here --------------------------------

            Sliders.setTargetPosition(3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.5);

            //move it forward 70cm

            Drive(45, 45, 45, 45);
            stopRobot();

            //raise sliders


            //////////////-----------chain if statements
        //--------------------MIDDLE------------------

        if (visionintX>300) {
            //drop the pixel
            Intake.setPower(0.5);
            sleep(900);
            Intake.setPower(0);
            //move away from dropped pixel
            Drive(-10, -10, -10, -10);
            stopRobot();

            //turn left here to face sliders towards board
            Drive(62, 62, -62, -62);
            stopRobot();

            //approach the board
            Drive(-200, -200, -200, -200);
            stopRobot();

            //align with board
            Drive(-12, 12, 12, 12);
            stopRobot();
            //////////////-----------end if statement

            //drop the pixel
            Wrist.setPosition(0.50);
            //nudge the robot forward a bit to ensure the pixel drops
            Drive(-20, -20, -20, -20);
            stopRobot();

            sleep(1000);

            Drive(10, 10, 10, 10);
            stopRobot();

            //reset wrist
            Wrist.setPosition(0.79);

            //move to parking on midddle side
            Drive(75, -75, -75, 75);
            stopRobot();
            //forward into parking
            Drive(-40, -40, -40, -40);
            stopRobot();
            //pull down the sliders
            Sliders.setTargetPosition(-3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.4);

            //move to parking
            Drive(-90, 90, 90, -90);
            stopRobot();
            //forward into parking
            Drive(-25, -25, -25, -25);
            stopRobot();
        }
        //---------------------LEFT SIDE-----------------------
        else if (visionintX <300&& visionintX >60) {
            //turn left here to face intake towards board
            Drive(-62, -62, 62, 62);
            stopRobot();

            //drop the pixel
            Intake.setPower(0.5);
            sleep(1500);
            Intake.setPower(0);

            //turn 180 to face slider towards board
            Drive(124, 124, -124, -124);
            stopRobot();

            //approach the board
            Drive(-200, -200, -200, -200);
            stopRobot();

            //align with board left side
            Drive(20, -20,-20, 20);
            stopRobot();
            //////////////-----------end if statement

            //drop the pixel
            Wrist.setPosition(0.5);

            //nudge the robot forward a bit to ensure the pixel drops
            Drive(20, -20, -20, -20);
            stopRobot();

            sleep(1000);

            Drive(10, 10, 10, 10);
            stopRobot();

            //reset wrist
            Wrist.setPosition(0.79);

            //pull down the sliders
            Sliders.setTargetPosition(-3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.4);

            //move to parking
            Drive(90, -90, -90, 90);
            stopRobot();
            //forward into parking
            Drive(-25, -25, -25, -25);
            stopRobot();
        } else {
            //-------------------------RIGHT SIDE---------------------------
            //turn right here to face intake towards board
            Drive(62, 62, -62, -62);
            stopRobot();

            //drop the pixel
            Intake.setPower(0.5);
            sleep(1500);
            Intake.setPower(0);

            //approach the board
            Drive(-200, -200, -200, -200);
            stopRobot();

            //align with board
            Drive(-20, 20,20, -20);
            stopRobot();
            //////////////-----------end if statement

            //drop the pixel
            Wrist.setPosition(0.5);

            //nudge the robot forward a bit to ensure the pixel drops
            Drive(-20, -20, -20, -20);
            stopRobot();

            sleep(1000);

            Drive(10, 10, 10, 10);
            stopRobot();

            //reset wrist
            Wrist.setPosition(0.79);

            //pull down the sliders
            Sliders.setTargetPosition(-3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.4);

            //move to parking
            Drive(-70, 70, 70, -70);
            stopRobot();
            //forward into parking
            Drive(-25, -25, -25, -25);
            stopRobot();
        }
    }

    public void Drive(int TargetPositionMotorFL, int TargetPositionMotorBL, int TargetPositionMotorFR,
                      int TargetPositionMotorBR) {

        TargetPositionMotorFL = (int) (16.67 * TargetPositionMotorFL);
        TargetPositionMotorBL = (int) (16.67 * TargetPositionMotorBL);
        TargetPositionMotorFR = (int) (16.67 * TargetPositionMotorFR);
        TargetPositionMotorBR = (int) (16.67 * TargetPositionMotorBR);

        // this is in terms of cm
        motorFL.setTargetPosition(TargetPositionMotorFL);
        motorBL.setTargetPosition(TargetPositionMotorBL);
        motorFR.setTargetPosition(TargetPositionMotorFR);
        motorBR.setTargetPosition(TargetPositionMotorBR);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(Math.abs(motorFL.getCurrentPosition()-TargetPositionMotorFL)>1){

            if(Math.abs(TargetPositionMotorFL - motorFL.getCurrentPosition())>300){
                motorBL.setPower(0.6);
                motorFL.setPower(0.6);
                motorBR.setPower(0.6);
                motorFR.setPower(0.6);
            }else{
                motorBL.setPower(0.2);
                motorFL.setPower(0.2);
                motorBR.setPower(0.2);
                motorFR.setPower(0.2);
            }

            telemetry.addData("motorFL Encoder Position: ", motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ", motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ", motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ", motorBR.getCurrentPosition());
            telemetry.addData("motor power", motorFL.getPower());
            //telemetry.addData("Wrist Position", Wrist.getController().getServoPosition(5));
            telemetry.update();
        }
    }

    // calculates the power which the motor should be set at.
    public double PIDControl(double setPosition, double currentPosition) {
        double error = setPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) ;
    }

    public void stopRobot() {
        // Stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        ///List<Recognition> currentRecognitions2 = tfod2.getRecognitions();
        ///telemetry.addData("# Objects Detected 2", currentRecognitions2.size());

        ///if (currentRecognitions.size() > 0) {
        ///    gameobjpos = 1;
        ///} else if (currentRecognitions2.size() > 0) {
        ///    gameobjpos = 2;
        ///} else {
        ///    gameobjpos = 0;
        ///}

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double imagecentreX = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double imagecentreY = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            visionintX = (int)imagecentreX;
            visionintY = (int)imagecentreY;

            telemetry.addData("Camera 1"," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", imagecentreX, imagecentreY);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        ///for (Recognition recognition2 : currentRecognitions2) {
        ///    double imagecentreX = (recognition2.getLeft() + recognition2.getRight()) / 2 ;
        ///    double imagecentreY = (recognition2.getTop()  + recognition2.getBottom()) / 2 ;

        ///    telemetry.addData("Camera 2"," ");
        ///    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition2.getLabel(), recognition2.getConfidence() * 100);
        ///    telemetry.addData("- Position", "%.0f / %.0f", imagecentreX, imagecentreY);
        ///    telemetry.addData("- Size", "%.0f x %.0f", recognition2.getWidth(), recognition2.getHeight());
        ///}   // end for() loop

    }   // end method telemetryTfod()

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



}
