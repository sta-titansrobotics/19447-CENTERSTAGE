package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*The alliances closest to the back board are '1'
 * The rest of the two are '2'
 *   Blue1Auto - closest to back board
 *   Blue2Auto - furtherest to back board
 *    Red1Auto -  closest to backboard
 *    Red2 Auto - furthurest to backboard
 * */
@Autonomous
public class Blue2AUto extends LinearOpMode {

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

    ElapsedTime timer = new ElapsedTime();
    public static double lastError = 0;

    @Override
    public void runOpMode() {
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

        //Reverse left side motors, as they start out reversed
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        timer.reset();
        //-------------Auto code goes here --------------------------------
        if(/*Straight*/true){
            //move it forward 70cm and push game obej
            Drive(78, 78, 78, 78);
            stopRobot();

            //back 2 cm
            Drive(-5, -5, -5, -5);
            stopRobot();
            //detect where pixel is

            //////////////-----------chain if statements
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

            //raise sliders
            Sliders.setTargetPosition(3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.5);
            while(Sliders.getCurrentPosition()<2999){

            }
            Sliders.setPower(0);



            //approach the board
            Drive(-90, -90, -90, -90);
            stopRobot();

            //align with board
            Drive(-13, 13,13, -13);
            stopRobot();
            //////////////-----------end if statement

            //drop the pixel
            Wrist.setPosition(0.50);

            //nudge the robot forward a bit to ensure the pixel drops
            Drive(-17, -17, -17, -17);
            stopRobot();

            sleep(1000);

            Drive(10, 10, 10, 10);
            stopRobot();

            //reset wrist
            Wrist.setPosition(0.79);

            //move to parking
            Drive(75, -75, -75, 75);
            stopRobot();
            //forward into parking
            Drive(-40, -40, -40, -40);
            stopRobot();
            //pull down the sliders
            Sliders.setTargetPosition(-3000);
            Sliders.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Sliders.setPower(0.4);
            while(Sliders.getCurrentPosition()>1){

            }
            Sliders.setPower(0);
        }
       /* //move it forward 60cm
        Drive(60, 60, 60, 60);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        //detect pixel and do whatever
        // left 50cm
        //drop the pixels onto back board
        //move to parking
        //  robot.Forward(40);
        // robot.StrafeLeft(35);*/


        //---------------------------------------------------------------------
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("motorFL Encoder Position: ", motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ", motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ", motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ", motorBR.getCurrentPosition());
            telemetry.addData("fasf", motorFL.getPower());
            telemetry.addData("Wrist Position", Wrist.getController().getServoPosition(5));
            if(!canContinue)
                telemetry.addData("still in the loop", 1);

            telemetry.update();
        }
    }

    //---------------------------------------------------------------------------------


    public void Drive(int TargetPositionMotorFL, int TargetPositionMotorBL, int TargetPositionMotorFR,
                      int TargetPositionMotorBR) {
        while(true){
            TargetPositionMotorFL = (int) (27.63 * TargetPositionMotorFL);
            TargetPositionMotorBL = (int) (27.63 * TargetPositionMotorBL );
            TargetPositionMotorFR = (int) (27.63 * TargetPositionMotorFR );
            TargetPositionMotorBR = (int) (27.63 * TargetPositionMotorBR);

            if(motorFL.getCurrentPosition()>TargetPositionMotorFL-0.5){
                return;
            }


            // this is in terms of cm


            motorFL.setTargetPosition(TargetPositionMotorFL);
            motorFR.setTargetPosition(TargetPositionMotorFR);
            motorBL.setTargetPosition(TargetPositionMotorBL);
            motorBR.setTargetPosition(TargetPositionMotorBR);


            motorFL.setPower(PIDControl(TargetPositionMotorFL, motorFL.getCurrentPosition())/10);
            motorBL.setPower(PIDControl(TargetPositionMotorBL, motorBL.getCurrentPosition())/10);
            motorFR.setPower(PIDControl(TargetPositionMotorFR, motorFR.getCurrentPosition())/10);
            motorBR.setPower(PIDControl(TargetPositionMotorBR, motorBR.getCurrentPosition())/10);

            // Wait until all motors reach the target position
        /*
        while (opModeIsActive() && motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) {
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        */

        }}

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
}