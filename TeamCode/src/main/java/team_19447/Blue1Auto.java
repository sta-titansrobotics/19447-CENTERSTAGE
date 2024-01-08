package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/*The alliances closest to the back board are '1'
* The rest of the two are '2'
*   Blue1Auto - closest to back board
*   Blue2Auto - furtherest to back board
*    Red1Auto -  closest to backboard
*    Red2 Auto - furthurest to backboard
* */
@Autonomous
public class Blue1Auto extends LinearOpMode {

    public static final double forwardTicks = 47.63;
    public static final double strafeTicks = 49.05;
    double default_power = 1;

    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    public int leftPos1;
    public int leftPos2;
    public int rightPos1;
    public int rightPos2;
    public boolean canContinue = false;

    double integralSum = 0;
    double Kp = 0.015;
    double Ki = 0;
    double Kd = 0.01;
    double Kf = 0.2;

    ElapsedTime timer = new ElapsedTime();
    public static double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");   --> Done
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); --> Done
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); for the robot to hang --> Done
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); for the robot to hang --> Done

        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); --> the thing that rotates the dropper //Servo Port 2
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
        //move it forward 60cm
        while(!canContinue)
            Drive(60, 60, 60, 60);
        canContinue = false;

        //detect pixel and do whatever


        // left 50cm
        while(!canContinue)
            Drive(50, 50, 50 ,50 );//not correct yet
        canContinue = false;
        //drop the pixels onto back board

        //move to parking
      //  robot.Forward(40);
       // robot.StrafeLeft(35);


        //---------------------------------------------------------------------
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("motorFL Encoder Position: ", motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ", motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ", motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ", motorBR.getCurrentPosition());
            telemetry.addData("Kd: " , Kd);
            telemetry.addData("Kp: " , Kp);
            telemetry.addData("current power --> BackLeft: " , motorBL.getPower());
            telemetry.addData("current power --> BackRight: " , motorBR.getPower());

            telemetry.update();
        }
    }

    //---------------------------------------------------------------------------------


    public void Drive(int TargetPositionMotorFL, int TargetPositionMotorBL, int TargetPositionMotorFR,
                      int TargetPositionMotorBR) {

        if(motorFL.getCurrentPosition()>TargetPositionMotorFL-0.5){
            canContinue = true;
        }

        // this is in terms of cm
        TargetPositionMotorFL = (int) (47.63 * TargetPositionMotorFL);
        TargetPositionMotorBL = (int) (47.63 * TargetPositionMotorBL );
        TargetPositionMotorFR = (int) (47.63 * TargetPositionMotorFR );
        TargetPositionMotorBR = (int) (47.63 * TargetPositionMotorBR);

        motorFL.setTargetPosition(TargetPositionMotorFL);
        motorFR.setTargetPosition(TargetPositionMotorFR);
        motorBL.setTargetPosition(TargetPositionMotorBL);
        motorBR.setTargetPosition(TargetPositionMotorBR);


        motorFL.setPower(PIDControl(TargetPositionMotorFL, motorFL.getCurrentPosition())/3);
        motorBL.setPower(PIDControl(TargetPositionMotorBL, motorBL.getCurrentPosition())/3);
        motorFR.setPower(PIDControl(TargetPositionMotorFR, motorFR.getCurrentPosition())/3);
        motorBR.setPower(PIDControl(TargetPositionMotorBR, motorBR.getCurrentPosition())/3);

        // Wait until all motors reach the target position
        /*while (opModeIsActive() && motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) {
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);*/

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



}