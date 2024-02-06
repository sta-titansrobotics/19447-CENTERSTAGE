package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class Blue1Auto extends LinearOpMode {

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
        // Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //Expansion hub 3
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");  //Expansion hub 2
       motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");  //Control hub 3
         motorBR = hardwareMap.get(DcMotor.class, "motorBackRight"); //Control hub 2

        // set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); //--> the thing that rotates the dropper //Servo Port 2
        Wrist.setPosition(0.77);
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

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //-------------Auto code goes here --------------------------------
            //move it forward 60cm

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
            // robot.StrafeLeft(35);


            //---------------------------------------------------------------------

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
        if(motorFL.getCurrentPosition()>TargetPositionMotorFL-1) {
            return;
        }
            TargetPositionMotorFL = (int) (47.63 * TargetPositionMotorFL);
            TargetPositionMotorBL = (int) (47.63 * TargetPositionMotorBL);
            TargetPositionMotorFR = (int) (47.63 * TargetPositionMotorFR);
            TargetPositionMotorBR = (int) (47.63 * TargetPositionMotorBR);


            // this is in terms of cm

            motorFL.setPower(PIDControl(TargetPositionMotorFL, motorFL.getCurrentPosition()) / 10);
            motorBL.setPower(PIDControl(TargetPositionMotorBL, motorBL.getCurrentPosition()) / 10);
            motorFR.setPower(PIDControl(TargetPositionMotorFR, motorFR.getCurrentPosition()) / 10);
            motorBR.setPower(PIDControl(TargetPositionMotorBR, motorBR.getCurrentPosition()) / 10);

        // Wait until all motors reach the target position

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

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