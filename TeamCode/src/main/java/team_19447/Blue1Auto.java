package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/*The alliances closest to the back board are '1'
* The rest of the two are '2'
*   Blue1Auto - closest to back board
*   Blue2Auto - furtherest to back board
*    Red1Auto -  closest to backboard
*    Red2 Auto - furthurest to backboard
* */
@Autonomous
public class Blue1Auto extends LinearOpMode {

    //counts_per_unit_distance = ticks per revolution / wheel_circumference
    // 1425.1/29.92 = 47.63 this is the encoder count for every 1cm travelled
    public static final double forwardTicks = 47.63;

    //Counts per Unit Lateral Distance = ticks per revolution / Lateral Distance
    //note 49.05 is just a guess
    public static final double strafeTicks = 49.05;
    double default_power = 1;
    @Override
    public void runOpMode() {

        //HARDWARE MAPPING
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor RearLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor RearRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");   --> Done
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); --> Done
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); for the robot to hang --> Done
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); for the robot to hang --> Done

        Servo DropperTop =  hardwareMap.get(Servo.class, "DropperTop"); --> Done   //Servo Port 0
        Servo DropperBottom =  hardwareMap.get(Servo.class, "DropperBottom"); --> Done //Servo Port 1
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); --> the thing that rotates the dropper //Servo Port 2
        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "AirplaneLauncher"); --> Still have to work on //Servo Port 3

        Climbing1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climbing2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        //Reverse left side motors, as they start out reversed
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        autoClass447 robot = new autoClass447(forwardTicks, strafeTicks, FrontLeft, RearLeft, FrontRight, RearRight);
        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Auto code
        //move it forward 60cm
        robot.Forward(60);

        //detect pixel and do whatever


        // left 50cm
        robot.StrafeLeft(50);

        //drop the pixels onto back board

        //move
        robot.Forward(40);
        robot.StrafeLeft(35);

        if (isStopRequested()) return;

        //movement - remember capitalization
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            telemetry.addData("encoder-front-left: ", FrontLeft.getCurrentPosition());
            telemetry.addData("encoder-rear-left: ", RearLeft.getCurrentPosition());
            telemetry.addData("encoder-front-right: ", FrontRight.getCurrentPosition());
            telemetry.addData("encoder-rear-right: ", RearRight.getCurrentPosition());
            telemetry.update();
        }
    }

    //---------------------------------------------------------------------------------

     /*Ignore this part, not needed for this year robot

    private void liftPosition(int distanceCM, double Speed, int Tolerance, boolean NextSequence) {
        // convert encoder ticks to centimetres
        double tick = distanceCM * forwardTicks;
        int ticks = (int) tick;

        LiftLeft.setTargetPosition(ticks);
        LiftRight.setTargetPosition(ticks);
        LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftLeft.setPower(Speed); //prob best to leave the speed as 1.
        LiftRight.setPower(Speed);
        ((DcMotorEx) LiftLeft).setTargetPositionTolerance(Tolerance);
        ((DcMotorEx) LiftRight).setTargetPositionTolerance(Tolerance);
        while (LiftLeft.isBusy() && LiftRight.isBusy() && NextSequence) {
        }
    }*/




}