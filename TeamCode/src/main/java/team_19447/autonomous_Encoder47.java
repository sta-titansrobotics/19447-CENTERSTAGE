package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class autonomous_Encoder47 extends LinearOpMode {

    public static final double forwardTicks = 52.3;
    public static final double strafeTicks = 54.05;
    double default_power = 1;
    private DcMotor LiftLeft, LiftRight;
    private Servo Cam, verticalRack;

    @Override
    public void runOpMode() {

        //HARDWARE MAPPING
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor RearLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor RearRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        //verticalRack = hardwareMap.get(Servo.class, "verticalRack");
        //Cam = hardwareMap.get(Servo.class, "Cam");

        //Reverse left side motors, as they start out reversed
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        autoClass447 robot = new autoClass447(forwardTicks, strafeTicks, FrontLeft, RearLeft, FrontRight, RearRight);
        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //Auto code
        //move it forward by 60cm
        robot.Forward(60);


        robot.StrafeLeft(60, true);

        //drop the pixels
        robot.Forward(40);
        robot.StrafeLeft(35, true);




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
    }


    public void raiseRack() {
        verticalRack.setPosition(0);

    }

    public void pickupCone() {
        verticalRack.setPosition(1);
        sleep(2000);
        Cam.setPosition(0.25); //turn 45 deg to pick it up
        sleep(1000);
        verticalRack.setPosition(0);
        sleep(1000);
    }

    public void dropCone() {
        Cam.setPosition(-0.25); //revert and turn back the 45 deg to drop it.

    public void rack(double position) {
        verticalRack.setPosition(position);
    }
     */
}