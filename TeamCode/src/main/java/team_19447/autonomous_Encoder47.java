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

        //move it forward by 40cm
        robot.Forward(40);


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
}