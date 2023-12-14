package team_19447.heheheha;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class PushandHang extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); //done   expansion hub 0
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); //done     control hub 3
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); // done    expansion hub 3
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2");// done     control hub 0

        Servo DropperTop = hardwareMap.get(Servo.class, "DropperTop"); //  control hub  servo port 1

        Servo DropperBottom = hardwareMap.get(Servo.class, "DropperBottom"); // control hub servo port 2
        Servo Wrist = hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 0

        /*
        Climbing1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Climbing1.setDirection(DcMotorSimple.Direction.REVERSE);

        Climbing1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climbing2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Climbing2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Climbing2.setDirection(DcMotorSimple.Direction.REVERSE);*/

        //Reverse right side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Intake
            if (gamepad2.right_bumper)
                Intake.setPower(0.6);
            else
                Intake.setPower(0);

            if (gamepad2.right_trigger > 0) {
                Intake.setPower(-gamepad2.right_trigger/2);
            }


            Climbing1.setPower(gamepad2.right_stick_y);
            Climbing2.setPower(gamepad2.right_stick_y);


            //------------------DRIVE TRAIN---------------------------------
            //Driving
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y; // heheheha

            if (gamepad1.right_stick_x > 0.5) {
                motorFL.setPower(gamepad1.right_stick_x);
                motorBL.setPower(-gamepad1.right_stick_x);
                motorFR.setPower(-gamepad1.right_stick_x);
                motorBR.setPower(gamepad1.right_stick_x);
            } else if (gamepad1.left_stick_x < -0.5) {
                motorFL.setPower(gamepad1.left_stick_x);
                motorBL.setPower(-gamepad1.left_stick_x);
                motorFR.setPower(-gamepad1.left_stick_x);
                motorBR.setPower(gamepad1.left_stick_x);
            } else {
                motorFL.setPower(leftPower);
                motorBL.setPower(leftPower);
                motorFR.setPower(rightPower);
                motorBR.setPower(rightPower);
            }

            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());

            telemetry.addData("Slider position",    Sliders.getPower());
            telemetry.addData("Climbing1",    Climbing1.getPower());
            telemetry.addData("Climbing2",    Climbing2.getPower());

            telemetry.addData("top Position:", DropperTop.getPosition());
            telemetry.addData("bottom Position:", DropperBottom.getPosition());

            telemetry.update();
        }
    }
}
