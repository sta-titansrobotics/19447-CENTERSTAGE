package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//so far the newest file as of Jan 9 2024
@TeleOp
public class AnotherDriveTrain47 extends LinearOpMode {

    int buttonA=0;
    int isClimbing=0;
    double wristpower = 0;
    int isSliding = 0 ;
    int buttonX=0;

    int button2A =0;
    boolean but2Acheck = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); //Expansion hub 3
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");  //Expansion hub 2
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");  //Control hub 3
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight"); //Control hub 2


        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); //  Expansion hub 1
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders");   // Control hub 0
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); // left    Control hub 1
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2");// right     Expansion hub 0

        Servo Wrist = hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 5
        Servo Launcher = hardwareMap.get(Servo.class, "Launcher"); // control hub servo port 4

        //Climbing1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Climbing1.setDirection(DcMotorSimple.Direction.REVERSE);

        Climbing1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climbing2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Climbing2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Climbing2.setDirection(DcMotorSimple.Direction.REVERSE);

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
                Intake.setPower(-gamepad2.right_trigger/3);
            }

            /*Climbing:  mapped to right joystick power
            Climbing1.setPower(gamepad2.right_stick_y);
            Climbing2.setPower(gamepad2.right_stick_y);*/

            //better climbing
            if (gamepad2.a && !but2Acheck) {
                button2A += 1;
                but2Acheck = true;
            }
            if (!gamepad2.a){
                but2Acheck = false;
            }
            if (!but2Acheck) {
                if (button2A % 2 == 1) {
                    if (Climbing1.getCurrentPosition() < 5000) { //replace 5000 with value when fully extended
                        Climbing1.setPower(1);
                        Climbing2.setPower(1);
                    } else {
                        Climbing1.setPower(0);
                        Climbing2.setPower(0);
                    }
                } else {
                    if (Climbing1.getCurrentPosition() > 5000) {
                        Climbing1.setPower(-1);
                        Climbing2.setPower(-1);
                    } else {
                        Climbing1.setPower(0);
                        Climbing2.setPower(0);
                    }
                }
            }


            //Slider
            if (Sliders.getCurrentPosition() > 7500)
                isSliding = 0;
            if (gamepad1.right_bumper) {
                isSliding = 1;
            }

            if (isSliding==1 && Sliders.getCurrentPosition() < 7500) {
                Sliders.setPower(1);
            } else if (gamepad1.right_trigger > 0 && Sliders.getCurrentPosition() > 100) {
                Sliders.setPower(-gamepad1.right_trigger);
            } else {
                Sliders.setPower(0);
            }

            //wrist
            //the strange calculations are because we need to convert (-1 - 1) into (0 - 1)
            if (gamepad1.left_bumper) {
                Wrist.setPosition(Wrist.getPosition() + 0.0005);
            }
            if (gamepad1.left_trigger > 0.5){
                Wrist.setPosition(Wrist.getPosition() - 0.0005);
            }
            //------------------DRIVE TRAIN---------------------------------
            //Driving
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y; // heheheha

            if (gamepad1.right_stick_x > 0.7) {
                motorFL.setPower(gamepad1.right_stick_x);
                motorBL.setPower(-gamepad1.right_stick_x);
                motorFR.setPower(-gamepad1.right_stick_x);
                motorBR.setPower(gamepad1.right_stick_x);
            } else if (gamepad1.left_stick_x < -0.7) {
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

            telemetry.update();
        }
    }
}
