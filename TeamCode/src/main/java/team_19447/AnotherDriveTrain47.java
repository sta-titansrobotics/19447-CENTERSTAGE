package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class AnotherDriveTrain47 extends LinearOpMode {

    int buttonA=0;
    int isClimbing=0;
    double wristpower = 0;
    int isSliding = 0 ;
    int buttonX=0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();


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

        Servo DropperTop =  hardwareMap.get(Servo.class, "DropperTop"); //  control hub  servo port 1

        Servo DropperBottom =  hardwareMap.get(Servo.class, "DropperBottom"); // control hub servo port 2
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 0



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
        if (gamepad1.a)
            buttonA +=1;
        if(buttonA%2==1){
            Intake.setPower(0.7);
        }else{
            Intake.setPower(0);
        }

        //Slider
            if(Sliders.getCurrentPosition() > 7500)
                isSliding = 0;
            if(gamepad1.right_bumper&&timer2.seconds()>0.5){
                isSliding += 1;
                timer2.reset();
            }
            if (isSliding%2==1 && Sliders.getCurrentPosition() < 7500)
            {
                Sliders.setPower(1);
            }else if (gamepad1.right_trigger > 0 && Sliders.getCurrentPosition() > 100)
            {
                Sliders.setPower(-gamepad1.right_trigger);
            }
            else{
                Sliders.setPower(0);
            }


        //Climbing:  mapped to right joystick power

                Climbing1.setPower(gamepad2.right_stick_y);
                Climbing2.setPower(gamepad2.right_stick_y);

        //dropper
        //drops the bottom slot then waits until button is not pressed
        //when button is not pressed load the top slot into the bottom slot
        if (gamepad2.x && timer.seconds() > 0.5) {
                buttonX += 1;
                timer.reset();
        }

        //Note: top and bottom servo have wierd ranges as the servo programmer
            // was bugging so range had to be adjusted to whatever range I can set it to
            // update: ignore?
        if (buttonX % 4 == 1)
            DropperTop.setPosition(1);
        else if (buttonX % 4 == 2) {
            DropperBottom.setPosition(1);
        } else if (buttonX % 4 == 3){
            DropperTop.setPosition(0);
        } else {
            DropperBottom.setPosition(0);
        }

            //wrist
            //the strange calculations are because we need to convert (-1 - 1) into (0 - 1)
            if(gamepad1.left_bumper)
                wristpower += 0.02;
            if(gamepad1.right_trigger>0.5)
                wristpower -= 0.02;

            wristpower = Range.clip(wristpower, 0, 1);

            Wrist.setPosition(wristpower);

        //------------------DRIVE TRAIN---------------------------------
            //Driving
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            //STRAFING VARIABLE
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing (moving from side to side)

            //THIS IS THE TURNING VARIABLE
            double rx = gamepad1.right_stick_x;

            //To prevent stick drift
            if(Math.abs(y)<0.1)
                y=0;
            if(Math.abs(x)<0.1)
                x=0;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            denominator *= 1.2; //complaints of motors being too sensitive so turning down the speed.
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);

            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());

            telemetry.addData("Slider position", Sliders.getCurrentPosition());
            telemetry.addData("Climbing1", Climbing1.getCurrentPosition());
            telemetry.addData("Climbing2", Climbing2.getCurrentPosition());
            telemetry.addData("top Position:", DropperTop.getPosition());
            telemetry.addData("bottom Position:", DropperBottom.getPosition());

            telemetry.update();
        }
    }
}
