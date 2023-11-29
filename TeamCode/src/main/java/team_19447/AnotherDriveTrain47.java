package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class AnotherDriveTrain47 extends LinearOpMode {

    int buttonA=0;
    int buttonX=0;
    int buttonY=0;
    int buttonB=0;

    int button2A =0;
    int button2X =0;
    int button2B =0;
    int button2Y =0;

    boolean dropping = false;
    //change -6 to how long it takes for the servo to change -1
    //- number to prevent the later if statement from being executed at the start
    int prevtime = -6;
    @Override
    public void runOpMode() {


        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); --> Done
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

        //toggle template, note: its just template, delete after
        if (gamepad1.a)
            buttonA +=1;
        if(buttonX%2==1){
            //Intake.setPower(1);
        }else {
            //Intake.setpower(0);
        }
        //wrist
        //the strange calculations are because we need to convert (-1 - 1) into (0 - 1)
        //Wrist.setPosition((gamepad2.left_stick_y*0.5)+0.5);

        //Intake
        /*
        if (gamepad1.a)
            buttonA +=1;
        if(buttonA%2==1){
            Intake.setPower(1);
        }else{
            Intake.setpower(0);
        }
        */

        //Climbing: if gamepad2 X is pressed, the climbing motors will go at full power and push the linear actuators up to a certain positiion
        // If the button X is pressed again, then the actuators will descend to [some certain point in terms of encoder reading] at full power and hang the robot
        if (gamepad2.x)
            button2X +=1;

        //Note: the encoder values are placeholders
        if(buttonX%2==1){
            //Climbing1.setTargetPosition(5000);
            //Climbing2.setTargetPosition(5000);
        }else {
            //Climbing1.setTargetPosition(2000);
            //Climbing2.setTargetPosition(2000);
        }

        //dropper
        //drops the bottom slot then waits until button is not pressed
        //when button is not pressed load the top slot into the bottom slot

        /*
        if (gamepad2.b){
            DropperBottom.setPosition(0.5);
            dropping = true;
        }
        if (!gamepad2.b && dropping) {
            DropperBottom.setPosition(0);
            prevtime = getRuntime();
            DropperTop.setPosition(0.5);
            dropping = false;
        }
        //change 5 to however long it takes for the servo to move into place
        if (getRuntime() - prevtime == 5){
            DropperTop.setPosition(0);
        }
        */

        //Reverse right side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        //ftc documentation https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html
        while (opModeIsActive()) {

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
            telemetry.update();
        }
    }
}
