package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_Drivetrain47 extends LinearOpMode {

    int buttonA=0;
    int buttonX=0;
    int buttonY=0;
    int buttonB=0;

    int button2A =0;
    int button2X =0;
    int button2B =0;
    int button2Y =0;

    boolean but2Acheck = false;
    boolean but2Bcheck = false;
    boolean but2Xcheck = false;
    boolean but2Ycheck = false;

    boolean dropping = false;
    //change -6 to how long it takes for the servo to change -1
    //- number to prevent the later if statement from being executed at the start
    int prevtime = -2001;
    @Override
    public void runOpMode() {


        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // Port:1 1st
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft"); // Port:2 1st
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight"); // Port:2 2nd
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight"); // Port:1 2nd

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake"); // --> Done // Port:
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); // --> Done // Port:
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); // for the robot to hang --> Done Port:
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); // for the robot to hang --> Done Port:

        Servo DropperTop =  hardwareMap.get(Servo.class, "DropperTop"); // --> Done Servo Port 0
        Servo DropperBottom =  hardwareMap.get(Servo.class, "DropperBottom"); // --> Done Servo Port 1
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); \\ --> the thing that rotates the dropper Servo Port 2
        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "AirplaneLauncher"); \\ --> Still have to work on //Servo Port 3

        Climbing1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climbing2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        //toggle template, note: its just template, delete after
        if (gamepad2.a && !but2Acheck) {
            button2A += 1;
            but2Acheck = true;
        }
        if (!gamepad2.a){
            but2Acheck = false;
        }
        if (!but2Acheck) {
            if (button2A % 2 == 1) {
                //Intake.setPower(1);
            } else {
                //Intake.setpower(0);
            }
        }

        //wrist
        //the strange calculations are because we need to convert (-1 - 1) into (0 - 1)
        //Wrist.setPosition((gamepad2.left_stick_y*0.5)+0.5);

        //Intake
        /*
        if (gamepad2.a && !but2Acheck){
            button2A +=1;
            but2Acheck = true;
        }
        if (!gamepad2.a)
            but2Acheck = false;
        if (!but2Acheck) {
            if (button2A % 2 == 1) {
                //Intake.setPower(1);
            } else {
                //Intake.setpower(0);
            }
        }
        */

        //Climbing: if gamepad2 X is pressed, the climbing motors will go at full power and push the linear actuators up to a certain positiion
        // If the button X is pressed again, then the actuators will descend to [some certain point in terms of encoder reading] at full power and hang the robot
        if (gamepad2.x && !but2Xcheck) {
            button2X += 1;
            but2Xcheck = true;
        }
        if (!gamepad2.x)
            but2Xcheck = false;

        //Note: the encoder values are placeholders
        if (!but2Xcheck) {
            if (button2X % 2 == 1) {
                //Climbing1.setTargetPosition(5000);
                //Climbing2.setTargetPosition(5000);
            } else {
                //Climbing1.setTargetPosition(2000);
                //Climbing2.setTargetPosition(2000);
            }
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
        if (getRuntime() - prevtime == 2000){
            DropperTop.setPosition(0);
        }
        */

        /*
        if (gamepad2.y && !but2Ycheck) {
            button2Y += 1;
            but2Ycheck = true;
        }
        if (!gamepad2.y){
            but2Ycheck = false;
        }
        if (!but2Ycheck) {
            if (button2Y % 2 == 1) {
                AirplaneLauncher.setPosition(0);
            } else {
                AirplaneLauncher.setPosition(1);
            }
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

            //Basic movement
            if(x>0&&y>0){
                //Moving forward-right (diagonally)
                motorFL.setPower(1);
                motorBR.setPower(1);
            }else if(x>0&&y==0){
                //Moving right straight
                motorFL.setPower(1);
                motorBL.setPower(1);
                motorFR.setPower(-1);
                motorBR.setPower(-1);
            }else if(x<0&&y==0){
                //moving left straight
                motorFL.setPower(-1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(-1);
            }else if(x==0&&y>0){
                //moving forward straight
                motorFL.setPower(1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(1);
            }else if(x<0&&y>0){
                //moving forward left (diagonally)
                motorFR.setPower(1);
                motorBL.setPower(1);
            }else if(x==0&&y<0){
                //moving backward straight
                motorFL.setPower(-1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(-1);
            }else if(x<0&&y<0){
                //moving backward-left (diagonally)
                motorFR.setPower(-1);
                motorBL.setPower(-1);
            }else if(x>0&&y<0){
                //moving backward-right (diagonally)
                motorFL.setPower(-1);
                motorBR.setPower(-1);
            }else{
                //if joystick is not moving, the robot should also stop
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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
