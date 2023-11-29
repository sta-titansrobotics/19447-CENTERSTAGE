package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_TankDrive47 extends LinearOpMode {
    int buttonA=0;
    int buttonX=0;
    int buttonY=0;
    int buttonB=0;

    int button2A =0;
    int button2X =0;
    int button2B =0;
    int button2Y =0;

    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");   --> Done
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); --> Done
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); for the robot to hang --> Done
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); for the robot to hang --> Done

        Servo DroperTop =  hardwareMap.get(Servo.class, "ClawDrop1"); --> Done
        Servo DroperBottom =  hardwareMap.get(Servo.class, "ClawDrop2"); --> Done
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); --> No idea
        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "Airplane Launcher"); --> Still have to work on

        Climbing1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climbing2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        boolean dropping = false;

        //toggle template, note: its just template, delete after
        if (gamepad1.a)
            buttonA +=1;
        if(buttonX%2==1){
            //Intake.setPower(1);
        }else {
            //Intake.setpower(0);
        }

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
            // Climbing1.setTargetPosition(5000);
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
            DropperTop.setPosition(0.5);
            DropperTop.setPosition(0);
            dropping = false;
        }*/

        //Reverse right side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        //ftc documentation https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html
        while (opModeIsActive()) {

            //Driving
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y; // heheheha

            if(gamepad1.right_stick_x > 0.3){
                motorFL.setPower(gamepad1.right_stick_x);
                motorBL.setPower(-gamepad1.right_stick_x);
                motorFR.setPower(-gamepad1.right_stick_x);
                motorBR.setPower(gamepad1.right_stick_x);
            }
            else if(gamepad1.left_stick_x < -0.3){
                motorFL.setPower(gamepad1.left_stick_x);
                motorBL.setPower(-gamepad1.left_stick_x);
                motorFR.setPower(-gamepad1.left_stick_x);
                motorBR.setPower(gamepad1.left_stick_x);
            }else{
                motorFL.setPower(leftPower);
                motorBL.setPower(leftPower);
                motorFR.setPower(rightPower);
                motorBR.setPower(rightPower);
            }

            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.update();

        }
    }
}