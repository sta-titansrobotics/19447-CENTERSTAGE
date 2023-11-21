package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_Drivetrain47 extends LinearOpMode {

    int buttonA=0;
    int button2A =0;
    int button2X =0;
    int buttonX=0;
    int buttonY=0;
    int buttonB=0;
    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders");
        DcMotor Climbing = hardwareMap.get(DcMotor.class, "Climbing");
        Servo ClawDrop1 =  hardwareMap.get(Servo.class, "ClawDrop1");
        Servo ClawDrop2 =  hardwareMap.get(Servo.class, "ClawDrop2");
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist");
        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "Airplane Launcher");*/

        //intake
        if (gamepad1.a)
            buttonA += 1;
        if (buttonA % 2 == 1) {
            //Intake.setPower(1);
        } else {
            //Intake.setpower(0);
        }

        //Climbing
        if (gamepad2.a)
            buttonX += 1;
        if (buttonX % 2 == 1) {
            //Intake.setPower(1);
        } else {
            //Intake.setpower(0);
        }

        if (gamepad1.a)
            buttonY += 1;
        if (buttonY % 2 == 1) {
            //Intake.setPower(1);
        } else {
            //Intake.setpower(0);
        }

        //slider

        if (gamepad1.a)
            buttonB += 1;
        if (buttonB % 2 == 1) {
            //Intake.setPower(1);
        } else {
            //Intake.setpower(0);
        }

        //ClawDrop1
        if (gamepad2.a)
            button2A += 1;
        if (button2A % 2 == 1) {
            //ClawDrop1.setPower(1);
        } else {
            //ClawDrop1.setpower(0);

            //ClawDrop2
            if (gamepad2.a)
                button2X += 1;
            if (button2X % 2 == 1) {
                //ClawDrop2.setpower(1);
            } else {
                //ClawDrop2.setpower(0);

                //ServoWrist

                //AirplaneLauncher

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

                    //Basic movements
            /*
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
            }*/

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

