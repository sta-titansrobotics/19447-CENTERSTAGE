package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_Drivetrain47 extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

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
            }
/*
            else if(//insert condition     ){
                //moving left straight
            }else if(//insert condition    ){
                //moving forward straight
            }else if(  //insert condition   ){
                //moving forward left (diagonally)
            }else if(//insert condition     ){
                //moving backward straight
            }else if(//insert condition     ){
                //moving backward-left (diagonally)
            }else if(//insert condition     ){
                //moving backward-right (diagonally)
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