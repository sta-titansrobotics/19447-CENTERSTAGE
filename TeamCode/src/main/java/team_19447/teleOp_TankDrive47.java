package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_TankDrive47 extends LinearOpMode {

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
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist");*/

        //Reverse right side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        //ftc documentation https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html
        while (opModeIsActive()) {

            //Driving
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y; // Remember, this is reversed

            motorFL.setPower(leftPower);
            motorBL.setPower(leftPower);
            motorFR.setPower(rightPower);
            motorBR.setPower(rightPower);

            if(gamepad1.right_stick_x > 0.1||gamepad1.left_stick_x < -0.1){
                motorFL.setPower(gamepad1.right_stick_x);
                motorBL.setPower(-gamepad1.right_stick_x);
                motorFR.setPower(-gamepad1.right_stick_x);
                motorBR.setPower(gamepad1.right_stick_x);
            }

            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.update();

        }
    }
}
//f