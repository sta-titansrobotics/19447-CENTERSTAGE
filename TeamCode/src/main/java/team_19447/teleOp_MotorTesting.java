package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class teleOp_MotorTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        /*DcMotor placeHolderName1 = hardwareMap.get(DcMotor.class, "placeHolderName1");
        DcMotor placeHolderName2 = hardwareMap.get(DcMotor.class, "placeHolderName2");
        DcMotor placeHolderName3 = hardwareMap.get(DcMotor.class, "placeHolderName3");
        DcMotor placeHolderName4 = hardwareMap.get(DcMotor.class, "placeHolderName4");*/


        DcMotor placeHolderName1 = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor placeHolderName2 = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor placeHolderName3 = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor placeHolderName4 = hardwareMap.get(DcMotor.class, "motorBackRight");


        placeHolderName1.setDirection(DcMotorSimple.Direction.REVERSE);
        placeHolderName2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double rightJoystickY = gamepad1.right_stick_y;

            placeHolderName1.setPower(rightJoystickY);
            placeHolderName2.setPower(rightJoystickY);
            placeHolderName3.setPower(rightJoystickY);
            placeHolderName4.setPower(rightJoystickY);
        }

        telemetry.addData("LF Power:", placeHolderName1.getPower());
        telemetry.addData("LB Power:", placeHolderName2.getPower());
        telemetry.addData("RF Power:", placeHolderName3.getPower());
        telemetry.addData("RB Power:", placeHolderName4.getPower());
        telemetry.update();
    }
    }