package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Christmas_Promotions extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor ClockRotation = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor Lights = hardwareMap.get(DcMotor.class, "motorBackLeft");
        int pressed = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double rightJoystickY = gamepad1.right_stick_y;
            ClockRotation.setPower(rightJoystickY);


            if(gamepad1.a){
                pressed+=1;
            }
            if(pressed%2==1){
                Lights.setPower(1);
            }else{
                Lights.setPower(0);
            }
        }
        telemetry.addData("Clock: ", ClockRotation.getPower());
        telemetry.addData("Lights: ", Lights.getPower());
        telemetry.update();
        }
    }
