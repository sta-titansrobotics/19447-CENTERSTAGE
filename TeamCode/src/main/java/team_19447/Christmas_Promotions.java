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
        int clockmode = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a){
                Lights.setPower(1);
            }
            if(gamepad1.b){
                Lights.setPower(0);
            }
            if(gamepad1.x){
                clockmode= 1;
            }
            if(gamepad1.y) {
                clockmode = 0;
            }

            if(clockmode%2==1){
                ClockRotation.setPower(0.15);
            }
            else{
                ClockRotation.setPower(gamepad1.right_stick_y);
            }

        }
        telemetry.addData("Clock: ", ClockRotation.getPower()); telemetry.addData("Lights: ", Lights.getPower());
        telemetry.update();
    }
}
