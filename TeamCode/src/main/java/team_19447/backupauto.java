package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class backupauto extends LinearOpMode {

    // Set motor variables
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    @Override
    public void runOpMode() {

        // Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the positions to zero, since the motor has not moved yet


        telemetry.addData("motorFL Encoder Position: ", motorFL.getCurrentPosition());
        telemetry.addData("motorBL Encoder Position: ", motorBL.getCurrentPosition());
        telemetry.addData("motorFR Encoder Position: ", motorFR.getCurrentPosition());
        telemetry.addData("motorBR Encoder Position: ", motorBR.getCurrentPosition());
        telemetry.addData("current power --> BackLeft: " , motorBL.getPower());
        telemetry.addData("current power --> BackRight: " , motorBR.getPower());

        telemetry.update();
        }
    }/*
    {
if (Motor1.getCurrentPosition() < 1) {
        Motor1.setPower(1);
        Motor2.setPower(1);
} else {
        Motor1.setPower(0);
        Motor2.setPower(0)
}
        } else{
        if(Motor1.getCurrentPosition() > 1) {
        Motor1.setPower(-1);
        Motor2.setPower(-1);
        }else{
        Motor1.setPower(0);
        Motor2.setPower(0);
        }
        }
  */