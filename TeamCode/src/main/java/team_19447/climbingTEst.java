package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class climbingTEst extends LinearOpMode {

    int buttonX = 0;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {

        Servo DropperTop =  hardwareMap.get(Servo.class, "DropperTop");
        Servo DropperBottom =  hardwareMap.get(Servo.class, "DropperBottom");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.x && timer.seconds() > 0.5) {
                buttonX += 1;
                timer.reset();
            }

            if (buttonX % 4 == 1)
                DropperTop.setPosition(-0.2);
            else if (buttonX % 4 == 2) {
                DropperBottom.setPosition(-0.2);
            } else if (buttonX % 4 == 3){
                DropperTop.setPosition(0);
        } else {
                DropperBottom.setPosition(0);
            }


        /*
        if (gamepad2.x)
            buttonX +=1;

        //Note: the encoder values are placeholders
        if(buttonX%2==1){
            Climbing1.setTargetPosition(5000);
        }else {
            Climbing1.setTargetPosition(2000);
        }*/

            telemetry.addData("Position:", DropperTop.getPosition());
            telemetry.update();


    }}
}