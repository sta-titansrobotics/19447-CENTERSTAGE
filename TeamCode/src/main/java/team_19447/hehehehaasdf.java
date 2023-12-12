package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class hehehehaasdf extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo Wrist = hardwareMap.get(Servo.class, "Wrist"); // control hub servo port 0
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //wrist
            //the strange calculations are because we need to convert (-1 - 1) into (0 - 1)
            if (gamepad1.left_bumper) {
                Wrist.setPosition(Wrist.getPosition() + 0.0005);
            }
            if (gamepad1.left_trigger > 0.5){
                Wrist.setPosition(Wrist.getPosition() - 0.0005);
            }


        }
    }
}