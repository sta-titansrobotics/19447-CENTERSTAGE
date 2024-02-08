package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
public class conditionalauto extends LinearOpMode {

    //counts_per_unit_distance = ticks per revolution / wheel_circumference
    // 1425.1/29.92 = 47.63 this is the encoder count for every 1cm travelled
    //public static final double forwardTicks = 47.63;

    //Counts per Unit Lateral Distance = ticks per revolution / Lateral Distance
    //note 49.05 is just a guess
    //public static final double strafeTicks = 49.05;

    void rotate(int direction, DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL){
        //direction dict:
        //0 = counter clock aka, 1 = clock

        if (direction == 0){
            FR.setTargetPosition(FR.getCurrentPosition() + (int) 2279.0955);
            FL.setTargetPosition(FR.getCurrentPosition() - (int) 2279.0955);
            BR.setTargetPosition(FR.getCurrentPosition() + (int) 2279.0955);
            BL.setTargetPosition(FR.getCurrentPosition() - (int) 2279.0955);
        } if (direction == 1){
            FR.setTargetPosition(FR.getCurrentPosition() - (int) 2279.0955);
            FL.setTargetPosition(FR.getCurrentPosition() + (int) 2279.0955);
            BR.setTargetPosition(FR.getCurrentPosition() - (int) 2279.0955);
            BL.setTargetPosition(FR.getCurrentPosition() + (int) 2279.0955);
        }
    }

    void strafe(int cm, int direction, DcMotor FR, DcMotor FL, DcMotor BR, DcMotor BL){
        //direction dict:
        //0 = forward, 1 = right, 2 = backwards, 3 = left
        //cm is distance to travel in cm

        cm *= 47.63;
        
        if (direction == 0){
            FR.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            FL.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            BR.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            BL.setTargetPosition(FR.getCurrentPosition() + (int) cm);
        } if (direction == 1){
            FR.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            FL.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            BR.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            BL.setTargetPosition(FR.getCurrentPosition() - (int) cm);
        } if (direction == 2){
            FR.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            FL.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            BR.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            BL.setTargetPosition(FR.getCurrentPosition() - (int) cm);
        } if (direction == 3){
            FR.setTargetPosition(FR.getCurrentPosition() + (int) cm);
            FL.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            BR.setTargetPosition(FR.getCurrentPosition() - (int) cm);
            BL.setTargetPosition(FR.getCurrentPosition() + (int) cm);
        }
    }

    @Override
    public void runOpMode() {

        //HARDWARE MAPPING
        DcMotor FL = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // Expansion hub 3
        DcMotor BL = hardwareMap.get(DcMotor.class, "motorBackLeft"); // Expansion hub 2
        DcMotor FR = hardwareMap.get(DcMotor.class, "motorFrontRight"); // Control hub 3
        DcMotor BR = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        DcMotor Intake = hardwareMap.get(DcMotor.class, "Intake");   --> Done
        DcMotor Sliders = hardwareMap.get(DcMotor.class, "Sliders"); --> Done
        DcMotor Climbing1 = hardwareMap.get(DcMotor.class, "Climbing1"); for the robot to hang --> Done
        DcMotor Climbing2 = hardwareMap.get(DcMotor.class, "Climbing2"); for the robot to hang --> Done

        Servo DropperTop =  hardwareMap.get(Servo.class, "DropperTop"); --> Done   //Servo Port 0
        Servo DropperBottom =  hardwareMap.get(Servo.class, "DropperBottom"); --> Done //Servo Port 1
        Servo Wrist =  hardwareMap.get(Servo.class, "Wrist"); --> the thing that rotates the dropper //Servo Port 2
        Servo AirplaneLauncher = hardwareMap.get(Servo.class, "AirplaneLauncher"); --> Still have to work on //Servo Port 3

        Climbing1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climbing2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //60 forward, 60 left, left turn 90,

        strafe(60,0, FR, FL, BR, BL);

        rotate(0, FR, FL, BR, BL);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("encoder-front-left: ", FL.getCurrentPosition());
            telemetry.addData("encoder-rear-left: ", BL.getCurrentPosition());
            telemetry.addData("encoder-front-right: ", FR.getCurrentPosition());
            telemetry.addData("encoder-rear-right: ", BR.getCurrentPosition());
            telemetry.update();
        }
    }
}