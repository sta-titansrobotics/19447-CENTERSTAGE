package team_19447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PIDTesting extends LinearOpMode {

    // Set motor variables
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    // Initializing encoder positions
    public int leftPos1;
    public int leftPos2;
    public int rightPos1;
    public int rightPos2;

    public static final int TICKS_PER_REVOLUTION = 1440; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_INCHES = 4.0; // Replace with your wheel diameter
    public static final double DRIVE_SPEED = 0.5; // Adjust the speed as needed
    public static final int TARGET_DISTANCE_INCHES = 12; // Replace with your target distance
    public static final double RADIUS = 1.875;

    /*
     * HOW TO ADJUST THE CONSTANT VALUES
     * Set all gains to 0.
     * Increase Kd until the system oscillates.
     * Reduce Kd by a factor of 2-4.
     * Set Kp to about 1% of Kd.
     * Increase Kp until oscillations start.
     * Decrease Kp by a factor of 2-4.
     * Set Ki to about 1% of Kp.
     * Increase Ki until oscillations start.
     * Decrease Ki by a factor of 2-4.
     */
    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0.01;
    double Kf = 0.2;

    ElapsedTime timer = new ElapsedTime();
    public static double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //note this part is for testing the values remove later and set these as constants
        if(gamepad1.a)
            Kd += 0.01;
        if(gamepad1.b)
            Kd -= 0.01;
        if(gamepad1.x)
            Kp += 0.05;
        if(gamepad1.y)
            Kp -= 0.01;

        // Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        // set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the positions to zero, since the motor has not moved yet
        leftPos1 = 0;
        leftPos2 = 0;
        rightPos1 = 0;
        rightPos2 = 0;

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // any code after this command will not be executed until the match has started
        waitForStart();
        timer.reset();

        // can now set drive distance because of the function below; now we just need to
        // input the distance
        // can also control the direction using the mecanum drivetrain directions here:
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        // Starting position with robot right side
        Drive(50, 50, 50, 50);

        // PIDDrive(250, 250, 250, 250);

        while (opModeIsActive()) {
            telemetry.addData("motorFL Encoder Position: ", motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ", motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ", motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ", motorBR.getCurrentPosition());
            telemetry.addData("Kd: " , Kd);
            telemetry.addData("Kp: " , Kp);
            telemetry.update();
        }
    }

    // these parameters are distance in CM
    public void Drive(int TargetPositionMotorFL, int TargetPositionMotorBL, int TargetPositionMotorFR,
                      int TargetPositionMotorBR) {

        // this is in terms of cm
        TargetPositionMotorFL = (int) (47.63 * TargetPositionMotorFL);
        TargetPositionMotorBL = (int) (47.63 * TargetPositionMotorBL );
        TargetPositionMotorFR = (int) (47.63 * TargetPositionMotorFR );
        TargetPositionMotorBR = (int) (47.63 * TargetPositionMotorBR);

        motorFL.setTargetPosition(TargetPositionMotorFL);
        motorFR.setTargetPosition(TargetPositionMotorFR);
        motorBL.setTargetPosition(TargetPositionMotorBL);
        motorBR.setTargetPosition(TargetPositionMotorBR);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(PIDControl(TargetPositionMotorFL, motorFL.getCurrentPosition()));
        motorBL.setPower(PIDControl(TargetPositionMotorBL, motorBL.getCurrentPosition()));
        motorFR.setPower(PIDControl(TargetPositionMotorFR, motorFR.getCurrentPosition()));
        motorBR.setPower(PIDControl(TargetPositionMotorBR, motorBR.getCurrentPosition()));

        // Run to target position
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach the target position
        while (opModeIsActive() && motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) {
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }

    // calculates the power which the motor should be set at.
    public double PIDControl(double setPosition, double currentPosition) {
        double error = setPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (setPosition * Kf);
    }
}
