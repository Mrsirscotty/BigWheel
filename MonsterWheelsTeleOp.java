package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MonsterWheelsTeleOp", group="Linear Opmode")

public class MonsterWheelsTeleOp extends LinearOpMode {

    // Declare OpMode members.
    MonsterWheelsHW robotHW   = new MonsterWheelsHW();   // Use a robotHW's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double INCREMENT   = 0.002;     // amount to slow servo 
    static final double CLAW_MAX_POS     =  1.0;     // Maximum rotational position
    static final double CLAW_MIN_POS     =  0.0;     // Minimum rotational position

    static final double HAND_MAX_POS     = 0.90;     // Maximum rotational position
    static final double HAN_MIN_POS     =  0.10;     // Minimum rotational position

    int drivePowerAdjustment = 1;
    double leftPower = 0;
    double rightPower = 0;
    double ArmPower = 0;
    double slidePower = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robotHW.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (0<gamepad1.right_trigger) {
                robotHW.arm.setPower(gamepad1.right_trigger);
            }
            else if (0<gamepad1.left_trigger) {
                robotHW.arm.setPower(-gamepad1.left_trigger);
            }
            else {
                robotHW.arm.setPower(0);
            }
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ; 
            // Send calculated power to wheels
            robotHW.leftFdrive.setPower(leftPower);
            robotHW.leftRdrive.setPower(leftPower);
            robotHW.rightFdrive.setPower(rightPower);
            robotHW.rightRdrive.setPower(rightPower);

            if (gamepad1.a)
                robotHW.slide.setPower(1);
            else if (gamepad1.b) 
                robotHW.slide.setPower(-1);
            else 
                robotHW.slide.setPower(0);



            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
