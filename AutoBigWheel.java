package org.firstinspires.ftc.teamcode;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name="AutoBigWheel")

public class AutoBigWheel extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor quackwheel = null;
    private DcMotor flipflop = null;
    private CRServo intake = null;
    private Servo intakelift = null;
    private Servo camerapivot = null;
    private CRServo slide = null;

    // **** BlinkLEDs
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    // **** BlinkLEDs
    


    // ******GYRO
    BNO055IMU gyro;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    // ******GYRO

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double INCREMENT   = 0.002;     // amount to slow servo 
    static final double MAX_POS     =  .90;     // Maximum rotational position
    static final double MIN_POS     =  0.10;     // Minimum rotational position
    double  intakeliftPosition = (MAX_POS); 
    double  camerapivotPosition = (.44); 
    boolean bDuckPosition1 = false;
    boolean bDuckPosition2 = false;
    boolean bDuckPosition3 = false;
    double leftPower = 0;
    double rightPower = 0;
    int flipflopPosition = 0;
    // *********  IMAGE
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball","Cube","Duck","Marker"};
    private static final String VUFORIA_KEY ="AQEaZrT/////AAABmZHrtKUrx0CypYFpiQL2+jVaEynEOIKp9gQR7tRECXbWSW69Mue+tG6z2YfMcduKAT9L1pqYGN/BfFjKL3Jugcnqng+mHibF1lwkc0/2Vjo7VGo7SKzFRsu5nTLZb1/mKQjmn2FAbeFDH+beQmzIFXjNC8gY7+lxJ5VCIPETQ4f8RURMzzy0X3TvMgqJTP4EEB89CuPETliQzhS3eVohllcSj9MoQIsf+4eLOZr7O3VCGUWqk8fdi88z7DM8copU3s8RSaRskOYMrjyA6dyyISXNLs1u884MUi9mE+XrlwMzwKI1DGPIVHANeZG8+xQ0TIYyGdxrs95RFhCMc8yFLdE91BLRlOsK5sRouLXzQSDZ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    // *********  IMAGE

    @Override 
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        quackwheel = hardwareMap.get(DcMotor.class, "quackwheel");
        flipflop = hardwareMap.get(DcMotor.class, "flipflop");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakelift = hardwareMap.get(Servo.class, "intakelift");
        slide = hardwareMap.get(CRServo.class, "slide");
        camerapivot = hardwareMap.get(Servo.class, "camera"); 
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        flipflop.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        flipflop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipflop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipflop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakelift.setPosition(intakeliftPosition);
        camerapivot.setPosition(camerapivotPosition);
        
        // **** BlinkLEDs
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriver.setPattern(pattern);
        // **** BlinkLEDs



        // *********  IMAGE
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0/9.0);
        }
        // *********  IMAGE
        // ******GYRO
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        // Set up our telemetry dashboard
        composeTelemetry();
        // ******GYRO


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
        }

        runtime.reset();
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Find Duck
        intakeliftPosition = 0.44;
        intakelift.setPosition(intakeliftPosition);
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        flipflopPosition = 450;
        if (duckSearch()){
            telemetry.addData("Duck Level 1", "Found");
            pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
            blinkinLedDriver.setPattern(pattern);
            flipflopPosition = 0;
        } 
        else {
            camerapivotPosition = .55;
            camerapivot.setPosition(camerapivotPosition);
            if (duckSearch()){
                telemetry.addData("Duck Level 2", "Found");
                pattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(pattern);
                flipflopPosition = 300;
            }
            else {
                camerapivotPosition = .64;
                camerapivot.setPosition(camerapivotPosition);
                if (duckSearch()){
                    telemetry.addData("Duck Level 3", "Found");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
                    blinkinLedDriver.setPattern(pattern);
                }
            }
        }
        telemetry.update();
        blinkinLedDriver.setPattern(pattern);

        intakeliftPosition = 0.12;
        intakelift.setPosition(intakeliftPosition);
        encoderDrive(DRIVE_SPEED, 6, 6, 3);
        //gyroDrive(DRIVE_SPEED, 6.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, 30.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, 30.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        flipflop.setTargetPosition(flipflopPosition);
        flipflop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipflop.setPower(.4);
        //gyroDrive(.4, 16.0, 30.0);  // Drive FWD 12 inches at 45 degrees
        encoderDrive(DRIVE_SPEED, 16, 16, 3);

        intake.setPower(-1);
        sleep(250);
        intakeliftPosition = 0.20;
        intakelift.setPosition(intakeliftPosition);
        sleep(250);
        intakeliftPosition = 0.3;
        intakelift.setPosition(intakeliftPosition);
        sleep(250);
        intakeliftPosition = 0.12;
        intakelift.setPosition(intakeliftPosition);
        sleep(750);
        intakelift.setPosition(.3);
        sleep(250);
        intakelift.setPosition(MAX_POS);
        encoderDrive(DRIVE_SPEED, -9, -9, 3);
        //gyroDrive(.3, -9.0, 30.0);  // Drive FWD 12 inches at 45 degrees
        intake.setPower(0);

        flipflop.setTargetPosition(0);
        flipflop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipflop.setPower(-.2);
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(pattern);

        intakelift.setPosition(MAX_POS);
        gyroTurn( TURN_SPEED, 90.0);     
        gyroHold( TURN_SPEED, 90, 0.5);   
        encoderDrive(DRIVE_SPEED, -27, -27, 4);
        //gyroDrive(.4, -27.0, 90.0);  
        encoderDrive(DRIVE_SPEED, 4, 4, 4);
        //gyroDrive(DRIVE_SPEED, 4.0, 90.0);  
        gyroTurn( TURN_SPEED, 0.0);    
        gyroHold( TURN_SPEED, 0, 0.5); 
        //gyroDrive(.3, -10.0, 0.0);  
        encoderDrive(DRIVE_SPEED, -10, -10, 4);
        pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
        blinkinLedDriver.setPattern(pattern);

        quackwheel.setPower(-.5);
        gyroDrive(DRIVE_SPEED, 1.0, 0.0); 
        gyroTurn( TURN_SPEED, 5.0);  
        gyroHold( TURN_SPEED, 5.0, 0.5); 
        gyroDrive(.3, -1.0, 5.0);  
        gyroHold( TURN_SPEED, 10.0, 1); 
        pattern = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
        blinkinLedDriver.setPattern(pattern);

        sleep(1000);
        quackwheel.setPower(0);
        encoderDrive(DRIVE_SPEED, 23, 23, 5);
        //gyroDrive(DRIVE_SPEED, 23 , -5.0); 


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = backleft.getCurrentPosition() + moveCounts;
            newRightTarget = backright.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            backleft.setTargetPosition(newLeftTarget);
            backright.setTargetPosition(newRightTarget);
            frontleft.setTargetPosition(newLeftTarget);
            frontright.setTargetPosition(newRightTarget);

            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            backleft.setPower(speed);
            backright.setPower(speed);
            frontleft.setPower(speed);
            frontright.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (backleft.isBusy() && backright.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                backleft.setPower(leftSpeed);
                backright.setPower(rightSpeed);
                frontleft.setPower(leftSpeed);
                frontright.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d", backleft.getCurrentPosition(),
                                                                backright.getCurrentPosition(),
                                                                frontleft.getCurrentPosition(),
                                                                frontright.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);

            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        backleft.setPower(leftSpeed);
        backright.setPower(rightSpeed);
        frontleft.setPower(leftSpeed);
        frontright.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        // ******GYRO
        robotError = targetAngle - angles.firstAngle; //- gyro.getIntegratedZValue(); // ******GYRO
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = gyro.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return gyro.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return gyro.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newbackleftTarget;
        int newbackrightTarget;
        int newfrontleftTarget;
        int newfrontrightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackleftTarget = backleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newfrontleftTarget = frontleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);
            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);

            // Turn On RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (backleft.isBusy() && backright.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newbackleftTarget,  newbackrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            backleft.getCurrentPosition(),
                                            backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);

            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    
    private boolean duckSearch() {

        boolean  onDuckFound = false;
        int d = 0;
        while (d < 50) {
    
            if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                  telemetry.addData("# Object Detected", updatedRecognitions.size());
                  // step through the list of recognitions and display boundary info.
                  int i = 0;
                  for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    if ("Duck" == recognition.getLabel()){
                        onDuckFound = true;
                        d = 50;
                    }
                        
                  }
                  d++;
                  telemetry.update();
                }
                
    
            }

        }
        return onDuckFound;
    }
    

}



