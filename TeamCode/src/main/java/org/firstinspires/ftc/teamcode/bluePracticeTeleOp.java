package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Config
@TeleOp(name = "blue far practice teleop")
public class bluePracticeTeleOp extends LinearOpMode {
    // Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotorEx Indexer = null;
    private DcMotorEx Flywheel = null;
    private DcMotorEx turret = null;
    private Servo leftKicker = null;
    private Servo rightKicker = null;

    private Servo Pitch = null;
    Limelight3A limelight;
    private DigitalChannel laserInput;
    private GoBildaPinpointDriver pinpoint = null;
    // Constants
    // indexer

    int shootCounter = 0;
    int intakeCounter = 1;
    int BallOneShoot = 90;
    int BallTwoShoot = 90+180;
    int ballThreeShoot = 90+360;

    int ballOneIntake = 0;
    int ballTwoIntake = ballOneIntake + 180;
    int ballThreeIntake = ballTwoIntake + 180;

    int TargetPosition = ballOneIntake;
    int[] ordera = { 2, 1, 1 };
    int[] orderb = { 1, 2, 1 };
    int[] orderc = { 1, 1, 2 };
    int[] targetOrder = orderc;
    int ballOneColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballTwoColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballThreeColor = 0;// 0 acts as null 1 is purple 2 is green

    boolean ballOneCounted = false;
    boolean ballTwoCounted = false;
    boolean ballThreeCounted = false;

    boolean purple = false;
    boolean green = false;
    int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
    boolean lastintake = true;
    // constants for indexer
    private ColorSensorV3 colorSensor;

    // declared like this so if motors are swapped I only have to find one value
    private PIDControllerRyan indexerPID = null;
    private PIDControllerRyan turretPID = null;
    private PIDTuner pidTuner = null;


    static double TargetX = 0;// find this

    double TargetAngle = 0;

    public static double VF = 0;
    int targetID = 0;
    double tx = 0;
    double ty = 0;

    double ta = 0;
    private int offset = 0;
    private final int turretmaxl = 1087 + offset;
    private final int turretmaxr = 0 + offset;


    private double tpr = 537.7;

    private double x = 0;
    private double y = 0;
    private double heading = 0;
    public boolean detected;
    public boolean lastDetected;

    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,48, 8, AngleUnit.DEGREES,90);//used to save position after autonomous
    boolean autoIntake = false;
    private double distanceToTarget;
    private double targetx = 0;
    private double targety = 144;
    private double xV;
    private double yV;
    private double netV;
    private double hV;
    private double targetangle;
    private double relTargetangle;
    private double TURRET_TICKS_PER_RADIAN = 537.7/(Math.PI*2);

    private int turretTargetPosition = 0;
    private double inpower;

    private double kp = 0;

    private double kf = 0;

    private double amount = 1;
    FlywheelPIDController pids = null;

    @Override
    public void runOpMode() {
        laserInput = hardwareMap.get(DigitalChannel.class, "laserInput");

        // Lime Light
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);

        //imu
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(3.425,6.424,DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(currentPose);

        // base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        Indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        Pitch = hardwareMap.get(Servo.class, "Pitch");
        leftKicker.setDirection(Servo.Direction.FORWARD);// 0 is min angle 1 is max angle
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Indexer.setTargetPosition(TargetPosition);
        Indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Indexer.setPositionPIDFCoefficients(10);
        Indexer.setPower(1);

        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setPower(-.5);
        sleep(2000);
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPositionPIDFCoefficients(20);
        turret.setPower(1);

        laserInput.setMode(DigitalChannel.Mode.INPUT);
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftKicker.setPosition(0.01);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(700,0,0,17);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,flyhweelconts);
        Thread KickerThread = new Thread(this::Operations);


        waitForStart();


        KickerThread.start();

        while (opModeIsActive()) { // Loop
            pinpoint.update();
            x = pinpoint.getPosition().getX(DistanceUnit.INCH);
            y = pinpoint.getPosition().getY(DistanceUnit.INCH);
            heading = pinpoint.getHeading(AngleUnit.RADIANS);
            distanceToTarget = Math.sqrt(Math.pow(x - targetx, 2) + Math.pow(y - targety, 2));
            Calculate(distanceToTarget);
            xV = pinpoint.getVelX(DistanceUnit.INCH);
            yV = pinpoint.getVelY(DistanceUnit.INCH);
            netV = Math.sqrt(Math.pow(xV, 2) + Math.pow(yV, 2));
            hV = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            targetangle = Math.atan2(targety - y, targetx - x);
            relTargetangle = targetangle - heading;
            relTargetangle = Math.atan2(Math.sin(relTargetangle), Math.cos(relTargetangle));

// Shift so forward = pi/2
            double turretAngle = relTargetangle + (Math.PI / 2.0);

// Clamp to turret range
            turretAngle = Math.max(0.0, Math.min(Math.PI, turretAngle));

// Convert to ticks
            turretTargetPosition = (int)(turretAngle * TURRET_TICKS_PER_RADIAN*4) + offset;
            // Clamp the target position to within the physical limits of the turret
            turretTargetPosition = Math.max(turretmaxr,
                    Math.min(turretmaxl, turretTargetPosition));


            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3); // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Intake
            if (gamepad1.right_trigger > 0.01) {
                Intake.setPower(1);
            } else {
                Intake.setPower(0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                intake();
            }

            if (gamepad1.dpadRightWasPressed()) {
                shoot();
            }
            if (gamepad1.dpadLeftWasPressed()) {
                reverseIntake();
            }
            if (gamepad1.dpadDownWasPressed()) {
                reverseShoot();
            }
            if(distanceToTarget>125){
                Pitch.setPosition(0);
            }
            else{
                Pitch.setPosition(1);
            }
            if(gamepad2.leftBumperWasPressed()){
                offset+=5;
            }
            if(gamepad2.rightBumperWasPressed()){
                offset-=5;
            }




            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.
            telemetry.addData("Laser", detected);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Indexer power", Indexer.getPower());
            telemetry.addData("Indexer power", inpower);
            telemetry.addData("Indexer Position", "%d",
                    Indexer.getCurrentPosition());
            telemetry.addData("Indexer Target Position", "%d",
                    TargetPosition);
            // colorSensor.addTelemetry();
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("H", heading);
            telemetry.addData("distance to target",distanceToTarget);
            telemetry.addData("Target Turret", turretTargetPosition);
            telemetry.addData("Turret pos", turret.getCurrentPosition());
            telemetry.addData("Ball one pos", ballOneIntake);
            telemetry.addData("Ball two pos ", ballTwoIntake);
            telemetry.addData("Ball three pos", ballThreeIntake);
            telemetry.addData("Current Order", targetOrder[0]);
            telemetry.addData("Current Order", targetOrder[1]);
            telemetry.addData("Current Order", targetOrder[2]);
            telemetry.addData("Flywheel Target RPM", VF);
            telemetry.addData("distance", TargetX);
            telemetry.addData("ty", ty);
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("ll status", limelight.isConnected());
            telemetry.update();
        }
    }

    public void PinpointLoop(){
        while (opModeIsActive()) {
            pinpoint.update();
            sleep(50);
        }

    }
    public void Calculate(double dih){
        if(dih<50){
            VF=1000;
        }
        else if(dih<120){
            VF= 4.95027*dih+670.39441;
        }
        else{
            VF = 4.96245*dih+684.17756;
        }
    }
    private void intake(){
        intakeCounter++;
        if(intakeCounter == 4){
            intakeCounter=1;
        }
        if (!lastintake) {
            lastintake = true;
            intakeCounter=1;
            TargetPosition = ballOneIntake;

        } else {
            if(intakeCounter == 1){
                TargetPosition = ballOneIntake;
            }
            if(intakeCounter==2){
                TargetPosition = ballTwoIntake;
            }
            if(intakeCounter==3){
                TargetPosition = ballThreeIntake;
            }

        }

    }
    private void reverseIntake(){
        intakeCounter--;
        if(intakeCounter == 0){
            intakeCounter=3;
        }
        if (!lastintake) {
            lastintake = true;
            intakeCounter=1;
            TargetPosition = ballOneIntake;

        } else {
            if(intakeCounter == 1){
                TargetPosition = ballOneIntake;
            }
            if(intakeCounter==2){
                TargetPosition = ballThreeIntake;
            }
            if(intakeCounter==3){
                TargetPosition = ballTwoIntake;
            }

        }

    }
    private void shoot(){
        shootCounter++;
        if(shootCounter == 4){
            shootCounter=1;
        }
        if (lastintake) {
            lastintake = false;
            shootCounter=1;
            TargetPosition = BallOneShoot;
        } else {
            if(shootCounter == 1){
                TargetPosition = ballOneIntake;
            }
            if(shootCounter==2){
                TargetPosition = BallTwoShoot;
            }
            if(shootCounter==3){
                TargetPosition = ballThreeShoot;
            }
        }
    }
    private void reverseShoot(){
        shootCounter--;
        if(shootCounter == 0){
            shootCounter=3;
        }
        if (lastintake) {
            lastintake = false;
            shootCounter=1;
            TargetPosition = BallTwoShoot;
        } else {
            if(shootCounter == 1){
                TargetPosition = BallTwoShoot;
            }
            if(shootCounter==3){
                TargetPosition = BallOneShoot;
            }
            if(shootCounter==2){
                TargetPosition = ballThreeShoot;
            }
        }
    }
    // Dedicated method for the Limlight;



    private void Operations() {
        while (opModeIsActive()) {
            detected = laserInput.getState();
            if (lastDetected && !detected) {
                sleep(1000);
                intake();
            }
            lastDetected = detected;
            if (gamepad1.aWasPressed()) {
                leftKicker.setPosition(.3);
                sleep(800);
                leftKicker.setPosition(.01);
            }
            Indexer.setTargetPosition(TargetPosition);
            turret.setTargetPosition(turretTargetPosition);
            Flywheel.setVelocity(VF);
            sleep(70);
        }
    }



    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
