package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousSpecimen_new", group="Robot")
public class AutonomousSpecimen_new extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor LinearSlideLeft = null;
    private DcMotor LinearSlideRight = null;
    private DcMotor foldable = null;
    private CRServo claw=null;
    private Servo joint=null;
    private Servo basket=null;
    private DcMotor encoderLeft=null;
    private DcMotor encoderRight=null;
    private DcMotor encoderBack=null;
    private DcMotor encoderFold=null;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        LinearSlideLeft = hardwareMap.get(DcMotor.class, "LSL");
        LinearSlideRight = hardwareMap.get(DcMotor.class, "LSR");
        foldable = hardwareMap.get(DcMotor.class, "foldable");
        joint = hardwareMap.get(Servo.class, "joint");
        claw = hardwareMap.get(CRServo.class, "claw");
        basket = hardwareMap.get(Servo.class, "basket");
        encoderLeft = leftFrontDrive;
        encoderRight = rightFrontDrive;
        encoderBack = LinearSlideRight;
        encoderFold = foldable;
        final double RADIUS = 4.375;
        final double RADIUS_INCHES = 1.771655;

        final double TICKS = 8192;
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        updateOrientation();
        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        foldable.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        int initial_positionLeft = encoderLeft.getCurrentPosition();
        int initial_positionRight = encoderRight.getCurrentPosition();
        int initial_positionBack = encoderRight.getCurrentPosition();
        int initial_positionFold = encoderFold.getCurrentPosition();

        double position = 0;
        claw.setPower(-1);
        double positionfold = encoderFold.getCurrentPosition() - initial_positionFold;
        while (Math.abs(positionfold) < 2200) {
            positionfold = encoderFold.getCurrentPosition() - initial_positionFold;
            foldable.setPower(.3);
        }
        foldable.setPower(-0.07);
        joint.setPosition(.2);
        moveForward(24,.5);
        while (Math.abs(positionfold) > 2400) {
            positionfold = encoderFold.getCurrentPosition() - initial_positionFold;
            foldable.setPower(-.3);
        }
        sleep(1000);
        joint.setPosition(.24);
        leftFrontDrive.setPower(-.2);
        rightFrontDrive.setPower(-.2);
        leftBackDrive.setPower(-.2);
        rightBackDrive.setPower(-.2);
        waits(3000);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        claw.setPower(1);
        waits(400);
        claw.setPower(0);
        moveForward(3,-.5);
















    }
    private void waits(long miliseconds){
        long starttime = System.currentTimeMillis();
        long timepassed = System.currentTimeMillis() - starttime;
        while((System.currentTimeMillis() - starttime)<miliseconds){

        }
    }
    private void openClaw(){
        claw.setPower(-.5);
        waits(1000);
        claw.setPower(0);
    }
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }
    public void moveForward(double inches,double power){
        int initial_positionLeft = encoderLeft.getCurrentPosition();
        int initial_positionRight = encoderRight.getCurrentPosition();
        int initial_positionBack = encoderRight.getCurrentPosition();
        int positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
        int positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
        int positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
        double positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
        double positionInRight = (positionRight* Math.PI*1.25984)/2000;
        double positionInBack = (positionBack* Math.PI*1.25984)/2000;
        while (Math.abs(positionInLeft)<inches){
            positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
            positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
            positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
            positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
            positionInRight = (positionRight* Math.PI*1.25984)/2000;
            positionInBack = (positionBack* Math.PI*1.25984)/2000;
            telemetry.addData("Encoder left/Right/Back", "%d, %d, %d", positionLeft, positionRight,positionBack);
            telemetry.addData("Inches left/Right/Back", "%4.2f, %4.2f, %4.2f", positionInLeft, positionInRight,positionInBack);
            telemetry.update();
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
    public  void turntime_right(double power, long miliseconds){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        waits(miliseconds);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public  void turntime_left(double power, long miliseconds){
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        waits(miliseconds);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void slide_down(int x) {

        // reset encoder counts kept by motors.
        LinearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        LinearSlideLeft.setTargetPosition(x);
        LinearSlideRight.setTargetPosition(x);

        LinearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LinearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LinearSlideLeft.setPower(-1);
        LinearSlideRight.setPower(1);
        while (opModeIsActive() && LinearSlideLeft.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("LinearSlideRight", LinearSlideRight.getCurrentPosition() + "  busy=" + LinearSlideRight.isBusy());
            telemetry.addData("LinearSlideLeft", LinearSlideLeft.getCurrentPosition() + "  busy=" + LinearSlideLeft.isBusy());
            telemetry.update();
            idle();
        }
        LinearSlideLeft.setPower(0);
        LinearSlideRight.setPower(0);

    }

    private void slide_up(int x) {
        LinearSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        LinearSlideRight.setDirection(DcMotor.Direction.REVERSE);
        // reset encoder counts kept by motors.
        //LinearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        //LinearSlideLeft.setTargetPosition(x);
        LinearSlideRight.setTargetPosition(x);

        //LinearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LinearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LinearSlideLeft.setPower(.5);
        LinearSlideRight.setPower(.5);
        while (opModeIsActive() && LinearSlideLeft.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("LinearSlideRight", LinearSlideRight.getCurrentPosition() + "  busy=" + LinearSlideRight.isBusy());
            telemetry.addData("LinearSlideLeft", LinearSlideLeft.getCurrentPosition() + "  busy=" + LinearSlideLeft.isBusy());
            telemetry.update();
            idle();
        }
        LinearSlideLeft.setPower(0);
        LinearSlideRight.setPower(0);

    }
    public void turnIMU_right(int degrees, double power){
        YawPitchRollAngles orientation1 = imu.getRobotYawPitchRollAngles();
        double degreesMoved= orientation1.getYaw(AngleUnit.DEGREES);
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        boolean condition=true;

        while(condition) {
            orientation = imu.getRobotYawPitchRollAngles();

            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            if(orientation.getYaw(AngleUnit.DEGREES)>degrees){
                condition=false;
            }
            if (orientationIsValid) {
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                updateOrientation();

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("startdegrees", "%.2f Deg/Sec", degreesMoved);
                telemetry.addData("degrees", "%.2f Deg/Sec", ((Math.abs(orientation.getYaw(AngleUnit.DEGREES)-degreesMoved))));
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            } else {
                telemetry.addData("Error", "Selected orientation on robot is invalid");
            }

            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
    public void Fold(double degrees,int initPosition, double power) {
        double positionfold = encoderFold.getCurrentPosition()-initPosition;
        double degreesToTick = (degrees*8192)/(360);
        if(positionfold<degrees){
            while(Math.abs(positionfold)<Math.abs(degreesToTick)){
                positionfold = encoderFold.getCurrentPosition()-initPosition;
                foldable.setPower(power);
            }
        }
        else{
            while(Math.abs(positionfold)>Math.abs(degreesToTick)){
                positionfold = encoderFold.getCurrentPosition()-initPosition;
                foldable.setPower(-power);
            }
        }

        if(degrees<150){
            foldable.setPower(.1);
        }
        else{
            foldable.setPower(-.1);
        }


    }
    public void turn_left(double degrees, double power){
        int initial_positionLeft = encoderLeft.getCurrentPosition();
        int initial_positionRight = encoderRight.getCurrentPosition();
        int initial_positionBack = encoderBack.getCurrentPosition();
        int positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
        int positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
        int positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
        double positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
        double positionInRight = (positionRight* Math.PI*1.25984)/2000;
        double positionInBack = (positionBack* Math.PI*1.25984)/2000;
        double inches= degrees*.083;

        while (Math.abs(positionInBack)<inches){
            positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
            positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
            positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
            positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
            positionInRight = (positionRight* Math.PI*1.25984)/2000;
            positionInBack = (positionBack* Math.PI*1.25984)/2000;
            telemetry.addData("Encoder left/Right/Back", "%d, %d, %d", positionLeft, positionRight,positionBack);
            telemetry.addData("Inches left/Right/Back", "%4.2f, %4.2f, %4.2f", positionInLeft, positionInRight,positionInBack);
            telemetry.update();
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);


        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
    public void turn_Right(double degrees, double power){
        int initial_positionLeft = encoderLeft.getCurrentPosition();
        int initial_positionRight = encoderRight.getCurrentPosition();
        int initial_positionBack = encoderBack.getCurrentPosition();
        int positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
        int positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
        int positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
        double positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
        double positionInRight = (positionRight* Math.PI*1.25984)/2000;
        double positionInBack = (positionBack* Math.PI*1.25984)/2000;
        double inches= degrees*.083;

        while (Math.abs(positionInBack)<inches){
            positionLeft = encoderLeft.getCurrentPosition()-initial_positionLeft;
            positionRight = encoderRight.getCurrentPosition()-initial_positionRight;
            positionBack = encoderBack.getCurrentPosition()-initial_positionBack;
            positionInLeft = (positionLeft* Math.PI*1.25984)/2000;
            positionInRight = (positionRight* Math.PI*1.25984)/2000;
            positionInBack = (positionBack* Math.PI*1.25984)/2000;
            telemetry.addData("Encoder left/Right/Back", "%d, %d, %d", positionLeft, positionRight,positionBack);
            telemetry.addData("Inches left/Right/Back", "%4.2f, %4.2f, %4.2f", positionInLeft, positionInRight,positionInBack);
            telemetry.update();
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);


        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
}

