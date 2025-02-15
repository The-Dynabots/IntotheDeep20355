/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp2nd", group="Linear OpMode")
public class TeleOp2nd extends LinearOpMode {

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
        imu = hardwareMap.get(IMU.class, "imu");
        //logoFacingDirectionPosition = 0; // Up
       // usbFacingDirectionPosition = 2; // Forward
        logoFacingDirectionPosition = 2;
        usbFacingDirectionPosition = 0;
        final double RADIUS = 4.375;
        final double RADIUS_INCHES = 1.771655;

        final double TICKS = 8192;

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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LinearSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        LinearSlideRight.setDirection(DcMotor.Direction.REVERSE);
        foldable.setDirection(DcMotor.Direction.FORWARD);


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
        double power=.75;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double max;
            double positionfold = encoderFold.getCurrentPosition() - initial_positionFold;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
            }



            if (gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05) {
                LinearSlideLeft.setPower(gamepad2.left_stick_y);
                LinearSlideRight.setPower(-gamepad2.left_stick_y);

            } else {
                LinearSlideLeft.setPower(0);
                LinearSlideRight.setPower(0);
            }
            if (gamepad1.right_bumper) {
                power=.5;
            } else if (gamepad1.left_bumper) {
                power = .3;
            }

            if (gamepad2.right_bumper) {
                claw.setPower(.4);
                waits(200);
                claw.setPower(0);

            } else if (gamepad2.left_bumper) {
                claw.setPower(-.4);
            }
            if (gamepad2.dpad_up) {
                joint.setPosition(.15);
            }
            if (gamepad2.dpad_down) {
                joint.setPosition(.17);
            }
            if (gamepad2.dpad_right) {
                basket.setPosition(.3);
            }
            if (gamepad2.dpad_left) {
                basket.setPosition(.9);
            }
            if (gamepad2.right_stick_y > 0.05 || gamepad2.right_stick_y < -0.05) {
                foldable.setPower(gamepad2.right_stick_y*.3);
            }
            else if (gamepad2.b) {
                if (Math.abs(positionfold) < 1600) {
                    foldable.setPower(.5);
                }
                if(Math.abs(positionfold)>1600 && Math.abs(positionfold)<2800) {
                    foldable.setPower(.2);
                    joint.setPosition(.20);
                    claw.setPower(0);
                }
            } else if (gamepad2.x) {
                if (Math.abs(positionfold)>1600) {
                    basket.setPosition(.3);
                    foldable.setPower(-.5);
                }
                if(Math.abs(positionfold)<1600 && positionfold<-800) {
                    basket.setPosition(.35);
                    joint.setPosition(.26);
                    foldable.setPower(-.3);
                }

            }
            else if (Math.abs(positionfold) > 2200) {
                foldable.setPower(-.1);
            }
            else if (Math.abs(positionfold) > 1600) {
                foldable.setPower(-.1);
            } else {
                foldable.setPower(0);
            }

            /*
             if (gamepad2.b) {
                if (positionfold > -3550) {
                    foldable.setPower(.6);
                    joint.setPosition(1);
                }
            } else if (gamepad2.x) {
                if (positionfold > -5600) {
                    foldable.setPower(.5);
                    if (positionfold > -3330) {
                        joint.setPosition(.9);
                    }

                }

             */



            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            rightFrontDrive.setPower(leftFrontPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightBackDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftFrontPower);
            int positionLeft = encoderLeft.getCurrentPosition() - initial_positionLeft;
            int positionRight = encoderRight.getCurrentPosition() - initial_positionRight;
            int positionBack = encoderBack.getCurrentPosition() - initial_positionBack;
            positionfold = encoderFold.getCurrentPosition() - initial_positionFold;
            double positionInLeft = (positionLeft * Math.PI * 1.25984) / 2000;
            double positionInRight = (positionRight * Math.PI * 1.25984) / 2000;
            double positionInBack = (positionBack * Math.PI * 1.25984) / 2000;
            double positionInFold = (positionfold / 8192) * 360;

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Encoder left/Right/Back", "%d, %d, %d", positionLeft, positionRight, positionBack);
            telemetry.addData("Inches left/Right/Back", "%4.2f, %4.2f, %4.2f", positionInLeft, positionInRight, positionInBack);
            telemetry.addData("Servo position", "%4.2f", position);
            telemetry.addData("Fold Degrees", "%4.2f", positionInFold);
            telemetry.addData("Fold ticks", "%4.2f", positionfold);
            telemetry.addData("LinearSlideRight", LinearSlideRight.getCurrentPosition() + "  busy=" + LinearSlideRight.isBusy());
            telemetry.addData("LinearSlideLeft", LinearSlideLeft.getCurrentPosition() + "  busy=" + LinearSlideLeft.isBusy());
            telemetry.update();
            if (orientationIsValid) {
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                updateOrientation();

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            } else {
                telemetry.addData("Error", "Selected orientation on robot is invalid");
            }

            telemetry.update();
        }
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
    private void waits(long miliseconds){
        long starttime = System.currentTimeMillis();
        long timepassed = System.currentTimeMillis() - starttime;
        while((System.currentTimeMillis() - starttime)<miliseconds){

        }
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
        LinearSlideRight.setDirection(DcMotor.Direction.FORWARD);
        // reset encoder counts kept by motors.
        LinearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        LinearSlideLeft.setTargetPosition(x);
        LinearSlideRight.setTargetPosition(x);

        LinearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LinearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LinearSlideLeft.setPower(1);
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
}
