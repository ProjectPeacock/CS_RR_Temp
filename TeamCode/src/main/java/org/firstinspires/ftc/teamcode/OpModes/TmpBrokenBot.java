package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.TmpHWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

import java.util.List;

@Config
@TeleOp(name = "Temp Broken Bot", group = "Development")
//@Disabled

public class TmpBrokenBot extends LinearOpMode {
    private final static TmpHWProfile robot = new TmpHWProfile();
    private LinearOpMode opMode = this;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        double currentTick, currentTime, currentRPM;
        double v1, v2, v3, v4, robotAngle, powerLevel=1;
        double dpadup, dpaddown, dpadleft, dpadright;
        double modePower = 1;
        double theta=0;
        double r;
        double rightX, rightY;
        double peakPower = 0.30;
        boolean fieldCentric = true;
        boolean servoGrabFlag=false, servoKickFlag=false, servoTransferFlag=false, servoIntakeFlag=false, servoRingFlag=false;
        boolean motorIntakeFlag = false;
        double armPosition = 0.5;
        double wobbleArmPosition = 0.5;
        boolean setup = true;
        double time = 0;


        boolean intake = false,             // tracks whether the user has enabled or disabled the intake system
                intakeForward = true,       // tracks which direction the intake system should go
                grabOpen = false,           // tracks whether the grabber should be open or closed
                intakeDeployed = false;     // tracks whether the intake should be out or in
        ElapsedTime runTime = new ElapsedTime();
        boolean shooter = false;
        double shooterPower = 0.80;
        double buttonPress = 0;
        boolean readyToShoot = false;

        /*
         * ServoRing in the up and ready to receive position:   robot.servoRing.setPosition(0.2);
         * ServoRing in the down and holding rings position:    robot.servoRing.setPosition(0.5);
         * servoTransfer up towards shooter position:           robot.servoTransfer.setPosition(0.7);
         * servoTransfer down to receive rings position:        robot.servoTransfer.setPosition(0.46);
         * servoKick up towards shooter position:               robot.servoKick.setPosition(0.43);
         * servoKick down to receive rings position:            robot.servoKick.setPosition(0.68);
         * servoIntake deployed:                                robot.servoIntake.setPosition(0.65);
         * servoIntake stored:                                  robot.servoIntake.setPosition(0.28);
         */

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        //Add/Subtract time delay (Testing)
       /* while(setup){
            if (gamepad2.x){
                time = time + 1000;
            }

            if (gamepad2.y){
                time = time - 1000;
            }

            if (gamepad2.a){
                setup = false;
            }
        } */

        waitForStart();

        while(opModeIsActive()) {
            currentTime = runTime.time();

            /*
             * Mecanum Drive Control section
             */
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
//                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
//                theta = 0;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;

            if (gamepad2.dpad_down) {
                dpaddown = 1;
                telemetry.addData("Motor = ", "MotorLR");
            } else dpaddown = 0;

            if (gamepad2.dpad_up) {
                dpadup = 1;
                telemetry.addData("Motor = ", "MotorLF");
            } else dpadup = 0;

            if (gamepad2.dpad_right) {
                dpadright = 1;
                telemetry.addData("Motor = ", "MotorRR");
            } else dpadright = 0;

            if(gamepad2.dpad_left) {
                dpadleft = 1;
                telemetry.addData("Motor = ", "MotorRF");
            }
            else dpadleft = 0;


            robot.motorLF.setPower(v1 * modePower + dpadup);
            robot.motorRF.setPower(v2 * modePower + dpadleft);
            robot.motorLR.setPower(v3 * modePower + dpaddown);
            robot.motorRR.setPower(v4 * modePower + dpadright);


            /**
             * ##############################################################################
             * ##############################################################################
             * ################    GAMEPAD 1 NORMAL CONTROLS   #############################
             * ##############################################################################
             * ##############################################################################
             */

            if(gamepad1.right_trigger > 0){
                drive.deployIntake();
                drive.intakePower(gamepad1.right_trigger);
            } else if(gamepad1.left_trigger > 0) {
                drive.resetIntake();
                drive.intakePower(-gamepad1.left_trigger);
            } else {
                drive.resetIntake();
                drive.intakePower(0);
            }

            if(gamepad1.left_bumper) {
                drive.dumpBucket();
            } else drive.resetBucket();

            if(gamepad1.a) {
                drive.armOut();
            } else if(gamepad1.b) {
                drive.armReset();
            }

            if (gamepad1.x) {
                drive.liftPosition(robot.LIFT_LOW);
            } else if (gamepad1.y){
                drive.resetLift();
            }

            /**
             * #################################################################################
             * #################################################################################
             * #################      PROVIDE USER FEEDBACK    #################################
             * #################################################################################
             * #################################################################################
             */
            telemetry.addData("IMU Value = ", drive.getZAngle());
            telemetry.addData("Servo Bucket = ", robot.servoBucket.getPosition());
            telemetry.addData("Servo Arm = ", robot.servoArm.getPosition());
            telemetry.addData("Servo Intake Right = ", robot.servoIntakeRight.getPosition());
            telemetry.addData("Servo Intake Left = ", robot.servoIntakeLeft.getPosition());
            telemetry.addData("Motor Lift Encoder = ", robot.motorLift.getCurrentPosition());
            telemetry.addData("Motor RF Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Motor LF Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Motor RR Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Motor LR Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Calculated Distance = ", drive.calcDistance(0,0,0,0,0));
            telemetry.update();

        }   // end of while opModeIsActive()
    }   // end of runOpMode method

    boolean toggleFlag(boolean flag){
        if (flag) return false;
        else return true;
        }   // end of while(opModeIsActive)

}       // end of BrokenBot class