package org.firstinspires.ftc.teamcode.ignore;

/*
 * Program Name:
 * Alliance:
 * Starting position
 * Functions of the program:
 *  - STEP1 =   gets the foundation into the build site
 *  - STEP2
 *
 *
 *
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.TmpHWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Red Auto Park Detect", group = "Peacock")
@Disabled

public class
ppRedAutoDetect extends LinearOpMode {

    private final static TmpHWProfile robot = new TmpHWProfile();
    private LinearOpMode opMode = this;
    private State state = State.DETECTION;
    private PixelPosition pixelposition = PixelPosition.MIDDLE;
    public ppRedAutoDetect(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        double startTime;
        double timeElapsed;
//        int position = 1;
        ElapsedTime runTime = new ElapsedTime();
        double lastTime=runTime.time();


        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Initialize all hardware
         */

        telemetry.addData("Ready to initialize all servos", "");
        telemetry.addData("Robot state = ", "Initializing");
        telemetry.addData("Keep appendages back ", "");
        telemetry.update();

        robot.servoBucket.setPosition(robot.BUCKET_RESET);
        robot.servoArm.setPosition(robot.ARM_RESET);

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();
        startTime = runTime.time();

        while(opModeIsActive()) {

            switch (state) {

                case DETECTION:

                        state = State.PLACE_PRELOAD;

                    break;

                case PLACE_PRELOAD:
                    //place preloaded pixel
                    switch (pixelposition) {

                        case LEFT:
                            drive.driveDistance(.5, 15, 25);
                            drive.driveDistance(-.5, 15, 10);

                            break;
                        case MIDDLE:
                            // drive forward to get away from wall
                            drive.driveDistance(.5, 0, 30);
                            drive.driveDistance(-.5, 0, 10);
                            break;
                        case RIGHT:
                            drive.driveDistance(.5, -15, 25);
                            drive.driveDistance(-.5, -15, 10);

                            break;

                    }

                    state = State.PLACE_PIXEL1;
                    break;

                case PLACE_PIXEL1:


                    // drive forward to get away from wall
                    //drive.driveDistance(.5, 0, 20);

                    // turn towards the backdrop
                    drive.PIDRotate(-90, 1);


                    // drive towards the back drop
                    drive.driveDistance(0.5, 180, 24);

                    // lift bucket into position to score
                    drive.intakePower(0.5);
                    sleep(50);
                    drive.liftPosition(robot.LIFT_LOW);
                    drive.armOut();
                    drive.intakePower(0);

                    // score pixels
                    drive.dumpPixel();

                    //reset the lift
                    drive.resetLift();
                    drive.driveDistance(0.1,0,5);
                    drive.driveDistance(0.5,90,18);
                    drive.PIDRotate(0,0.5);
                    drive.driveDistance(0.1,90,5);
                    state = State.HALT;

                   break;

                case CYCLE1:
                    // drive to gather more pixels heading towards stage door
                   // drive.robotCorrect2(0.5, 45, 1.1);
                    drive.driveDistance(0.1,0,5);
                    drive.driveDistance(0.5,90,20);

                    // head towards pixels
                    drive.PIDRotate(-90, 0.5);
                    drive.driveDistance(0.5,0, 100);

                    // turn intake on and gather pixels
                    drive.deployIntakeMid();
                    drive.intakePower(1.0);

                    // gather pixels
                    drive.driveDistance(0.1, 0, 20);

                    // spit out extra pixels and retract intake
                    drive.intakePower(-1);
                    sleep(50);
                    drive.resetIntake();

                    // go score
                    drive.PIDRotate(-90,0.5);
                    drive.driveDistance(0.5, 180, 85);

                    //head to backdrop
                    drive.driveDistance(0.5, -90, 20);
                    drive.driveDistance(0.1,180,15);

                    // deploy lift
                    drive.intakePower(0.5);
                    sleep(50);
                    drive.liftPosition(robot.LIFT_MID);
                    drive.armOut();
                    drive.intakePower(0);

                    // score pixels
                    drive.dumpPixel();

                    //reset the lift
                    drive.resetLift();

                    state = State.HALT;

                    break;


                case CYCLE2:

                    state = State.CYCLE3;
                    break;

                case CYCLE3:
                    state = State.PARK;

                    break;

                case PARK:

                    state = State.HALT;

                    break;

                case HALT:
                    // Stop all motors
                    drive.motorsHalt();

                    requestOpModeStop();

                    break;
            }   // end of the switch state
        }   // end of while opModeIsActive()

    }   // end of runOpMode method

    /*
     * Enumuerate the states of the machine
     */
    enum State {
        DETECTION, PLACE_PRELOAD, PLACE_PIXEL1, CYCLE1, CYCLE2, CYCLE3, PARK, HALT;
    }   // end of enum State

    enum PixelPosition{
        LEFT, MIDDLE, RIGHT;
    }   // end of enum State

}   // end of TestAuto.java class