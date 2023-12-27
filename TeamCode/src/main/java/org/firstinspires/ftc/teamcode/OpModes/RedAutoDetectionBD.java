package org.firstinspires.ftc.teamcode.OpModes;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.TmpHWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "Red Auto Backdrop", group = "Peacock")
//@Disabled

public class RedAutoDetectionBD extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    private final static TmpHWProfile robot = new TmpHWProfile();
    private LinearOpMode opMode = this;
    private State state = State.DETECTION;

    public void runOpMode(){

        int position = 1;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);
        DriveMecanum drive = new DriveMecanum(robot, opMode);
        initDoubleVision();

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

        while(!opModeIsActive()) {
            position = telemetryTfod();
            telemetry.addData("Postion = ", position);
            if(position == 1) {
                telemetry.addData("Location = ", "LEFT");
            } else if (position == 2) {
                telemetry.addData("Location = ", "Center");
            } else telemetry.addData("Location = ", "RIGHT");
            telemetry.update();
        }  // end of while

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup

        // waitForStart();
        while(opModeIsActive()) {
            switch (state) {

                case DETECTION:

                    // drop the intake arm down
                    robot.servoIntakeLeft.setPosition(robot.INTAKE_DOWN_LEFT);
                    robot.servoIntakeRight.setPosition(robot.INTAKE_DOWN_RIGHT);

                    if (position == 1) {
                        state = State.PLACE_LEFT;
                    } else if (position == 2) {
                        state = State.PLACE_CENTER;
                    } else state=State.PLACE_RIGHT;

                    break;

                case PLACE_LEFT:

                    // drive forward to get away from wall
                    drive.driveDistance(0.3, 0, 10);

                    // turn towards the line
                    drive.PIDRotate(-45, 1);

                    // Drive forward to place the pixel was 11
                    drive.driveDistance(0.2, 0, 12);

                    robot.servoIntakeLeft.setPosition(robot.INTAKE_UP_LEFT);
                    robot.servoIntakeRight.setPosition(robot.INTAKE_UP_RIGHT);

                    // Back away from the pixel
                    drive.driveDistance(0.3, 180, 5);

                    // turn towards the backdrop
                    drive.PIDRotate(-90, 1);

                    // drive towards the back drop
                    drive.driveDistance(0.3, 180, 24);

                    // strafe to place the yellow pixel on the backdrop was 13
                    drive.driveDistance(0.3, 90, 16);

                    // drive closer to the backdrop
                    drive.driveDistance(0.1, 180, 12);

                    // lift bucket into position to score
                    drive.intakePower(0.5);
                    sleep(50);
                    drive.liftPosition(robot.LIFT_LOW);
                    drive.armOut();
                    drive.intakePower(0);
                    drive.liftPosition(robot.LIFT_SCORE);
                    sleep(1000);

                    // score pixels
                    drive.dumpPixel();

                    //reset the lift
                    drive.resetLift();

                    // strafe to the parking zone
                    drive.driveDistance(0.3, 0, 5);

                    // strafe to the parking zone
                    drive.driveDistance(0.3, 90, 20);

                    state = State.PARK;

                   break;

                case CYCLE1:
                    state = State.HALT;

                    break;


                case PLACE_CENTER:
                    // drive forward to place the pixel on the center line
                    drive.driveDistance(0.3, 0, 29);

                    robot.servoIntakeLeft.setPosition(robot.INTAKE_UP_LEFT);
                    robot.servoIntakeRight.setPosition(robot.INTAKE_UP_RIGHT);

                    // back away from the pixel placed on the center line
                    drive.driveDistance(0.3, 180, 5);

                    // turn towards the backdrop
                    drive.PIDRotate(-90, 1);

                    // drive towards the back drop
                    drive.driveDistance(0.3, 180, 24);

                    // strafe into position
                    drive.driveDistance(0.3, 90, 8);

                    // drive backwards into the backdrop
                    drive.driveDistance(0.1, 180, 8);

                    // lift bucket into position to score
                    drive.intakePower(0.5);
                    sleep(50);
                    drive.liftPosition(robot.LIFT_LOW);
                    drive.armOut();
                    drive.intakePower(0);
                    drive.liftPosition(robot.LIFT_SCORE);
                    sleep(1000);

                    // score pixels
                    drive.dumpPixel();

                    //reset the lift
                    drive.resetLift();

                    // drive to parking position
                    drive.driveDistance(0.3, 0, 5);

                    // strafe to parking position
                    drive.driveDistance(0.3, 90, 25);

                    state = State.PARK;
                    break;

                case PLACE_RIGHT:
                    // drive forward to place the pixel on the center line
                    drive.driveDistance(.3, 0, 5);

                    // rotate towards the line
                    drive.PIDRotate(25, 0.5);

                    // drive to place pixel
                    drive.driveDistance(0.3, 0, 10);

                    robot.servoIntakeLeft.setPosition(robot.INTAKE_UP_LEFT);
                    robot.servoIntakeRight.setPosition(robot.INTAKE_UP_RIGHT);

                    // Rotate towards
                    drive.PIDRotate(0, 0.5);
                    // back away from the pixel
                    drive.driveDistance(0.3, 180, 5);

                    // strafe towards the back drop
                    drive.driveDistance(0.3, 90, 10);

                    // turn towards the backdrop
                    drive.PIDRotate(-90, 0.5);

                    // drive back towards the backdrop
                    drive.driveDistance(0.3, 180, 12);

                    // strafe into the correct position was 8
                    drive.driveDistance(0.3, 90, 5);

                    // drive back into the backdrop
                    drive.driveDistance(0.1, 180, 12);

                    // lift bucket into position to score
                    drive.intakePower(0.5);
                    sleep(50);
                    drive.liftPosition(robot.LIFT_LOW);
                    drive.armOut();
                    drive.intakePower(0);
                    drive.liftPosition(robot.LIFT_SCORE);
                    sleep(1000);
                    // score pixels
                    drive.dumpPixel();

                    //reset the lift
                    drive.resetLift();

                    // drive into parking position
                    drive.driveDistance(0.3, 0, 5);

                    // strafe to parking position
                    drive.driveDistance(0.3, 90, 25);
                    state = State.PARK;

                    break;

                case PARK:

                    drive.PIDRotate(0, 0.5);

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
        DETECTION, PLACE_LEFT, PLACE_CENTER, PLACE_RIGHT, CYCLE1, PARK, HALT
    }   // end of enum State

    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        String[] LABELS = {
                "Blue_Rook",
                "Red_Rook"};

        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("PP_CenterStage_v1.tflite")
                .setModelLabels(LABELS)

                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.US,"XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US,"PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US,"RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.US,"Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int telemetryTfod() {
        int position=1;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (recognition.getLabel().equals("Red_Rook")) {
                if (recognition.getLeft() > 200 && recognition.getLeft() <= 400 ) {
                    telemetry.addData("Target Location = ", "CENTER");
                    position = 2;
                } else if (recognition.getLeft() > 400){
                    telemetry.addData("Target Location = ", "RIGHT");
                    position = 3;
                } else {
                    telemetry.addData("Target Location = ", "LEFT");
                    position = 1;
                }
            }
        }   // end for() loop

        return position;

    }   // end method telemetryTfod()


}   // end of TestAuto.java class