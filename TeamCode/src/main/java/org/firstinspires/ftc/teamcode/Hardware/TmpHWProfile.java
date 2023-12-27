package org.firstinspires.ftc.teamcode.Hardware;


import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TmpHWProfile {

    //constants
    public final boolean fieldCentric=true;

    //lift PID constants
    public final double kF=-0.2;
    public final double ticks_in_degrees = 8192/360;


    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //drive constants
    public final double TURN_MULTIPLIER = 0.75;
    public final int USD_COUNTS_PER_INCH = 23;

    //bucket constants
    public final double BUCKET_RESET = 0.75;
    public final double BUCKET_UP = 1.0;
    public final double BUCKET_DUMP = 0.1;

    // servoGate constants
    public final double GATE_OPEN = 0.5;
    public final double GATE_CLOSE = 0.2;

    // arm constants
    public final double ARM_OUT = 0;
    public final double ARM_RESET = 1;


    //lift constants
    final public int liftAdjust=375;
    final public double LIFT_POW=1;
    final public int MAX_LIFT_VALUE = -57500;
    public final int ARM_THRESHOLD =-15600;
    final public int LIFT_BOTTOM=-1;
    final public int LIFT_LOW=-334;
    final public int LIFT_SCORE=-250;
    final public int LIFT_MID=-603;
    final public int LIFT_HIGH=-839;

    //intake constants
    final public double INTAKE_UP_LEFT = 0.5;
    final public double INTAKE_UP_RIGHT = 0.5;
    final public double INTAKE_MID_LEFT = 0.65;
    final public double INTAKE_MID_RIGHT = 0.35;
    final public double INTAKE_DOWN_LEFT = .8;
    final public double INTAKE_DOWN_RIGHT = 0.2;

    //final private int liftTicksPerInch= (int) 8192/4.3267;
    final public int liftTicksPerInch= 1893;

    final public int DRIVE_TICKS_PER_INCH = 23;        // needs to be set


    /* Public OpMode members. */
    public DcMotor motorLF = null;
    public DcMotor motorLR = null;
    public DcMotor motorRF = null;
    public DcMotor motorRR = null;

    public DcMotor motorIntake = null;
    public DcMotor motorLift = null;
    public Servo servoIntakeLeft = null;
    public Servo servoIntakeRight = null;
    public Servo servoArm= null;
    public Servo servoBucket = null;
    public Servo servoGate= null;
    public RevIMU imu = null;

    public MecanumDrive mecanum = null;
    // public MotorEx autoLight = null;


    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public TmpHWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*
         * Initialize Motors
         */

        motorLF = hwMap.dcMotor.get("motorLF");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);

        motorLR = hwMap.dcMotor.get("motorLR");
        motorLR.setDirection(DcMotor.Direction.REVERSE);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);

        motorRF = hwMap.dcMotor.get("motorRF");
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);

        motorRR = hwMap.dcMotor.get("motorRR");
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);

        motorLift = hwMap.dcMotor.get("motorLift");
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);

        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRR.setPower(0);

        servoIntakeLeft = hwMap.servo.get("servoIntakeLeft");
        servoIntakeRight = hwMap.servo.get("servoIntakeRight");
        servoArm = hwMap.servo.get("servoArm");
        servoGate = hwMap.servo.get("servoGate");
        servoBucket = hwMap.servo.get("servoBucket");

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();

    }   // end of init() method

}       // end of the HardwareProfile class