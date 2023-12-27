package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HWProfile {
    //constants
    public final boolean fieldCentric=true;

    //lift PID constants
    public final double kF=-0.2;
    public final double ticks_in_degrees = 8192/360;


    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //drive constants
    public final double TURN_MULTIPLIER = 0.75;


    //lift constants
    final public int liftAdjust=375;
    final public double LIFT_POW=1;
    // was -57500
    final public int MAX_LIFT_VALUE = -3355;
    public final int ARM_THRESHOLD =-15600;
    final public int LIFT_BOTTOM=-1;
    final public int LIFT_LOW=-334;
    final public int LIFT_MID=-603;
    final public int LIFT_HIGH=-839;
    final public int CLIMB_START = 0;
    final public int CLIMB_DEPLOY = 2000;


    //intake constants
    final public double INTAKE_RIGHT_UP = 0.5;
    final public double INTAKE_RIGHT_DOWN = 0.2;
    final public double INTAKE_LEFT_UP = 0.5;
    final public double INTAKE_LEFT_DOWN = 0.8;
    //power was -.5
    final public double INTAKE_IN_PWR = -0.75;
    final public double INTAKE_OUT_PWR = 1;

    //final private int liftTicksPerInch= (int) 8192/4.3267;
    final public int liftTicksPerInch= 1893;


    /* Public OpMode members. */
    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;

    public MotorEx motorIntake = null;
    public DcMotorEx motorLift = null;

    public DcMotorEx motorClimbLeft = null;
    public DcMotorEx motorClimbRight = null;
    public ServoImplEx servoIntakeLeft = null;
    public ServoImplEx servoIntakeRight = null;
    public ServoImplEx servoArm= null;
    public ServoImplEx servoBucket = null;
    public ServoImplEx servoLauncher = null;
    public RevIMU imu = null;

    public MecanumDrive mecanum = null;
   // public MotorEx autoLight = null;


    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //drive motor init
        motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_1150);
        motorLF.setRunMode(Motor.RunMode.RawPower);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLF.resetEncoder();

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_1150);
        motorLR.setRunMode(Motor.RunMode.RawPower);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.resetEncoder();

        motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_1150);
        motorRF.setRunMode(Motor.RunMode.RawPower);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRF.resetEncoder();

        motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_1150);
        motorRR.setRunMode(Motor.RunMode.RawPower);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRR.resetEncoder();

        //drivebase init
        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        //intake motors init
        motorIntake = new MotorEx(ahwMap, "motorIntake", 8192,1150);

        //motorLiftRight = new MotorEx(ahwMap, "motorLiftRight", 8192,6000);
        motorLift = hwMap.get(DcMotorEx.class,"motorLift");
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);

        motorClimbLeft = hwMap.get(DcMotorEx.class,"motorClimbLeft");
        motorClimbLeft.setDirection(DcMotor.Direction.REVERSE);
        motorClimbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorClimbLeft.setTargetPosition(CLIMB_START);
        motorClimbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorClimbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorClimbLeft.setPower(0);

        motorClimbRight = hwMap.get(DcMotorEx.class,"motorClimbRight");
        // motorClimb.setDirection(DcMotor.Direction.REVERSE);
        motorClimbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorClimbRight.setTargetPosition(CLIMB_START);
        motorClimbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorClimbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorClimbRight.setPower(0);

        //intake servo init
        servoIntakeLeft = hwMap.get(ServoImplEx.class, "servoIntakeLeft");
        servoIntakeRight = hwMap.get(ServoImplEx.class, "servoIntakeRight");
        servoArm = hwMap.get(ServoImplEx.class, "servoArm");
        servoBucket = hwMap.get(ServoImplEx.class, "servoBucket");
        servoLauncher = hwMap.get(ServoImplEx.class, "servoLauncher");

        //init imu
        imu = new RevIMU(ahwMap);
        imu.init();
    }
}  // end of HWProfile Class