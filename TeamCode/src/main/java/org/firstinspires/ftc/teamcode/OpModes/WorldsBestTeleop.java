package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.LiftControlClass;

@Config
@TeleOp(name = "World's Best Teleop", group = "Competition")
public class WorldsBestTeleop extends OpMode {
    private PIDController controller;
    public static double kP=0;
    public static int tolerance=0;
    public static double p=0.0015, i=0, d=0;
    public static double f=-0.2;

    public static int target=0;

    private final double ticks_in_degrees = 8192/360;

    private int bumpCount;
    private int offset, liftPos, climbPos;
    private boolean toggleReadyUp, toggleReadyDown, armOverride, alignAdjustReady, intakeToggle, clawReady;
    private boolean armOverrideReady, armToggle;
    private HWProfile robot;
    private GamepadEx gp1;
    private LiftControlClass liftClass;
    private double strafePower, forwardPower, turnPower;
    private double armPos = 1.0;

    private boolean climbDeployed = false;
    private boolean armScore = false;
    private double bucketStart = 0.6;

    @Override
    public void init(){
        robot = new HWProfile();
        robot.init(hardwareMap);
        controller=new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gp1 = new GamepadEx(gamepad1);
        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        liftClass = new LiftControlClass(robot);
        liftClass.intakeOff();
        robot.servoArm.setPosition(1.0);
        robot.servoBucket.setPosition(bucketStart);
        robot.servoLauncher.setPosition(0.5);

        //lift.setPositionTolerance(tolerance);
        //lift.setPositionCoefficient(kP);
    }

    @Override
    public void loop(){
        //DRIVE CONTROL SECTION//
        //drive power input from analog sticks
        forwardPower=gp1.getLeftY();
        strafePower=gp1.getLeftX();
        if(gp1.getRightX()<0.75){
            turnPower=-Math.pow(gp1.getRightX(),2)*0.75*robot.TURN_MULTIPLIER;
        }else{
            turnPower=-gp1.getRightX()*robot.TURN_MULTIPLIER;
        }

        //FTCLib drive code, instantiated in HWProfile
        robot.mecanum.driveFieldCentric(strafePower,forwardPower,-gp1.getRightX()*robot.TURN_MULTIPLIER,robot.imu.getRotation2d().getDegrees()+180, true);        //END OF DRIVE CONTROL SECTION//

        //CLAW CONTROL SECTION//
        if(gp1.isDown(GamepadKeys.Button.A)&&clawReady){
            intakeToggle =!intakeToggle;
        }

        //forces claw to only open or close if button is pressed once, not held
        clawReady = !gp1.isDown(GamepadKeys.Button.A);

        //apply value to claw
        if(!gamepad1.y) {
            if (intakeToggle) {
                liftClass.intakeOn();
            } else {
                liftClass.intakeOff();
            }
        }else{
            liftClass.intakeReverse();
        }
        //END OF CLAW CONTROL SECTION//

        // toggle clawReady
        clawReady= !gp1.isDown(GamepadKeys.Button.A);

        //ARM CONTROL SECTION//
        if(gp1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)&&armOverrideReady){
            armOverride=!armOverride;
        }

        //checks for arm button being held down
        armOverrideReady= !gp1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON);

        //change armToggle based on lift position
        armToggle = bumpCount > 0;

        if (bumpCount==0){
            armOverride=false;
        }

        //apply value to arm
        if(!armOverride) {
            if (armToggle) {
              // liftClass.armScore();
            } else {
              //  liftClass.armIntake();
            }
        }else{
           // liftClass.armIntake();
        }
        //END OF ARM CONTROL SECTION//

        //LIFT CONTROL SECTION//
        if(!gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            toggleReadyUp=true;
        }
        if(!gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            toggleReadyDown=true;
        }

        //lift reset
        if(gp1.isDown(GamepadKeys.Button.B)){
            robot.motorLift.setPower(0.25);
            bumpCount=0;
            offset=0;
            robot.servoArm.setPosition(1);
            robot.servoBucket.setPosition(bucketStart);
            target= robot.LIFT_BOTTOM;
            armOverride=false;
        }

        //increase lift position
        if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&toggleReadyUp){
            offset=0;
            toggleReadyUp=false;
            if(bumpCount<3){
                bumpCount++;
            }

            //decrease lift position
        }else if(gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)&&toggleReadyDown){
            offset=0;
            toggleReadyDown=false;
            if(bumpCount>0){
                bumpCount--;
            }
        }

        //move lift down to score
 /*
        if(gp1.isDown(GamepadKeys.Button.X)&&alignAdjustReady){
            offset+=300;
        }

        if(!gp1.isDown(GamepadKeys.Button.X)){
            alignAdjustReady=true;
        }
*/
        //apply lift positions
        if (bumpCount == 0) {
            robot.servoBucket.setPosition(bucketStart);
            robot.servoArm.setPosition(1);
            target = robot.LIFT_BOTTOM + offset;
        } else if (bumpCount == 1) {
            robot.servoArm.setPosition(0);
            target = robot.LIFT_LOW+offset;
        } else if (bumpCount == 2) {
            robot.servoArm.setPosition(0);
            target = robot.LIFT_MID+offset;
        } else if (bumpCount == 3) {
            robot.servoArm.setPosition(0);
            target = robot.LIFT_HIGH+offset;
        }
        //adjust lift position
        if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5){
            offset-= robot.liftAdjust;
        }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5){
            offset += robot.liftAdjust;
        }

        //clip lift position to proper range (encoder counts in reverse so clip is negative)
        liftPos= Range.clip(liftPos,robot.MAX_LIFT_VALUE,-50);

        int armPos = robot.motorLift.getCurrentPosition();

        //FLIPPER CONTROL SECTION//
        if(gp1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            //liftClass.flippersDown();
        }else{
            //liftClass.flippersUp();
        }


        //END OF FLIPPER CONTROL SECTION//

        controller.setPID(p,i,d);
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((target/ticks_in_degrees)))*f;

        robot.motorLift.setTargetPosition(target);
        if(bumpCount>0) {
            robot.motorLift.setPower(1);
            if(gp1.getButton(GamepadKeys.Button.X)){
                robot.servoBucket.setPosition(0);
            }else{
                robot.servoBucket.setPosition(1);
            }
        }else{
            robot.motorLift.setPower(0.25);
        }


            if(gp1.getButton(GamepadKeys.Button.DPAD_UP)){
                robot.servoLauncher.setPosition(0.25);
                }
/*
        if(gp1.getButton(GamepadKeys.Button.DPAD_LEFT)){
            if (!climbDeployed){
                robot.motorClimbLeft.setPower(1);
             //   robot.motorClimbLeft.setTargetPosition(robot.CLIMB_DEPLOY);
                robot.motorClimbRight.setPower(1);
                //robot.motorClimbRight.setTargetPosition(robot.CLIMB_DEPLOY);
                climbDeployed = true;

            }
        else {
                if (climbDeployed) {
                //    robot.motorClimbLeft.setTargetPosition(robot.CLIMB_START);
                    //robot.motorClimbRight.setTargetPosition(robot.CLIMB_START);
                    climbDeployed = false;
                }
            }
        }

 */

       // telemetry.addData("pos: ",armPos);
        telemetry.addData("climb: ",climbDeployed);
        telemetry.addData("climb pos",robot.motorClimbLeft.getTargetPosition());

         //climb position
        if (gamepad1.dpad_left){
            climbPos+=25;
        }else if (gamepad1.dpad_right){
            climbPos-=25;
        }
        climbPos=Range.clip(climbPos,0,robot.CLIMB_DEPLOY);

        robot.motorClimbRight.setTargetPosition(climbPos);
        robot.motorClimbRight.setPower(1);
        robot.motorClimbLeft.setTargetPosition(climbPos);
        robot.motorClimbLeft.setPower(1);
    }

}
