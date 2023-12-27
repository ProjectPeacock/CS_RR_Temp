package org.firstinspires.ftc.teamcode.ignore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@Config
@TeleOp(name = "World's Best Teleop CS", group = "Competition")
@Disabled
public class WorldsBestTeleopCS extends OpMode {
    private HWProfile robot;
    private GamepadEx gp1;
    private double strafePower, forwardPower, turnPower;
//    private opMode myOpmode= this;

    @Override
    public void init(){
        robot = new HWProfile();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gp1 = new GamepadEx(gamepad1);
        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

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

    }

}
