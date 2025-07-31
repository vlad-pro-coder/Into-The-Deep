package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if(hub == Hubs.ControlHub){
                RobotInitializers.ControlHubServos.setServoPosition(port, angle / 355.f);
            } else if(hub == Hubs.ExpansionHub) {
                RobotInitializers.ExpansionHubServos.setServoPosition(port, angle / 355.f);
            } else {
                RobotInitializers.ServoHub.setServoPosition(port, angle / 355.f);
            }
        }
    }

    public enum Hubs{
        ControlHub,
        ExpansionHub,
        ServoHub
    }
    public static Hubs hub = Hubs.ControlHub;
    public static int port = 0;
    public static double angle = 0;


}