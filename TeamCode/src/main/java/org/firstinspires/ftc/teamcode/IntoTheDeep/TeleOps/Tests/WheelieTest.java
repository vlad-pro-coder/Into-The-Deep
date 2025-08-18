package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class WheelieTest extends LinearOpMode {


    public static boolean activate = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            if(!activate)
            {
                PtoAndWheelie.IdleWheeliePos();
            }
            else
            {
                PtoAndWheelie.TiltRobot();
            }
        }
    }
}
