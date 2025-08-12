package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class HoldPos extends LinearOpMode {

    public static SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(610,-315,Math.toRadians(98));
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);


        waitForStart();

        while(opModeIsActive())
        {

            RobotInitializers.clearCache();

            Chassis.setTargetPosition(pos);
            Chassis.update();
            Localizer.Update();
        }
    }
}
