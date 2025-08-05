package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
@Config
public class PurePersuitTest extends LinearOpMode {

    public static double radius = 800;//mm
    public static double targetHeading = Math.toRadians(90);//mm

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);


        ArrayList<PurePersuit.Point> points = new ArrayList<>(Arrays.asList(
                new PurePersuit.Point(Localizer.getCurrentPosition().x,Localizer.getCurrentPosition().y),
                new PurePersuit.Point(-325, -505),
                new PurePersuit.Point(-2158, -522),
                new PurePersuit.Point(-2168, -2862),
                new PurePersuit.Point(-977, -3000)
        ));
        PurePersuit trajectory = new PurePersuit(points,Math.toRadians(targetHeading),radius);
        while (opModeInInit()){

        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            SparkFunOTOS.Pose2D pos = trajectory.FromLinesGeneratePointToFollow();
            RobotLog.ii("want to go pos", " x: " + pos.x + " y: " + pos.y + " h: " + pos.h);
            Chassis.setTargetPosition(pos);
            Chassis.update();
            Localizer.Update();
        }
    }
}
