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
        RobotInitializers.clearCache();
        Localizer.Update();

        ArrayList<PurePersuit.Point> points = new ArrayList<>(Arrays.asList(
                new PurePersuit.Point(Localizer.getCurrentPosition().x,Localizer.getCurrentPosition().y),
                new PurePersuit.Point(-325, -505),
                new PurePersuit.Point(-2158, -522),
                new PurePersuit.Point(-2168, -2862),
                new PurePersuit.Point(-977, -3000)
        ));
        Chassis.PurePersuitTrajectory = new PurePersuit(points,targetHeading,radius);
        Chassis.usedTrajectory = Chassis.trajectoryStates.FOLLOWINGPUREPERSUIT;
        while (opModeInInit()){
            Localizer.Update();
            RobotInitializers.clearCache();
            RobotInitializers.Dashtelemetry.addData("x",Localizer.getCurrentPosition().x);
            RobotInitializers.Dashtelemetry.addData("y",Localizer.getCurrentPosition().y);
            RobotInitializers.Dashtelemetry.addData("h",Localizer.getCurrentPosition().h);
        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            RobotInitializers.Dashtelemetry.addData("is traj dist done",Chassis.IsPositionDone(30));
            RobotInitializers.Dashtelemetry.addData("is heading traj done",Chassis.IsHeadingDone(5));
            Chassis.update();
            Localizer.Update();
        }
    }
}
