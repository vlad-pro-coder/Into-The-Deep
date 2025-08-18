package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_fromsubmersibletobasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathbacktobasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathtosubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_radius;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer.pinPoint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
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

        Scheduler follower = new Scheduler();
        Scheduler tasks = new Scheduler()
                .waitSeconds(2)
                .StartPurePersuit(PUREPERSUIT_pathtosubmersible,HEADING_infrontofsubmersible,PUREPERSUIT_radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(30) && Chassis.IsHeadingDone(5);
                    }
                })
                .waitSeconds(2)
                .StartPurePersuit(PUREPERSUIT_pathbacktobasket,HEADING_fromsubmersibletobasket,PUREPERSUIT_radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(30) && Chassis.IsHeadingDone(5);
                    }
                });


        while (opModeInInit()){
            Localizer.Update();
            RobotInitializers.clearCache();
            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(445, -175,HEADING_fromsubmersibletobasket));
            Chassis.update();
            RobotInitializers.Dashtelemetry.addData("x",Localizer.getCurrentPosition().x);
            RobotInitializers.Dashtelemetry.addData("y",Localizer.getCurrentPosition().y);
            RobotInitializers.Dashtelemetry.addData("h",Localizer.getCurrentPosition().h);
        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            if(follower.IsSchedulerDone())
                follower.AddAnotherScheduler(tasks);
            follower.update();

            Chassis.update();
            Localizer.Update();
            RobotInitializers.Dashtelemetry.addData("",pinPoint.getFrequency());
        }
    }
}
