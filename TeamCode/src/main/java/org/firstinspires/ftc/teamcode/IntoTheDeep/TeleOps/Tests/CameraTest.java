package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.orderedSamples;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.GetExtendoTicksToTravelAndNeededAngleFromSample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;

import java.util.List;

@TeleOp
@Config
public class CameraTest extends LinearOpMode {

    Logitech920 camera;

    public static boolean gofaceSample = false;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        Intake.DropUp();
        Extendo.DoingAuto = true;
        Scheduler tasks = new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        YellowSampleDetectionPipeline.SamplePoint bestsample = camera.yellow.getSamplesPrioritized().poll();
                        SparkFunOTOS.Pose2D data = GetExtendoTicksToTravelAndNeededAngleFromSample(new SparkFunOTOS.Pose2D(bestsample.x,bestsample.y,0));
                        Extendo.state = Extendo.ExtendoStates.GOTOPOS;
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(Localizer.getCurrentPosition().x,Localizer.getCurrentPosition().y,data.h));
                        Extendo.setExtendoPos(data.x);
                    }

                    @Override
                    protected boolean Conditions() {
                        Chassis.update();
                        Extendo.update();
                        return false;
                    }
                });

        camera  = new Logitech920("logitik",hardwareMap);
        camera.startCamera();

        while(opModeInInit()){

        }
        waitForStart();
        while(opModeIsActive()){
            RobotInitializers.clearCache();
            //RobotInitializers.Dashtelemetry.clear();
            if(!gofaceSample) {
                List<Double> txes = camera.yellow.getTx(), tyes = camera.yellow.getTy();
                for (int i = 1; i < txes.size(); i++) {
                    RobotInitializers.Dashtelemetry.addData("tx" + i, Math.toDegrees(txes.get(i)));
                    RobotInitializers.Dashtelemetry.addData("ty" + i, Math.toDegrees(tyes.get(i)));

                    SparkFunOTOS.Pose2D pos = GetSamplePosition.GetGlobalSamplePosition(Math.toDegrees(txes.get(i)), Math.toDegrees(tyes.get(i)));
                    RobotInitializers.Dashtelemetry.addData("x" + i, pos.x);
                    RobotInitializers.Dashtelemetry.addData("y" + i, pos.y);

                    SparkFunOTOS.Pose2D data = GetExtendoTicksToTravelAndNeededAngleFromSample(pos);

                    RobotInitializers.Dashtelemetry.addData("extend" + i, data.x);
                    RobotInitializers.Dashtelemetry.addData("needed heading" + i, data.h);

                }

                RobotInitializers.Dashtelemetry.addData("camera x", GetSamplePosition.CameraRelativeToField(Localizer.getCurrentPosition()).x);
                RobotInitializers.Dashtelemetry.addData("camera y", GetSamplePosition.CameraRelativeToField(Localizer.getCurrentPosition()).y);
            }

            RobotInitializers.Dashtelemetry.addData("extendo ticks",Extendo.getPosition());
            RobotInitializers.Dashtelemetry.addData("extendo target",Extendo.pidController.getTargetPosition());
            RobotInitializers.Dashtelemetry.update();
            if(gofaceSample)
                tasks.update();

            Localizer.Update();
        }
    }
}
