package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;

@TeleOp
public class CameraTest extends LinearOpMode {

    Logitech920 camera;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        camera  = new Logitech920("logitik",hardwareMap);
        camera.startCamera();

        while(opModeInInit()){

        }
        waitForStart();
        while(opModeIsActive()){
            RobotInitializers.Dashtelemetry.addData("tx",Math.toDegrees(camera.yellow.getTx(camera.yellow.getLargetDetectionId())));
            RobotInitializers.Dashtelemetry.addData("ty",Math.toDegrees(camera.yellow.getTy(camera.yellow.getLargetDetectionId())));

            RobotInitializers.Dashtelemetry.addData("camera x", GetSamplePosition.CameraRelativeToField(Localizer.getCurrentPosition()).x);
            RobotInitializers.Dashtelemetry.addData("camera y", GetSamplePosition.CameraRelativeToField(Localizer.getCurrentPosition()).y);

            SparkFunOTOS.Pose2D pos = GetSamplePosition.GetGlobalSamplePosition(Math.toDegrees(camera.yellow.getTx(camera.yellow.getLargetDetectionId())),Math.toDegrees(camera.yellow.getTy(camera.yellow.getLargetDetectionId())));
            RobotInitializers.Dashtelemetry.addData("x",pos.x);
            RobotInitializers.Dashtelemetry.addData("y",pos.y);
            RobotInitializers.Dashtelemetry.update();
            Localizer.Update();
        }
    }
}
