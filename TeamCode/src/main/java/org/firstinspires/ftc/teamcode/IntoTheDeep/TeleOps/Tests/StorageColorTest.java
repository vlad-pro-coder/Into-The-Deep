package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class StorageColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            RobotInitializers.Dashtelemetry.addData("color", Intake.getStorageStatus());
            RobotInitializers.Dashtelemetry.addData("disatnce",Intake.rangesensor.getDist());
            RobotInitializers.Dashtelemetry.addData("isagaisttrapdoor",Intake.SampleReachedTrapDoor());
            RobotInitializers.Dashtelemetry.update();
        }
    }
}
