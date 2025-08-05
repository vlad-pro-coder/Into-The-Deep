package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample1pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample2pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample3pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample3;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.PutLastSamplePredefined.PutLastSamplePredefinedActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.SimultaniosSampleGrabingScoringActions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@Autonomous
@Config
public class Samples extends LinearOpMode {
    public enum RobotStates{
        PUTPRELOAD,
        RETRACT,
        PUTFIRSTSAMPLE,
        PUTSECONDSAMPLE,
        PUTTHIRDSAMPLE,
        GOTOSUBMERSIBLE,
    }
    public boolean WereInstructionGiven = false;
    public RobotStates robotstate = RobotStates.PUTPRELOAD;
    public double currentpredefinedsamples = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        Scheduler tasks = new Scheduler();
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample1pos,EXTENDO_sample1,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample2pos,EXTENDO_sample2,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample3pos,EXTENDO_sample3,1));
        tasks.AddAnotherScheduler(PutLastSamplePredefinedActions);


        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;


        while(opModeInInit()){
            RobotInitializers.clearCache(false);
            Outtake.setOverHeadPos(OVERHEAD_idle);
            Intake.Block();
            Outtake.openClaw();

            Lift.update();
            Extendo.update();
            Outtake.update();

        }

        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("robot states", robotstate);
            RobotInitializers.Dashtelemetry.addData("lift target pos", Lift.pidController.getTargetPosition());
            RobotInitializers.Dashtelemetry.addData("is Scheduler done", tasks.tasks.size());
            RobotInitializers.Dashtelemetry.addData("storage status", Intake.HasMixedTeamPiece());


            Chassis.update();
            Lift.update();
            Extendo.update();
            Outtake.update();
            tasks.update();
            Localizer.Update();

        }

    }
}
