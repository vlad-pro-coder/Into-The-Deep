package org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation;


public abstract class Task {

    public boolean RanOnce = false;
    public final boolean Run() {
        if(!RanOnce){
            RanOnce = true;
            Actions();
        }
        boolean result = Conditions();
        if(result)
            RanOnce = false;
        return result;
    }

    // You override this instead of Run()
    protected abstract void Actions();

    // Called repeatedly after setup
    protected abstract boolean Conditions();
}
