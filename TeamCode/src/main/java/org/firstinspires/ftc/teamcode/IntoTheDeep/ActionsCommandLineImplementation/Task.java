package org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation;


public abstract class Task {
    private boolean initialized = false;

    public final boolean Run() {
        if (!initialized) {
            Actions();
            initialized = true;
        }
        return Conditions();
    }

    // You override this instead of Run()
    protected abstract void Actions();

    // Called repeatedly after setup
    protected abstract boolean Conditions();
}
