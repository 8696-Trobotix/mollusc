package org.firstinspires.ftc.teamcode.mollusc.auto;

public class Instruction {
    public String action;
    public int [] arguments;
    public int    line;
    public Instruction(String action, int [] arguments, int line) {
        this.action = action;
        this.arguments = arguments;
        this.line = line;
    }
}
