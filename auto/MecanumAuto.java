/*
Mecanum Auto

Autonomous configuration class.

vIX-XXX-XXIII
*/

package org.firstinspires.ftc.teamcode.mollusc.auto;

import java.util.stream.Collectors;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.mollusc.drivetrain.MecanumRobotCentric;
import org.firstinspires.ftc.teamcode.mollusc.drivetrain.MecanumFieldCentric;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumAuto <DrivetrainType extends Drivetrain> {

    public DrivetrainType drivetrain;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public MecanumAuto(DrivetrainType drivetrain, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        if (telemetry != null) {
            telemetry.log().add("Initialized autonomous with drivetrain type: " + 
                (drivetrain instanceof MecanumRobotCentric ? "robot centric." : "") + 
                (drivetrain instanceof MecanumFieldCentric ? "field centric." : "")
            );
            telemetry.update();
        }
    }

    // Script Parser and Executor / Interpreter
    /*
    Script:
        Each line contains exactly one command and its parameters.
        The command consists of a name without spaces.
        It is then followed by a space and its arguments (space separated).
            "Command 0 9 8 7 6"
        The arguments must be integers, for now.
        Lines starting with "//" are comments, and will be ignored.
        An instruction is a term describing a command followed by its arguments.
    Parser:
        Parsing of the entire script is done prior to execution.
        Leading and trailing whitespace for each line is removed.
        The parser will notify of an error if:
            Extra spaces are found within a command.
            The number of arguments do not match the parameter count determined by command registration.
            The command is not registered (see below).
            An argument is invalid.
    Run:
        Call to parse the script.
        Returns false upon parse failure or if runState is set to false by other Java, and true upon success.
    
    Registering Commands (advanced):
        Any commands used by the script must first be registered in the HashMap `actions`.
            This includes if-else logic and loops.
        A command is registered by adding its name and parameter count, separated by a '-', to the HashMap.
            Along with a lambda expression that expects an integer array dictating what it does.
            The lambda must return an integer exit code, which may be used by other registered commands. This value can be anything.
            "
            actions.put("Command-2", (int [] args) -> {
                telemetry.log().add("Executing Command " + args[0] + ' ' + args[1]);
                return 0;
            });
            "
    Misc:
        A function called `jumpTo` which accepts a single integer will move script execution to that instruction.
            All subsequent executions will be from that instruction.
            Use this function with care, as it can easily create infinite loops and go out of indexing bounds.
            Note: Instructions are indexed from zero.
        Note: Disables telemetry auto clear. Does not disable upon exit.
        Note: If you change the Instruction ArrayList you must update the instructionCount variable!
    */

    public HashMap <String, Action> actions = new HashMap <> ();
    public ArrayList <Instruction> instructions = null;
    public int instructionCount = -1, instructionNum = 0, returnCode = 0;
    public boolean runState = true;
    public boolean run(String script) {
        telemetry.setAutoClear(false);
        instructions = parse(script);
        if (instructions == null) return false;
        instructionCount = instructions.size();
        for (; runState && instructionNum < instructionCount && instructionNum > -1; ++instructionNum) {
            Instruction i = instructions.get(instructionNum);
            telemetry.log().add(i.line + ": " + 
                                i.action.substring(0, i.action.length() - 2) + 
                                ' ' + Arrays.toString(i.arguments));
            returnCode = actions.get(i.action).execute(i.arguments);
            telemetry.update();
        }
        return runState;
    }

    public ArrayList <Instruction> parse(String script) {
        ArrayList <Instruction> instructions = new ArrayList <> ();
        String [] lines = script.split("\n");
        int lineNum = 0;
        for (String line : lines) {
            ++lineNum;
            line = line.trim();
            if (line.isEmpty() || line.startsWith("//")) continue;
            String [] v = line.split(" ");
            for (String i : v) if (i.isEmpty()) {
                error(lineNum, "Extra spaces detected.");
                return null;
            }
            v[0] += "-" + (v.length - 1);
            if (!actions.containsKey(v[0])) {
                error(lineNum, "Unidentified instruction name / not declared:" + v[0]);
                return null;
            }
            int [] arguments = new int [v.length - 1];
            for (int i = 1; i < v.length; ++i) {
                int arg;
                try {
                    arg = Integer.parseInt(v[i]);
                }
                catch (Exception e) {
                    error(lineNum, "Invalid argument passed as parameter. Must be an integer.");
                    return null;
                }
                arguments[i - 1] = arg;
            }
            instructions.add(new Instruction (v[0], arguments, lineNum));
        }
        return instructions;
    }

    public void jumpTo(int lineNum) {
        instructionNum = lineNum - 2;
    }

    public void error(int lineNum, String msg) {
        telemetry.log().add("Script error on line: " + lineNum + " --> " + msg);
        telemetry.update();
    }

    // For loading script stored in src/main/assets.
    public String loadScript(String path) {
        
        // Reference: https://stackoverflow.com/questions/309424/how-do-i-read-convert-an-inputstream-into-a-string-in-java
        // Note: The app's context is required to get assets. Conveniently, the SDK provides a medium for this through the hardwareMap.

        try {
            return new BufferedReader(
                new InputStreamReader(hardwareMap.appContext.getAssets().open(path))).lines().collect(Collectors.joining("\n")
            );
        }
        catch (IOException e) {
            runState = false;
            error(0, "Could not open script file.");
            return "";
        }
    }
}

/*
Copyright 2023 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
