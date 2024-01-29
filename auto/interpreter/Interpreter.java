package org.firstinspires.ftc.teamcode.mollusc.auto.interpreter;

import org.firstinspires.ftc.teamcode.mollusc.exception.ScriptParseException;
import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.Configuration;
import org.firstinspires.ftc.teamcode.mollusc.utility.Asset;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Arrays;

public class Interpreter {
    private static final String IDENTIFIER_DELIMITER = "-", INTEGER_MANGLING = "i", DOUBLE_MANGLING = "f", STRING_MANGLING = "s";
    public static final Action NONE_ACTION = (Object[] args) -> {};

    private ArrayList<Instruction> instructions = new ArrayList<>();
    private HashMap<String, Action> actions = new HashMap<>();
    private String[][] script;

    private int instructionPointer = 0;
    private boolean running = true;

    public Interpreter(Asset asset) throws ParityException {
        Configuration.useLinearOpMode();
        script = asset.getTokens();
        parse(script);
    }
    public Interpreter(String[] rawLines) throws ParityException {
        Configuration.useLinearOpMode();
        script = Asset.tokenize(rawLines);
        parse(script);
    }

    public void parse(String[][] tokens) throws ScriptParseException {
        int lineNum = 0;
        for (String[] tokenLine : tokens) {
            ++lineNum;
            if (tokenLine.length == 0 || tokenLine[0].startsWith("//")) {
                continue;
            }
            if (tokenLine[0].contains("-")) {
                error(lineNum, "Instruction identifier must not contain IDENTIFIER_DELIMITER: " + tokenLine[0]);
            }

            String instructionName = tokenLine[0] + IDENTIFIER_DELIMITER;
            Object[] arguments = new Object[tokenLine.length - 1];

            String strInstructionName = tokenLine[0] + IDENTIFIER_DELIMITER;
            Object[] strArguments = new Object[tokenLine.length - 1];

            for (int i = 1; i < tokenLine.length; ++i) {
                String token = tokenLine[i];
                Object arg;
                if (isInteger(token)) {
                    instructionName += INTEGER_MANGLING;
                    arg = Integer.parseInt(token);
                } else if (isDouble(token)) {
                    instructionName += DOUBLE_MANGLING;
                    arg = Double.parseDouble(token);
                } else {
                    instructionName += STRING_MANGLING;
                    arg = tokenLine[i];
                }
                arguments[i - 1] = arg;

                strInstructionName += STRING_MANGLING;
                strArguments[i - 1] = tokenLine[i];
            }
            if (actions.containsKey(instructionName)) {
                instructions.add(new Instruction(instructionName, arguments, lineNum));
            } else if (actions.containsKey(strInstructionName)) {
                instructions.add(new Instruction(strInstructionName, strArguments, lineNum));
            } else {
                error(lineNum, "Unidentified instruction name / not declared. No instruction matches: " + instructionName);
            }
        }
    }

    public void run(boolean log) throws Exception {
        int instructionCount = instructions.size();
        while (
            running 
            && instructionPointer < instructionCount 
            && instructionPointer > -1
        ) {
            Instruction i = instructions.get(instructionPointer);
            if (log) {
                Mollusc.opMode.telemetry.log().add(i.line + ": " + i.name + ' ' + Arrays.toString(i.arguments));
            }
            actions.get(i.name).execute(i.arguments);
            ++instructionPointer;
        }
    }

    public void jumpTo(int instructionNum) {
        instructionPointer = instructionNum - 1;
    }

    public void advanceTo(String instructionIdentifier) {
        String auxIdentifier = instructionIdentifier + IDENTIFIER_DELIMITER;
        for (; !instructions.get(instructionPointer).name.startsWith(auxIdentifier); ++instructionPointer);
        --instructionPointer;
    }

    public void backTo(String instructionIdentifier) {
        String auxIdentifier = instructionIdentifier + IDENTIFIER_DELIMITER;
        for (; !instructions.get(instructionPointer).name.startsWith(auxIdentifier); --instructionPointer);
        --instructionPointer;
    }

    private void error(int lineNum, String msg) throws RuntimeException {
        throw new RuntimeException("Script error on line: " + lineNum + " --> " + msg);
    }

    public static boolean isInteger(String s) {
        try {
            Integer.parseInt(s);
        } catch (NullPointerException|NumberFormatException e) {
            return false;
        }
        return true;
    }

    public static boolean isDouble(String s) {
        try {
            Double.parseDouble(s);
        } catch (NullPointerException|NumberFormatException e) {
            return false;
        }
        return true;
    }

    public void register(String identifier, Action action, Class<?> ...types) {
        String name = identifier + IDENTIFIER_DELIMITER;
        for (Class<?> type : types) {
            if (type.equals(Integer.class)) {
                name += INTEGER_MANGLING;
            } else if (type.equals(Double.class)) {
                name += DOUBLE_MANGLING;
            } else {
                name += STRING_MANGLING;
            }
        }
        actions.put(name, action);
    }

    public ArrayList<Instruction> getInstructions() {
        return instructions;
    }

    public HashMap<String, Action> getActions() {
        return actions;
    }

    public String[][] getScriptTokens() {
        return script;
    }

    // Use `jumpTo` to set the instruction pointer.
    public int getInstructionPointer() {
        return instructionPointer;
    }

    public boolean getRunning() {
        return running;
    }
    public boolean setRunning(boolean running) {
        this.running = running;
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
