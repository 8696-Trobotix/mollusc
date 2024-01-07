package org.firstinspires.ftc.teamcode.mollusc.auto;

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

    private ArrayList<Instruction> instructions = new ArrayList<>();
    private HashMap<String, Action> actions = new HashMap<>();
    private String[][] script;

    private int instructionPointer = 0;
    public boolean running = true;

    public Interpreter(Asset asset) throws ParityException {
        Configuration.useLinearOpMode();
        script = asset.getTokens();
    }
    public Interpreter(String[] rawLines) throws ParityException {
        Configuration.useLinearOpMode();
        script = Asset.tokenize(rawLines);
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
            }
            if (!actions.containsKey(instructionName)) {
                error(lineNum, "Unidentified instruction name / not declared. No instruction matches: " + instructionName);
            }
            instructions.add(new Instruction(instructionName, arguments, lineNum));
        }
    }

    public void run(boolean log) throws Exception {
        parse(script);
        int instructionCount = instructions.size();
        for (; running 
            && instructionPointer < instructionCount 
            && instructionPointer > -1; 
            ++instructionPointer
        ) {
            Instruction i = instructions.get(instructionPointer);
            if (log) {
                Mollusc.opMode.telemetry.log().add(i.line + ": " + i.name + ' ' + Arrays.toString(i.arguments));
            }
            actions.get(i.name).execute(i.arguments);
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
