package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.exception.AssetRetrievalException;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import java.util.stream.Collectors;
import java.util.LinkedList;

import java.io.InputStreamReader;
import java.io.BufferedReader;

public class Asset {

    private String data;

    public Asset(String path) throws AssetRetrievalException {
        try {
            InputStreamReader inputStreamReader = new InputStreamReader(
                Mollusc.instance().hardwareMap.appContext.getAssets().open(path)
            );
            BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
            data = bufferedReader.lines().collect(Collectors.joining("\n"));
            bufferedReader.close();
        }
        catch (Exception e) {
            data = null;
            throw new AssetRetrievalException(
                "Failed to open asset at: " + path + "\n" + e.getMessage(), 
                e.getCause()
            );
        }
    }

    public String getData() {
        return data;
    }
    public String[] getLines() {
        return data.split("\n");
    }
    // Non-blank tokens per line delineated by spaces.
    public String[][] getTokens() {
        String[] lines = getLines();
        return tokenize(lines);
    }

    public static String[][] tokenize(String[] lines) {
        String[][] tokens = new String[lines.length][];

        for (int i = 0; i < lines.length; ++i) {
            LinkedList<String> list = new LinkedList<>();
            String[] parsed = lines[i].split("[ \t]+");

            for (String token : parsed) {
                token = token.trim();
                if (!token.isEmpty()) {
                    list.add(token);
                }
            }

            tokens[i] = list.toArray(new String[0]);
        }

        return tokens;
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
