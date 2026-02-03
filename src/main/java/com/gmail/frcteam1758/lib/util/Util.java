package com.gmail.frcteam1758.lib.util;

public class Util {
    
    public static double minAll(Double a, Double... b) {

        if (b.length == 0) {
            return a;
        }

        return minAllImpl(0, a, b);
    }

    private static double minAllImpl(int depth, Double prev, Double... args) {

        if (args.length == 2 + depth) {

            return Math.min(prev, args[depth]);
        }

        return minAllImpl(depth + 1, Math.max(prev, args[depth]), args);
    }
}
