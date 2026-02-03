package com.gmail.frcteam1758.lib.util;

import java.util.Arrays;

public class RollingAverage {
    
    protected final int m_sz;
    protected int m_idx = 0;
    protected double m_total;
    protected double[] m_vals;

    public RollingAverage(int sz) {
        this(sz, 0);
    }
    public RollingAverage(int sz, double df) {
        m_sz = sz;
        m_vals = new double[sz]; for (int i = 0; i < sz; ++i) m_vals[i] = df;
        m_total = df * sz;
    }
    public RollingAverage(double[] vals) {
        m_sz = vals.length;
        m_vals = vals;
        m_total = 0;
        for (double i : vals) m_total += i;
    }

    public double get() { return m_total; }

    public double calculate(double val) {
        if (++m_idx == m_sz) m_idx = 0;
        m_total -= m_vals[m_idx];
        m_vals[m_idx] = val;
        m_total += val;
        return m_total;
    }

    public double getValue(int idx) {
        return m_vals[(m_idx - idx + m_sz) % m_sz];
    }

    public static RollingAverage copyOf(RollingAverage src, int sz) {
        return new RollingAverage(Arrays.copyOf(src.m_vals, sz));
    }
}
