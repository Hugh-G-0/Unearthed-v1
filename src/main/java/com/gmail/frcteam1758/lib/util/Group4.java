package com.gmail.frcteam1758.lib.util;

/**
 * Stores exactly 4 {@code T}s
 * Can be using in  constructors/functions in place of an
 * array to guarantee that 4 objects are provided
 */
public final class Group4<T> {
    
    T m_1, m_2, m_3, m_4;

    public Group4(
        T p_1,
        T p_2,
        T p_3,
        T p_4
    ) {
        m_1 = p_1;
        m_2 = p_2;
        m_3 = p_3;
        m_4 = p_4;
    }

    public T get1() { return m_1; }
    public T get2() { return m_2; }
    public T get3() { return m_3; }
    public T get4() { return m_4; }
}
