package com.gmail.frcteam1758.lib.util;

public class BoundedSet<T> {
    
    protected T[] m_items;

    protected int m_idx;

    /**constructs a BoundedSet from an array*/
    public BoundedSet(T[] p_items) {
        m_items = p_items;
    }

    /**
     * attempts to increase the target index
     * 
     * @param p_numTimes how much to increase
     */
    public void increase(int p_numTimes) {

        assert p_numTimes >= 0;

        m_idx = m_idx + p_numTimes > m_items.length? m_items.length : m_idx + p_numTimes;
    }

    /**
     * attempts to decrease the target index
     * 
     * @param p_numTimes how much to increase
     */
    public void decrease(int p_numTimes) {

        assert p_numTimes >= 0;

        m_idx = m_idx - p_numTimes > 0? m_idx - p_numTimes : 0;
    }

    /** gets the item at the target index */
    public T get() {
        return m_items[m_idx];
    }

    /**
     * sets the target index to zero and returns the first item in the set
     * 
     * @return the first item in the set
    */
    public T toBeginning() {
        m_idx = 0;
        return m_items[0];
    }

    /**
     * sets the target index to the end and returns the last item in the set
     * 
     * @return the last item in the set
    */
    public T toEnd() {
        m_idx = m_items.length;
        return m_items[m_idx];
    }

}
