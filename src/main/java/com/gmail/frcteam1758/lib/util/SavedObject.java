package com.gmail.frcteam1758.lib.util;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.net.URI;

/**
 * class that saves objects to a {@link File}. The {@link #save(Serializable)} method
 * must be called to save an object (this can be done in {@code Robot.disabledInit())}.
 * There are two options for loading:
 * <p>
 * {@link #loadOrThrow()} attempts to load the object, but throws an {@link Exception}
 * if anything goes wrong. This allows the user to specify the behavior.
 * <p>
 * {@link #loadSafely()} attempts to load the object, but returns the default instead
 * if anything goes wrong.
 */
public class SavedObject<T extends Serializable> {
    
    protected File m_file;

    protected T m_default;

    /**
     * constructs a SavedObject from a filename and a default
     * 
     * @param p_filename the filename to use for loading and saving
     * @param p_default what to return from {@link #loadSafely()} if an
     * {@link Exception} occurs
     */
    public SavedObject(String p_filename, T p_default) {
        m_file = new File(p_filename);
        m_default = p_default;
    }

    /**
     * constructs a SavedObject from a {@link File} and a default
     * 
     * @param p_file the {@link File} to use for loading and saving
     * @param p_default what to return from {@link #loadSafely()} if an
     * {@link Exception} occurs
     */
    public SavedObject(File p_file, T p_default) {
        m_file = p_file;
        m_default = p_default;
    }

    /**
     * constructs a SavedObject from a {@link URI} and a default
     * 
     * @param p_uri the filepath to use for loading and saving
     * @param p_default what to return from {@link #loadSafely()} if an
     * {@link Exception} occurs
     */
    public SavedObject(URI p_uri, T p_default) {
        m_file = new File(p_uri);
        m_default = p_default;
    }

    /**
     * Saves an object to the file.
     * 
     * @param p_object the object to save
     * 
     * @throws IOException {@link Exception} throwable by all IO operations
     * @throws FileNotFoundException if the {@link File}/filename used to construct this {@code SavedObject}
     * cannot be found
     */
    public void save(T p_object) throws IOException, FileNotFoundException {

        FileOutputStream   l_fout = null;
        ObjectOutputStream l_oout = null;

        try {
            l_fout = new FileOutputStream(m_file);
            l_oout = new ObjectOutputStream(l_fout);

            l_oout.writeObject(p_object);
        }
        finally {
            try {
                if (l_oout != null) l_oout.close();
                if (l_fout != null) l_fout.close();
            }
            catch (IOException e_ioe) {
                System.out.println(
                    "Resource Leak: Unable to close OutputStream(s) while saving" +
                    "an Object to" + m_file.getAbsolutePath()
                );
            }
        }
    }

    /**
     * loads the object from the file, or raises an {@link Exception}
     * 
     * @return the object
     * 
     * @throws IOException {@link Exception} throwable by all IO operations
     * @throws ClassNotFoundException if the object was saved as a different class
     * or version of a class
     * @throws FileNotFoundException if the {@link File}/filename used to construct this {@code SavedObject}
     * cannot be found
     */
    @SuppressWarnings("unchecked") // cast to type parameter "T"
    public T loadOrThrow() throws IOException, ClassNotFoundException, FileNotFoundException {

        T l_r;

        FileInputStream l_fin = null;
        ObjectInputStream l_oin = null;

        try {
            l_fin = new FileInputStream(m_file);
            l_oin = new ObjectInputStream(l_fin);
        
            l_r = (T)l_oin.readObject();
        }
        finally {
            try {
                l_oin.close();
                l_fin.close();
            }
            catch (IOException e_ioe) {
                System.out.println(
                    "Resource Leak: Unable to close InputStream(s) while loading" +
                    "an Object from" + m_file.getAbsolutePath()
                );
            }
        }
        return l_r;
    }

    @SuppressWarnings("unchecked")
    public T loadSafely() {

        T l_r;

        FileInputStream   l_fin = null;
        ObjectInputStream l_oin = null;

        try {
            l_fin = new FileInputStream(m_file);
            l_oin = new ObjectInputStream(l_fin);

            l_r = (T)l_oin.readObject();
        }
        catch (Exception e_ge) {
            l_r = m_default;
        }
        finally {
            try {
                if (l_oin != null) l_oin.close();
                if (l_fin != null) l_fin.close();
            }
            catch (IOException e_ioe) {
                System.out.println(
                    "Resource Leak: Unable to close InputStream(s) while loading" +
                    "an Object from" + m_file.getAbsolutePath()
                );
            }
        }
        return l_r;
    }
}
