package frc.lib.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import us.hebi.quickbuf.ProtoMessage;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;

/**
 * Drop-in replacement for {@link Logger}'s {@code recordOutput} overloads. Output logging is
 * disabled on a real robot unless {@link Constants#tuningMode} is enabled. Sim and replay always
 * log.
 */
public final class OutputLogger {
    private static final boolean ENABLED =
            Constants.currentMode != Constants.Mode.REAL || Constants.tuningMode;

    private OutputLogger() {}

    public static void recordOutput(String key, byte[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, byte[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, boolean value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, BooleanSupplier value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, boolean[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, boolean[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, int value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, IntSupplier value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, int[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, int[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, long value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, LongSupplier value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, long[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, long[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, double value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, DoubleSupplier value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, double[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, double[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, String value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, String[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, String[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, LoggedMechanism2d value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, Color value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float value, Unit unit) {
        if (ENABLED) {
            Logger.recordOutput(key, value, unit);
        }
    }

    public static void recordOutput(String key, float value, String unit) {
        if (ENABLED) {
            Logger.recordOutput(key, value, unit);
        }
    }

    public static void recordOutput(String key, double value, Unit unit) {
        if (ENABLED) {
            Logger.recordOutput(key, value, unit);
        }
    }

    public static void recordOutput(String key, double value, String unit) {
        if (ENABLED) {
            Logger.recordOutput(key, value, unit);
        }
    }

    public static <E extends Enum<E>> void recordOutput(String key, E value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <E extends Enum<E>> void recordOutput(String key, E[] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <E extends Enum<E>> void recordOutput(String key, E[][] value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <U extends Unit> void recordOutput(String key, Measure<U> value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <T> void recordOutput(String key, Struct<T> struct, T value) {
        if (ENABLED) {
            Logger.recordOutput(key, struct, value);
        }
    }

    public static <T> void recordOutput(String key, Struct<T> struct, T... values) {
        if (ENABLED) {
            Logger.recordOutput(key, struct, values);
        }
    }

    public static <T> void recordOutput(String key, Struct<T> struct, T[][] values) {
        if (ENABLED) {
            Logger.recordOutput(key, struct, values);
        }
    }

    public static <T, M extends ProtoMessage<?>> void recordOutput(
            String key, Protobuf<T, M> proto, T value) {
        if (ENABLED) {
            Logger.recordOutput(key, proto, value);
        }
    }

    public static <T extends WPISerializable> void recordOutput(String key, T value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <T extends StructSerializable> void recordOutput(String key, T... values) {
        if (ENABLED) {
            Logger.recordOutput(key, values);
        }
    }

    public static <T extends StructSerializable> void recordOutput(String key, T[][] values) {
        if (ENABLED) {
            Logger.recordOutput(key, values);
        }
    }

    public static <R extends Record> void recordOutput(String key, R value) {
        if (ENABLED) {
            Logger.recordOutput(key, value);
        }
    }

    public static <R extends Record> void recordOutput(String key, R... values) {
        if (ENABLED) {
            Logger.recordOutput(key, values);
        }
    }

    public static <R extends Record> void recordOutput(String key, R[][] values) {
        if (ENABLED) {
            Logger.recordOutput(key, values);
        }
    }
}
