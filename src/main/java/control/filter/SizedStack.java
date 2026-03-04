package control.filter;

import java.util.ArrayList;
import java.util.List;

/**
 * A fixed-size stack that maintains the most recent N values. When a new value
 * is pushed and the stack is at capacity, the oldest value is removed to make
 * room for the new value. This can be useful for maintaining a history of
 * recent values for filtering or analysis purposes.
 *
 * @param <T> The type of values stored in the stack.
 */
public class SizedStack<T> {
    private final int capacity;
    private final List<T> data;

    /**
     * Creates a new SizedStack with the specified capacity.
     *
     * @param capacity The maximum number of values the stack can hold. Must be
     *                 positive.
     */
    public SizedStack(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be positive");
        }
        this.capacity = capacity;
        this.data = new ArrayList<>(capacity);
    }

    /**
     * Pushes a new value onto the stack. If the stack is at capacity, the oldest
     * value will be removed to make room for the new value.
     *
     * @param value The value to push onto the stack.
     */
    public void push(T value) {
        if (data.size() < capacity) {
            data.add(value);
        } else {
            for (int i = 1; i < data.size(); i++) {
                data.set(i - 1, data.get(i));
            }
            data.set(data.size() - 1, value);
        }
    }

    /**
     * Returns the most recent value on the stack without removing it. If the stack
     * is empty, returns null.
     *
     * @return The most recent value on the stack, or null if the stack is empty.
     */
    public T peek() {
        if (data.isEmpty()) {
            return null;
        }
        return data.get(data.size() - 1);
    }

    /**
     * Returns the value at the specified index in the stack, where index 0 is the
     * oldest value and index size() - 1 is the most recent value. If the index is
     * out of bounds, returns null.
     *
     * @param index The index of the value to retrieve, where 0 is the oldest value
     *              and size() - 1 is the most recent value.
     * @return The value at the specified index, or null if the index is out of
     *         bounds.
     */
    public T get(int index) {
        if (index < 0 || index >= data.size()) {
            return null;
        }
        return data.get(index);
    }

    /**
     * Returns the number of values currently stored in the stack, which will be
     * less than or equal to the capacity.
     *
     * @return The number of values currently stored in the stack.
     */
    public int size() {
        return data.size();
    }

    /**
     * Returns a list of the values currently stored in the stack, ordered from
     * oldest to most recent.
     *
     * @return A list of the values currently stored in the stack, ordered from
     *         oldest to most recent.
     */
    public List<T> toList() {
        return new ArrayList<>(data);
    }
}
