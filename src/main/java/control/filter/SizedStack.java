package control.filter;

import java.util.ArrayList;
import java.util.List;

public class SizedStack<T> {
    private final int capacity;
    private final List<T> data;

    public SizedStack(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity must be positive");
        }
        this.capacity = capacity;
        this.data = new ArrayList<>(capacity);
    }

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

    public T peek() {
        if (data.isEmpty()) {
            return null;
        }
        return data.get(data.size() - 1);
    }

    public T get(int index) {
        if (index < 0 || index >= data.size()) {
            return null;
        }
        return data.get(index);
    }

    public int size() {
        return data.size();
    }

    public List<T> toList() {
        return new ArrayList<>(data);
    }
}
