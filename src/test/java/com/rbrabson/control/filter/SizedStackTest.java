package com.rbrabson.control.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SizedStackTest {
    @Test
    void retainsNewestValuesAtCapacity() {
        SizedStack<Integer> stack = new SizedStack<Integer>(2);
        stack.push(1);
        stack.push(2);
        stack.push(3);

        assertEquals(2, stack.size());
        assertEquals(2, stack.get(0));
        assertEquals(3, stack.peek());
        assertEquals(2, stack.toList().size());
        assertNull(stack.get(-1));
    }

    @Test
    void safelyHandlesZeroCapacity() {
        SizedStack<Integer> stack = new SizedStack<Integer>(0);
        stack.push(1);
        assertEquals(0, stack.size());
    }
}
