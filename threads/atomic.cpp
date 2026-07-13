/*
 * Atomic Variables Tutorial in C++
 * ================================
 *
 * This tutorial covers the basics of std::atomic in C++, which provides
 * atomic operations for concurrent programming without the need for mutexes
 * in simple cases.
 *
 * Topics covered:
 *   1. What are atomic variables and why we need them
 *   2. Basic usage: atomic types and operations
 *   3. Memory ordering concepts
 *   4. Practical examples
 *   5. Best practices
 *
 * Compile with: g++ -std=c++11 -pthread atomic.cpp -o atomic
 * Run: ./atomic
 */

#include <iostream>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>

// Example 1: Basic atomic counter
// -------------------------------
// Demonstrates the problem with non-atomic variables and the fix with atomic.
void example_basic_counter() {
    std::cout << "\n=== Example 1: Basic Atomic Counter ===\n";

    // Non-atomic version (for comparison) - THIS IS BROKEN
    {
        int non_atomic_counter = 0;
        auto increment_task = [&non_atomic_counter]() {
            for (int i = 0; i < 1000; ++i) {
                ++non_atomic_counter;  // Not atomic: read-modify-write race condition
            }
        };

        std::thread t1(increment_task);
        std::thread t2(increment_task);
        t1.join();
        t2.join();

        std::cout << "Non-atomic counter result: " << non_atomic_counter
                  << " (expected 2000, but likely less due to race)\n";
    }

    // Atomic version - FIXED
    {
        std::atomic<int> atomic_counter{0};
        auto increment_task = [&atomic_counter]() {
            for (int i = 0; i < 1000; ++i) {
                ++atomic_counter;  // Atomic operation: thread-safe
            }
        };

        std::thread t1(increment_task);
        std::thread t2(increment_task);
        t1.join();
        t2.join();

        std::cout << "Atomic counter result: " << atomic_counter.load()
                  << " (expected 2000, guaranteed correct)\n";
    }
}


int main() {
    std::cout << "C++ Atomic Variables Tutorial\n";
    std::cout << "=============================\n";

    example_basic_counter();

    std::cout << "\nTutorial completed.\n";
    return 0;
}