/**
 * @file reference_wrapper_demo.cpp
 * @brief Simple demonstration of std::reference_wrapper, std::ref and std::cref.
 *
 * Compile:
 *   g++ -std=c++17 -pthread -O2 reference_wrapper_demo.cpp -o reference_wrapper_demo
 *
 * Run:
 *   ./reference_wrapper_demo
 */

#include <iostream>
#include <functional>   // std::reference_wrapper, std::ref, std::cref
#include <vector>
#include <string>

// A simple class to show copying vs referencing
class Simple {
public:
    Simple(int v = 0) : value(v) { std::cout << "Simple ctor: " << value << '\n'; }
    Simple(const Simple& other) : value(other.value) {
        std::cout << "Simple copy ctor: " << value << '\n';
    }
    Simple& operator=(const Simple& other) {
        std::cout << "Simple copy assignment: " << other.value << '\n';
        value = other.value;
        return *this;
    }
    ~Simple() { std::cout << "Simple dtor: " << value << '\n'; }

    int get() const { return value; }
    void set(int v) { value = v; }

private:
    int value;
};

// Function that takes a reference (non-const)
void modify(Simple& s) {
    s.set(99);
    std::cout << "  Inside modify(), value = " << s.get() << '\n';
}

// Function that takes a const reference (read‑only)
void observe(const Simple& s) {
    std::cout << "  Inside observe(), value = " << s.get() << '\n';
}

int main() {
    std::cout << "=== Creating objects ===\n";
    Simple a(1);
    Simple b(2);
    Simple c(3);

    std::cout << "\n=== Storing references in a vector with std::ref ===\n";
    std::vector<std::reference_wrapper<Simple>> vec;
    vec.push_back(std::ref(a));   // non‑const wrapper
    vec.push_back(std::ref(b));
    vec.push_back(std::ref(c));

    std::cout << "\n=== Modifying through the vector (non‑const reference) ===\n";
    for (auto& wrapper : vec) {
        // wrapper is a reference_wrapper<Simple>; it can be used like Simple&
        modify(wrapper);   // implicit conversion to Simple&
    }

    std::cout << "\n=== Values after modification ===\n";
    std::cout << "a.value = " << a.get() << '\n';
    std::cout << "b.value = " << b.get() << '\n';
    std::cout << "c.value = " << c.get() << '\n';

    std::cout << "\n=== Using std::cref (const reference) ===\n";
    std::vector<std::reference_wrapper<const Simple>> const_vec;
    const_vec.push_back(std::cref(a));
    const_vec.push_back(std::cref(b));
    const_vec.push_back(std::cref(c));

    std::cout << "\n=== Observing through const vector (read‑only) ===\n";
    for (const auto& wrapper : const_vec) {
        observe(wrapper);   // implicit conversion to const Simple&
    }

    // Attempt to modify via const vector – will not compile if uncommented:
    // modify(const_vec[0]); // error: cannot bind non‑const lvalue reference to const

    std::cout << "\n=== Copying reference_wrapper does NOT copy the object ===\n";
    std::reference_wrapper<Simple> r1 = std::ref(a);
    std::reference_wrapper<Simple> r2 = r1;   // copies the wrapper, still refers to 'a'
    std::cout << "r1.get().get() = " << r1.get().get() << ", r2.get().get() = " << r2.get().get() << '\n';
    r2.get().set(55);   // modify via r2
    std::cout << "After r2.set(55): a.get() = " << a.get() << ", r1.get().get() = " << r1.get().get() << '\n';

    std::cout << "\n=== Using .get() explicitly ===\n";
    Simple& ref_to_a = r1.get();   // obtain actual reference
    ref_to_a.set(77);
    std::cout << "After explicit get() and set(77): a.get() = " << a.get() << '\n';

    std::cout << "\n=== End of demo ===\n";
    return 0;
}