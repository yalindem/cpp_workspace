/**
 * @file sensor_demo.cpp
 * @brief Demo of multi‑threaded sensor read/write using only mutexes (no condition_variable).
 *
 * This version replaces the condition‑variable signaling with a simple flag
 * (has_new_data_) protected by each sensor's mutex. The consumer thread
 * periodically tries to lock each sensor's mutex, reads the value if new
 * data is available, and clears the flag.
 *
 * Compile (C++17):
 *   g++ -std=c++17 -pthread -O2 sensor_demo.cpp -o sensor_demo
 *
 * Run:
 *   ./sensor_demo
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <random>
#include <iomanip>   // std::put_time
#include <sstream>   // std::ostringstream

// ---------------------------------------------------------------------------
// Sensor class – protects its value and a "new data" flag with a mutex
// ---------------------------------------------------------------------------
class Sensor {
public:
    explicit Sensor(const std::string& name)
        : name_(name), value_(0.0), has_new_data_(false) {}

    // Producer: called by a thread that simulates reading the sensor
    void update(double new_value) {
        std::lock_guard<std::mutex> lock(mtx_);
        value_ = new_value;
        has_new_data_ = true;
    }

    // Consumer: attempts to acquire the mutex and, if new data is present,
    // copies it out and clears the flag. Returns true if data was retrieved.
    bool try_get_data(double& out_value) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_new_data_) {
            return false;
        }
        out_value = value_;
        has_new_data_ = false;
        return true;
    }

    const std::string& name() const { return name_; }

private:
    std::string name_;
    double value_;
    std::mutex mtx_;
    bool has_new_data_; // true when value_ holds fresh data
};

// ---------------------------------------------------------------------------
// Helper: generate a random double in [min, max]
// ---------------------------------------------------------------------------
double random_double(double min, double max, std::mt19937& rng) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

// ---------------------------------------------------------------------------
// Producer thread function – simulates a sensor delivering periodic readings
// ---------------------------------------------------------------------------
void sensor_producer(Sensor& sensor,
                     std::chrono::milliseconds period,
                     std::atomic<bool>& stop_flag,
                     int seed_offset) {
    std::mt19937 rng(std::random_device{}() + seed_offset);
    // Example ranges per sensor type; we encode a simple mapping via name
    double min_val = 0.0, max_val = 100.0;
    if (sensor.name().find("Temp") != std::string::npos) {
        min_val = -10.0; max_val = 50.0; // °C
    } else if (sensor.name().find("Pressure") != std::string::npos) {
        min_val = 950.0; max_val = 1050.0; // hPa
    } else if (sensor.name().find("Humidity") != std::string::npos) {
        min_val = 0.0; max_val = 100.0; // %RH
    }

    while (!stop_flag.load(std::memory_order_acquire)) {
        double val = random_double(min_val, max_val, rng);
        sensor.update(val);

        // Log production (optional, can be noisy)
        {
            static std::mutex io_mtx;
            std::lock_guard<std::mutex> io_lock(io_mtx);
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::tm tm{};
#if defined(_WIN32) || defined(_WIN64)
            localtime_s(&tm, &time_t);
#else
            localtime_r(&time_t, &tm);
#endif
            std::cout << "[" << std::put_time(&tm, "%H:%M:%S")
                      << "] Producer " << sensor.name()
                      << ": " << std::fixed << std::setprecision(2)
                      << val << " units\n";
        }

        std::this_thread::sleep_for(period);
    }

    {
        static std::mutex io_mtx;
        std::lock_guard<std::mutex> io_lock(io_mtx);
        std::cout << "[" << sensor.name() << "] Producer stopping.\n";
    }
}

// ---------------------------------------------------------------------------
// Consumer thread function – reads from all sensors and aggregates
// ---------------------------------------------------------------------------
void consumer_thread(const std::vector<std::reference_wrapper<Sensor>>& sensors,
                     std::atomic<bool>& stop_flag) {
    std::vector<double> latest_values(sensors.size(), 0.0);
    size_t update_counter = 0;

    while (!stop_flag.load(std::memory_order_acquire)) {
        bool any_updated = false;
        for (size_t i = 0; i < sensors.size(); ++i) {
            Sensor& sensor = sensors[i].get();
            double val;
            if (sensor.try_get_data(val)) {
                latest_values[i] = val;
                any_updated = true;
            }
        }

        if (any_updated) {
            ++update_counter;
            // Simple aggregate: compute average of latest readings
            double sum = 0.0;
            for (double v : latest_values) sum += v;
            double avg = sum / static_cast<double>(latest_values.size());

            static std::mutex io_mtx;
            {
                std::lock_guard<std::mutex> io_lock(io_mtx);
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::tm tm{};
#if defined(_WIN32) || defined(_WIN64)
                localtime_s(&tm, &time_t);
#else
                localtime_r(&time_t, &tm);
#endif
                std::cout << "[" << std::put_time(&tm, "%H:%M:%S")
                          << "] Consumer update #" << update_counter
                          << " – Avg = " << std::fixed << std::setprecision(2)
                          << avg << " (";
                for (size_t i = 0; i < sensors.size(); ++i) {
                    std::cout << sensors[i].get().name() << "="
                              << std::fixed << std::setprecision(2)
                              << latest_values[i];
                    if (i + 1 < sensors.size()) std::cout << ", ";
                }
                std::cout << ")\n";
            }
        }

        // Small sleep to avoid busy‑loop if sensors are slow
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    {
        static std::mutex io_mtx;
        std::lock_guard<std::mutex> io_lock(io_mtx);
        std::cout << "[Consumer] Stopping.\n";
    }
}

// ---------------------------------------------------------------------------
// Main – orchestrates producers and consumer
// ---------------------------------------------------------------------------
int main() {
    std::cout << "=== Multi‑Threaded Sensor Demo (mutex‑only) ===\n";

    // Define a few simulated sensors
    Sensor temp_sensor("Temperature");
    Sensor pressure_sensor("Pressure");
    Sensor humidity_sensor("Humidity");

    std::vector<std::reference_wrapper<Sensor>> sensors = {
        temp_sensor, pressure_sensor, humidity_sensor
    };

    std::atomic<bool> stop_flag{false};

    // Launch producer threads with different periods
    std::vector<std::thread> producers;
    producers.emplace_back(sensor_producer, std::ref(temp_sensor),
                           std::chrono::milliseconds(500), std::ref(stop_flag), 1);
    producers.emplace_back(sensor_producer, std::ref(pressure_sensor),
                           std::chrono::milliseconds(700), std::ref(stop_flag), 2);
    producers.emplace_back(sensor_producer, std::ref(humidity_sensor),
                           std::chrono::milliseconds(400), std::ref(stop_flag), 3);

    // Launch consumer thread
    std::thread consumer(consumer_thread, std::cref(sensors), std::ref(stop_flag));

    // Let the demo run for a fixed duration (e.g., 10 seconds)
    const std::chrono::seconds demo_duration(10);
    std::this_thread::sleep_for(demo_duration);

    // Request shutdown
    std::cout << "\n[Main] Stopping all threads...\n";
    stop_flag.store(true, std::memory_order_release);

    // Join threads
    for (auto& th : producers) {
        if (th.joinable()) th.join();
    }
    if (consumer.joinable()) consumer.join();

    std::cout << "[Main] Demo finished.\n";
    return 0;
}