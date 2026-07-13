#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <queue>

// -------------------- Paylaşılan veri yapısı --------------------
std::mutex mtx;
std::condition_variable cv;
std::queue<int> jobQueue;          // Üretici tarafından doldurulur, tüketici tarafından tüketilir
const size_t MAX_QUEUE_SIZE = 10;  // Kuyruk kapasitesi (opsiyonel)

// -------------------- Üretici fonksiyonu --------------------
void producer(int id) {
    for (int i = 0; i < 20; ++i) {
        // Üretim işlemi (örnek: 100ms bekleme)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int value = i + id * 100 + i;  // örnek veri

        {
            std::unique_lock<std::mutex> lk(mtx);
            // Kuyruk doluysa üretici bekler
            cv.wait(lk, []{ return jobQueue.size() < MAX_QUEUE_SIZE; });
            jobQueue.push(value);
            std::cout << "Producer " << id << " produced " << value
                      << ", queue size = " << jobQueue.size() << '\n';
        }
        cv.notify_one();   // Tüketiciyi uyandır (bir alan boşaldı)
    }
}

// -------------------- Tüketici fonksiyonu --------------------
void consumer(int id) {
    while (true) {
        int value;
        {
            std::unique_lock<std::mutex> lk(mtx);
            // Kuyruk boşsa tüketici bekler
            cv.wait(lk, []{ return !jobQueue.empty(); });

            // Üretici sonlandı ve kuyruk boşsa çık
            if (jobQueue.empty() && /* üretici sonlandmış mı */ false) break;

            value = jobQueue.front();
            jobQueue.pop();
            std::cout << "  Consumer " << id << " consumed " << value
                      << ", queue size = " << jobQueue.size() << '\n';
        }
        cv.notify_one();   // Üreticiyi uyandır (bir alan boşaldı)
        // Tüketim işlemi (örnek: 150ms bekleme)
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
}

// -------------------- main --------------------
int main() {
    std::thread prod1(producer, 1);
    std::thread prod2(producer, 2);
    std::thread cons1(consumer, 1);
    std::thread cons2(consumer, 2);

    prod1.join(); prod2.join();
    cons1.join(); cons2.join();

    std::cout << "All threads finished.\n";
    return 0;
}