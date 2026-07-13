#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>

// ============================================================
// 1. Basit bir thread oluşturma ve çalıştırma
// ============================================================
void hello_from_thread() {
    std::cout << "Merhaba! Ben bir thread'iden geliyorum. ID: "
              << std::this_thread::get_id() << std::endl;
}
void demo_create_and_run_thread() {
    std::cout << "=== 1. Thread Oluşturma ve Çalıştırma ===\n";
    std::thread t(hello_from_thread);          // thread nesnesi oluştur, fonksiyonu çalıştır
    std::cout << "Ana thread: Thread başlatıldı, ben başka işler yapabilirim.\n";
    t.join();                                 // thread'in bitmesini bekle
    std::cout << "Ana thread: Thread bitti, devam ediyorum.\n\n";
}

// ============================================================
// 2. Paylaşılan veri ve race condition (veri yarışı)
// ============================================================
int shared_counter = 0; // tüm thread'ler tarafından erişilebilir
void increment_without_sync() {
    for (int i = 0; i < 10000; ++i) {
        ++shared_counter; // Bu işlem atomik DEĞİL! (okuma‑artma‑yazma)
    }
}
void demo_race_condition() {
    std::cout << "=== 2. Race Condition (Veri Yarışı) Gösterimi ===\n";
    shared_counter = 0;
    std::thread t1(increment_without_sync);
    std::thread t2(increment_without_sync);
    t1.join(); t2.join();
    std::cout << "Beklenen değer: 20000\n";
    std::cout << "Gerçek değer (race condition nedeniyle likely farklı): " << shared_counter << "\n\n";
}

// ============================================================
// 3. Mutex ile koruma (temel kilitleme)
// ============================================================
std::mutex mtx;
void increment_with_mutex() {
    for (int i = 0; i < 10000; ++i) {
        mtx.lock();        // kilidi al
        ++shared_counter;  // críticos bölge
        mtx.unlock();      // kilidi bırak
    }
}
void demo_mutex_basic() {
    std::cout << "=== 3. Temel Mutex Kullanımı ===\n";
    shared_counter = 0;
    std::thread t1(increment_with_mutex);
    std::thread t2(increment_with_mutex);
    t1.join(); t2.join();
    std::cout << "Beklenen değer: 20000\n";
    std::cout << "Mutex ile korunan değer: " << shared_counter << "\n\n";
}

// ============================================================
// 4. std::lock_guard ile RAII (kilidi otomatik yönetme)
// ============================================================
void increment_with_lock_guard() {
    for (int i = 0; i < 10000; ++i) {
        std::lock_guard<std::mutex> lock(mtx); // kilidi alır, scope bittiğinde otomatik acar
        ++shared_counter;
    }
}
void demo_lock_guard() {
    std::cout << "=== 4. std::lock_guard ile RAII ===\n";
    shared_counter = 0;
    std::thread t1(increment_with_lock_guard);
    std::thread t2(increment_with_lock_guard);
    t1.join(); t2.join();
    std::cout << "Beklenen değer: 20000\n";
    std::cout << "Lock guard ile korunan değer: " << shared_counter << "\n\n";
}

// ============================================================
// 5. std::unique_lock (daha esnek kilitleme)
// ============================================================
std::mutex mtx2;
int shared_counter2 = 0;
void increment_with_unique_lock() {
    for (int i = 0; i < 10000; ++i) {
        std::unique_lock<std::mutex> lock(mtx2); // lock_guard'ın esnek versiyonu
        ++shared_counter2;
        // kilidi burada manuel açabiliriz: lock.unlock();
    }
}
void demo_unique_lock() {
    std::cout << "=== 5. std::unique_lock Kullanımı ===\n";
    shared_counter2 = 0;
    std::thread t1(increment_with_unique_lock);
    std::thread t2(increment_with_unique_lock);
    t1.join(); t2.join();
    std::cout << "Beklenen değer: 20000\n";
    std::cout << "Unique lock ile korunan değer: " << shared_counter2 << "\n\n";
}

// ============================================================
// 6. Condition variable ile thread senkronizasyonu
// ============================================================
std::mutex mtx_cv;
std::condition_variable cv;
bool data_ready = false;
int shared_data = 0;
void producer() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // üretim gecikmesi
    {
        std::lock_guard<std::mutex> lock(mtx_cv);
        shared_data = 42;
        data_ready = true;
    }
    cv.notify_one(); // bekleyen bir consumer'ı uyandır
}
void consumer() {
    std::unique_lock<std::mutex> lock(mtx_cv);
    cv.wait(lock, []{ return data_ready; }); // data_ready true olana kadar bekle
    std::cout << "Consumer: Alınan veri = " << shared_data << std::endl;
}
void demo_condition_variable() {
    std::cout << "=== 6. Condition Variable ile Thread Senkronizasyonu ===\n";
    data_ready = false;
    shared_data = 0;
    std::thread t1(producer);
    std::thread t2(consumer);
    t1.join(); t2.join();
    std::cout << "Consumer veri başarıyla aldı.\n\n";
}

// ============================================================
// 7. Birden çok thread’i yönetip sonuçları toplama
// ============================================================
void worker(int id, int& result, std::mutex& mtx) {
    int local_sum = 0;
    for (int i = 0; i < 1000; ++i) local_sum += i;
    {
        std::lock_guard<std::mutex> lock(mtx);
        result += local_sum;
    }
    std::cout << "Worker " << id << " finished, partial sum = " << local_sum << std::endl;
}
void demo_multiple_threads() {
    std::cout << "=== 7. Birden Çok Thread ile Toplama ===\n";
    int total = 0;
    std::mutex total_mtx;
    std::vector<std::thread> threads;
    for (int i = 0; i < 4; ++i)
        threads.emplace_back(worker, i, std::ref(total), std::ref(total_mtx));
    for (auto& t : threads) t.join();
    std::cout << "Toplam sonuç: " << total << std::endl;
    std::cout << "(Beklenen: 4 * (0+1+…+999) = 4 * 499500 = 1998000)\n\n";
}

// ============================================================
// 8. std::scoped_lock (C++17) - kilit kilitlenme önleyici
// ============================================================
std::mutex mtx_a;
std::mutex mtx_b;
int shared_resource = 0;

void worker_scoped(int id, bool /* lock_a_first */) {
    // The boolean parameter is kept for symmetry with the earlier example;
    // scoped_lock locks both mutexes in a dead‑lock‑avoiding way regardless of order.
    for (int i = 0; i < 5000; ++i) {
        std::scoped_lock lock(mtx_a, mtx_b); // locks both mutexes atomically
        ++shared_resource;
    }
    std::cout << "Worker " << id << " finished with scoped_lock.\n";
}

void demo_scoped_lock() {
    std::cout << "=== 8. std::scoped_lock (C++17) - Öldürülen kilitlenme önleyici ===\n";
    shared_resource = 0;
    std::thread t1(worker_scoped, 1, true);  // order does not matter for scoped_lock
    std::thread t2(worker_scoped, 2, false);
    t1.join();
    t2.join();
    std::cout << "Final shared_resource: " << shared_resource << " (expected 10000)\n\n";
}

// ============================================================
// main: tüm demo’ları sırayla çalıştır
// ============================================================
int main() {
    std::cout << "C++ Multithreading Temeller Thread Temelleri - Adım Adım Açıklama\n";
    std::cout << "====================================================\n\n";

    demo_create_and_run_thread();
    demo_race_condition();
    demo_mutex_basic();
    demo_lock_guard();
    demo_unique_lock();
    demo_condition_variable();
    demo_multiple_threads();
    demo_scoped_lock();

    std::cout << "Tüm demo'lar tamamlandı.\n";
    return 0;
}