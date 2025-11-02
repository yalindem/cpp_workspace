#include <iostream>
#include <vector>
#include <algorithm>
#include <list>
#include <deque>
#include <stack>
#include <queue>
#include <limits>
#include <memory>
#include <thread>
#include <future>
#include <condition_variable>
#include <chrono>
#include <random>
#include <optional>
#include <cstring>

namespace ClassProblems
{
    namespace DiamondProblem
    {
        class Base {
        public:
            void hello() { std::cout << "Hello from Base\n"; }
        };

        class A : virtual public Base { };

        class B : virtual public Base { };

        class C : public A, public B { };


        void run()
        {
            C obj;
            obj.hello();
        }
    }

    namespace Virtual
    {
        class Base
        {
            public: 
                virtual void print()
                {
                    std::cout << "Hi from Base\n";
                }

                /*
                    You need a virtual destructor when you use polymorphism, i.e., when you're deleting a derived class object through a base class pointer.
                    If you don’t make the base class destructor virtual:
                        The derived class destructor won't be called.
                        This causes memory/resource leaks.
                        The destruction will be incomplete and unsafe.
                */
                virtual ~Base()
                {
                    std::cout << "Base destructor\n";
                }

        };

        class Child : public Base{
            public:
                // override keyword is not "have to" is "nice to have"
                void print() override
                {
                    std::cout << "Hi from child class\n";
                } 
                
                ~Child()
                {
                    std::cout << "Child destructor\n";
                }
        };

        class OtherChild: public Base
        {
            public:
                void print() override
                {
                    std::cout << "Hi from other child class\n";
                }
                ~OtherChild()
                {
                    std::cout << "OtherChild destructor\n";
                }
        };


        void run()
        {
            Base* base = new Child();
            base->print();
            base = new OtherChild();
            base->print();
            delete base;
        }
    }

}

namespace STL
{
    /*
    Container Comparison:

    vector
        - Structure: Dynamic array
        - When to use: Random access, frequent access by index, appending elements at end
        - Pros: Fast random access (O(1)), cache friendly, dynamic resizing
        - Cons: Insert/delete at middle/front is slow (O(n))

    list
        - Structure: Doubly linked list
        - When to use: Frequent insert/delete anywhere, no random access needed
        - Pros: Fast insert/delete at any position (O(1)), stable iterators
        - Cons: No random access, more memory overhead, cache unfriendly

    deque
        - Structure: Double-ended queue (dynamic array blocks)
        - When to use: Fast insert/delete at front and back, random access needed
        - Pros: Fast push/pop at front/back, random access (O(1))
        - Cons: Slightly more memory overhead than vector

    stack
        - Structure: LIFO adapter (usually based on vector or deque)
        - When to use: When Last In First Out behavior needed
        - Pros: Simple API, fast push/pop at top
        - Cons: No iteration, limited access

    queue
        - Structure: FIFO adapter (usually based on deque)
        - When to use: When First In First Out behavior needed
        - Pros: Simple API, fast enqueue/dequeue
        - Cons: No iteration, limited access

    set
        - Structure: Balanced BST (usually Red-Black tree)
        - When to use: Store sorted unique elements, fast search
        - Pros: Automatic sorting, unique elements, logarithmic search/insert/delete
        - Cons: Slower than hash-based containers for lookup (O(log n))

    unordered_set
        - Structure: Hash table
        - When to use: Store unique elements, fast average lookup/insert/delete
        - Pros: Average O(1) operations, unordered
        - Cons: No ordering, worst case O(n), more memory usage

    Usage recommendations:

    - Use vector if you want fast random access and mostly add/remove at the end.
    - Use list if you need to insert/remove elements frequently in the middle or front and don't need random access.
    - Use deque if you want fast insert/remove at both ends and random access.
    - Use stack when you want a strict Last-In-First-Out structure (e.g., recursive calls, undo functionality).
    - Use queue when you want a strict First-In-First-Out structure (e.g., task scheduling, BFS).
    - Use set when you want sorted, unique elements and need ordered traversal.
    - Use unordered_set when you want unique elements with the fastest average lookup but don’t care about order.
    */

    namespace STL_vector
    {   

        /*
        dynamic array 
        contiguous block of memory
        stores its elements in a single continuous block of memory, The elements are placed side by side without gaps
        At the start: When you create an empty vector, usually no memory or a small capacity is allocated.

        Adding elements:
            Each push_back increases the size by 1.
            If size exceeds capacity, the vector allocates more memory.

        Capacity growth strategy:
            Usually, capacity doubles each time it needs to grow (e.g., 4 → 8 → 16 elements).
            This reduces the number of reallocations and improves performance.

        Reallocation process:
            Allocate a bigger block of memory.
            Copy existing elements to the new block.
            Release the old memory block.

        Usage:
            - random access O(1)
            - sona ekleme ve cikarma O(1)
            - searching element is fast O(1)
        */
        template <typename T>
        void print_vec(const std::vector<T> vec)
        {
            auto print = [](const T& n) { std::cout << n << ' '; };
            std::for_each(vec.begin(), vec.end(), print);
            std::cout << "\n";
        }

        void test_find()
        {
            std::vector<int> vec {1,2,3,4,5};
            auto it = std::find(vec.begin(), vec.end(), 3);
            if(it != vec.end())
            {
                std::cout << "found at index: " << std::distance(vec.begin(), it) << std::endl;
            }
            else{
                std::cout << "not found\n";
            }
        }

        void test_erase()
        {
            std::vector<int> vec{1,2,3,4,5,5,6,7};
            print_vec(vec);
            auto it = std::remove(vec.begin(), vec.end(), 5);
            vec.erase(it, vec.end());
            print_vec(vec);
        }

        int run()
        {
            //test_find();
            test_erase();
            return 0;
        }
    }

    namespace STL_list
    {
        /*
        Each element is stored independently in separate memory locations (nodes)
        Every node contains the element plus two pointers:

            One to the previous node
            One to the next node

        No contiguous memory — elements can be scattered anywhere in memory.
        No reallocation or copying happens when inserting or deleting elements.
        Insertion or deletion at any position is efficient (O(1)), once you have an iterator to that position.

        Dynamic allocation per element:
        Each insertion creates a new node (allocates memory dynamically).

        No capacity or size doubling:
        Unlike vector, it does not pre-allocate or grow capacity.

        Slower element access:
        Random access by index is not supported efficiently (O(n)), because you must traverse the list.

        Fast insertion and removal:
        Adding or removing elements at any position (beginning, middle, end) is fast (constant time).

        - Fast insertions/removals anywhere (beginning, middle, end).
        - No expensive reallocations or copying elements.
        - Good if you frequently insert/remove elements in the middle of the sequence.
        - searching elements is slow
        */
        
        template <typename T>
        void list_list(const std::list<T>& lst)
        {
            auto print = [](const T& i){std::cout << i << " "; };
            std::for_each(lst.begin(), lst.end(), print);
            std::cout << "\n";
        }

        void basics()
        {
            std::list<int> lst {1,2,3,4,5};
            list_list(lst);
            lst.push_back(1);
            list_list(lst);
            lst.push_front(1);
            list_list(lst);
            auto it = lst.begin();
            lst.insert(it,0);
            list_list(lst);
            lst.pop_back();
            list_list(lst);
            lst.pop_front();
            list_list(lst);
            lst.remove(1);
            list_list(lst);
            it = lst.begin();
            std::advance(it,2);
            lst.erase(it);
            list_list(lst);
        }

        void run()
        {  
            basics();
        }
    }

    namespace STL_deque
    {
        /*
        You want efficient push_front() and push_back() operations.
        Unlike std::vector, deque supports fast insertion/removal at the front without costly shifts.
        You want to access elements by index fast (operator[]) — unlike std::list, deque provides O(1) random access.
        Great if you want the flexibility of vector-like access plus efficient front insertion/removal.
        Insertions or deletions in the middle are slower (because elements may need to be shifted).
        If you do need frequent mid-container insertions, consider std::list.

        */
        template <typename T>
        void list_deque(const std::deque<T>& deq)
        {
            auto print = [](const T& i){std::cout << i << " ";};
            std::for_each(deq.begin(), deq.end(), print);
            std::cout << "\n";
        }

        void basics()
        {
            std::deque<int> deq {1,2,3,4};
            deq.push_back(1);
            deq.push_front(2);
            std::cout << deq.back() << "\n";
            std::cout << deq.front() << "\n";
            auto it = deq.begin();
            deq.insert(it, 4);
            deq.pop_back();
            deq.pop_front();
            deq.erase(it);   
            list_deque(deq); 
        }

        void run()
        {
            basics();
        }
    }

    namespace STL_stack
    {
        /*
        std::stack is a container adapter in C++ that provides a LIFO (Last In, First Out) data structure.
        Think of it as a stack of plates: you put a new plate on top, and you also take the top plate off first.
        It only allows access to the top element; you cannot access elements in the middle or bottom directly.
        */

        void basics()
        {
            std::stack<int> stack;

            stack.push(1);
            stack.push(2);
            stack.push(3);

            std::cout << stack.top() << std::endl;

            if (stack.empty() == false)
                std::cout << "stack is not empty\n";

            while (!stack.empty()) {
                std::cout << stack.top(); 
                stack.pop();
            }

        }

        void run()
        {
            basics();
        }
    }

    namespace STL_queue
    {
        /*
        A queue is a linear data structure that follows the FIFO principle — First In, First Out.
        The first element added to the queue will be the first one to be removed.
        Think of it like a line at a checkout counter: the first person in line is served first.
        */

        void basics()
        {
            std::queue<int> q;
            q.push(10);
            q.push(20);
            std::cout << "Front element: " << q.front() << std::endl;
            q.pop();
            std::cout << "Front element after pop: " << q.front() << std::endl;
        }

        void run()
        {
            basics();
        }
    }

}

namespace STL_algorithms
{
    template <typename T> 
    void print_vec(const std::vector<T>& vec)
    {
        auto print = [](const T& i){std::cout << i << " ";};
        std::for_each(vec.begin(), vec.end(), print);
        std::cout << "\n";
    }

    namespace Sort{
        
        void sort()
        {
            std::vector<int> vec = {5, 3, 8, 1};
            std::sort(vec.begin(), vec.end());
            print_vec(vec);

        }
        
        void run()
        {
            sort();
        }

    }

    namespace BinarySearch
    {
        void binary_search()
        {
            std::vector<int> vec = {5, 3, 8, 1};
            std::sort(vec.begin(), vec.end());
            bool found = std::binary_search(vec.begin(), vec.end(), 3);
            std::cout << "found: " << found << "\n";
        }

        void run()
        {
            binary_search();
        }
    }

    namespace Bound
    {
        void lower_bound()
        {
            std::vector<int> vec = {1, 3, 12, 5};
            std::sort(vec.begin(), vec.end());
            auto it = std::lower_bound(vec.begin(), vec.end(), 4);
            std::cout << "index: " << std::distance(vec.begin(), it) << std::endl;
            std::cout << "value: " << *std::lower_bound(vec.begin(), vec.end(), 4) << "\n";

        }

        void upper_bound()
        {
            std::vector<int> vec = {1, 3, 12, 5};
            std::sort(vec.begin(), vec.end());
            auto it = std::upper_bound(vec.begin(), vec.end(), 4);
            std::cout << "index: " << std::distance(vec.begin(), it) << std::endl;
            std::cout << "value: " << *std::upper_bound(vec.begin(), vec.end(), 4) << "\n";
        }

        void run()
        {
            lower_bound();
            upper_bound();
        }
    }

    namespace Count
    {
        void count()
        {            
            std::vector<int> vec = {1, 3, 12, 5, 5};
            int c = std::count(vec.begin(), vec.end(), 5);
            std::cout << "c: " << c << "\n";
        }

        void count_if()
        {
            std::vector<int> vec = {1, 3, 12, 5, 5};
            int evens = std::count_if(vec.begin(), vec.end(), [](int x){ return x % 2 == 0; });
            std::cout << "even number count: " << evens << std::endl;
        }

        void equal()
        {
            std::vector<int> a = {1, 2, 3};
            std::vector<int> b = {1, 2, 3};
            bool same = std::equal(a.begin(), a.end(), b.begin());
            std::cout << "same: " << same << std::endl;
        }

        void run()
        {
            count();
            count_if();
            equal();
        }
    }

    namespace Transform
    {
        void transform()
        {
            std::vector<int> v = {1, 2, 3};
            std::vector<int> result(3);
            auto quad = [](int x){return x * x;};
            std::transform(v.begin(), v.end(), result.begin(), quad);
            print_vec(v);
            print_vec(result);
        }

        void run()
        {
            transform();
        }
    }

}

namespace Algorithms
{
    namespace DFS
    {
        struct TreeNode {
            int val;
            TreeNode* left;
            TreeNode* right;
            TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
        };

        int getMaxDepth(TreeNode* root)
        {
            if(root == nullptr)
                return 0;
            int left_depth = getMaxDepth(root->left);
            int right_depth = getMaxDepth(root->right);
            return 1 + std::max(left_depth, right_depth);
        }

        struct Node
        {
            int val;
            std::vector<Node*> children;
            Node(int x) : val(x){}
        };

        int maxDepth(Node* root)
        {
            if (root == nullptr) return 0;
            int depth = 0;
            for (Node* child : root->children) {
                depth = std::max(depth, maxDepth(child));
            }

            return depth + 1;
        }

        void run()
        {
            TreeNode* trroot = new TreeNode(3);
            trroot->left = new TreeNode(9);
            trroot->right = new TreeNode(20);
            trroot->right->left = new TreeNode(15);
            trroot->right->right = new TreeNode(7);
            int depth = getMaxDepth(trroot);

            std::cout << "depth: " << depth << "\n";

            Node* root = new Node(1);
            Node* node3 = new Node(3);
            Node* node2 = new Node(2);
            Node* node4 = new Node(4);
            Node* node5 = new Node(5);
            Node* node6 = new Node(6);
            Node* node7 = new Node(10);

            root->children = {node3, node2, node4};
            node3->children = {node5, node6};
            node4->children = {node7};

            std::cout << "Max depth: " << maxDepth(root) << std::endl; 

        }
    }

    namespace BFS
    {
        /*
        Start from the source node.
        Use a queue to keep track of nodes to visit next.
        Mark the source as visited and enqueue it.
        While the queue is not empty:
            Dequeue a node u.
            Process u (e.g., print or store it).
            Enqueue all unvisited neighbors of u and mark them visited.
        */

        void bfs(int start, const std::vector<std::vector<int>>& graph) {
            int n = graph.size();
            std::vector<bool> visited(n, false);
            std::queue<int> q;

            visited[start] = true;
            q.push(start);

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                std::cout << u << " ";

                // Explore neighbors
                for (int v : graph[u]) {
                    if (!visited[v]) {
                        visited[v] = true;
                        q.push(v);
                    }
                }
            }
        }

        void run()
        {
            int n = 6;
            std::vector<std::vector<int> > graph(n);

            graph[0] = {1, 2};
            graph[1] = {0, 3, 4};
            graph[2] = {0, 4};
            graph[3] = {1, 5};
            graph[4] = {1, 2};
            graph[5] = {3};

            bfs(0, graph);

        }
    }

    namespace Dijkstra
    {
        /*
        It finds the shortest path from a single source node to all other nodes in a weighted graph.
        Works only with non-negative edge weights.
        Uses a greedy approach to pick the closest unvisited node each time.
        
        How it works:
        Initialize distances to all nodes as infinity, except the source node which is 0.
        Use a priority queue (min-heap) to always pick the next closest node.
        Update the distances to neighbors if a shorter path is found through the current node.
        Repeat until all nodes are processed.
        */

        typedef std::pair<int, int> Edge;
        const int INF = std::numeric_limits<int>::max();

        void dijkstra(int start, const std::vector<std::vector<Edge>>& graph, std::vector<int>& distances) 
        {
            int n = graph.size();
            distances.assign(n, INF);
            distances[start] = 0;

            std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
            pq.push({0, start});

            while (!pq.empty()) {
                int dist = pq.top().first;
                int node = pq.top().second;
                pq.pop();

                // Daha önce daha kısa bir yol bulunduysa atla
                if (dist > distances[node])
                    continue;

                for (const Edge& edge : graph[node]) {
                    int neighbor = edge.first;
                    int weight = edge.second;
                    int newDist = distances[node] + weight;

                    if (newDist < distances[neighbor]) {
                        distances[neighbor] = newDist;
                        pq.push({newDist, neighbor});
                    }
                }
            }
        }

        void run()
        {
            int n = 5;
            std::vector<std::vector<Edge>> graph(n);

            graph[0].push_back({1, 2});
            graph[1].push_back({0, 2}); 

            graph[0].push_back({3, 6});
            graph[3].push_back({0, 6});

            graph[1].push_back({2, 3});
            graph[2].push_back({1, 3});

            graph[1].push_back({3, 8});
            graph[3].push_back({1, 8});

            graph[1].push_back({4, 5});
            graph[4].push_back({1, 5});

            graph[2].push_back({4, 7});
            graph[4].push_back({2, 7});

            std::vector<int> distances;
            dijkstra(0, graph, distances);

            for (int i = 0; i < n; ++i) {
                std::cout << "from 0 to" << i << "shortest path: ";
                if (distances[i] == INF)
                    std::cout << "cannot be reached" << std::endl;
                else
                    std::cout << distances[i] << std::endl;
            }

        }
    }

    namespace BellmanFord
    {

    }

    namespace ASearch
    {

    }

    namespace TopologicalSort
    {

    }

    namespace Kruskal
    {

    }

    namespace Prims 
    {

    }

    namespace Backtracking
    {

    }

    namespace DynamicProgramming
    {
        /*
        DP is used to solve problems by breaking them down into simpler overlapping subproblems.
        It stores the results of these subproblems to avoid redundant calculations (called memoization).
        It’s especially useful for optimization problems and counting problems.
        */
    }

    namespace Greedy
    {

    }

}

namespace Polymorphism
{
    class Shape {
        public:
            virtual void draw() const = 0; // Saf sanal fonksiyon - abstract class
            virtual double area() const = 0;
            virtual ~Shape() = default; // Base sınıflarda virtual destructor ŞART!
        };

        // Türetilmiş sınıflar
        class Circle : public Shape {
        private:
            double radius_;
        public:
            Circle(double r) : radius_(r) {}
            void draw() const override {
                std::cout << "Drawing Circle with radius " << radius_ << std::endl;
            }
            double area() const override {
                return 3.14159 * radius_ * radius_;
            }
        };

        class Rectangle : public Shape {
        private:
            double width_, height_;
        public:
            Rectangle(double w, double h) : width_(w), height_(h) {}
            void draw() const override {
                std::cout << "Drawing Rectangle " << width_ << "x" << height_ << std::endl;
            }
            double area() const override {
                return width_ * height_;
            }
        };
    

    //Factory Pattern
    std::unique_ptr<Shape> createShape(const std::string& type, double param1, double param2 = 0)
    {
        if (type == "circle")
        {
            return std::make_unique<Circle>(param1);
        }
        else if(type == "rectangle")
        {
            return std::make_unique<Rectangle>(param1, param2);
        }
        throw std::invalid_argument("Unknown shape type");
    }

    // Strategy Pattern
    class DrawingTool {
        private:
            std::unique_ptr<Shape> shape_; // Hangi şekli çizeceğimizi bilmiyoruz, sadece Shape arayüzünü biliyoruz
        public:
            DrawingTool(std::unique_ptr<Shape> shape) : shape_(std::move(shape)) {}
            
            void useTool() {
                std::cout << "Using drawing tool: ";
                shape_->draw();
            }
    };
    

    void strategyExample() {
        // Aynı tool'u farklı şekillerle kullanabiliriz
        DrawingTool circleTool(std::make_unique<Circle>(7.0));
        DrawingTool rectTool(std::make_unique<Rectangle>(3.0, 9.0));
        
        circleTool.useTool(); // Drawing Circle with radius 7
        rectTool.useTool();   // Drawing Rectangle 3x9
    }
    
    void run()
    {
        std::vector<std::unique_ptr<Shape>> shapes;
        //shapes.reserve(0);

        auto shape1 = createShape("circle", 10.0);
        auto shape2 = createShape("circle", 15.0);
        auto shape3 = createShape("rectangle", 5.0, 5.0);
        auto shape4 = createShape("rectangle", 5.0, 10.0);

        shapes.push_back(std::move(shape1));
        shapes.push_back(std::move(shape2));
        shapes.push_back(std::move(shape3));
        shapes.push_back(std::move(shape4));

        strategyExample();
        
    }
}

namespace CopyandMoveConstructor
{   
    //
    /*
        Eğer hiçbirini yazmazsan:
        Derleyici otomatik olarak copy constructor ve copy assignment oluşturur.

        Ama move constructor’ı sadece copy constructor yoksa otomatik üretir.

        Birini manuel yazarsan, diğerleri bazen otomatik oluşturulmaz.
        Bu yüzden Rule of Five der:

        “Eğer birini yazıyorsan, muhtemelen hepsini yazmalısın.”
    
    */

    class String{
        private:
            char* data;

        public:
            String(const char* s = "")
            {
                std::cout << "Constructor\n";
                data = new char[strlen(s) + 1];
                strcpy(data, s);
            }

            //Copy Constructor
            String(const String& other)
            {
                std::cout << "Copy Constructor\n";
                // data = other.data; // Shallow copy, which copies just the memory adress of other.data to data
                                      // if we call delete[] data, that would cause some issues because then 
                                      // the memory adress will be released and data or other.data couldnt 
                                      // be reached anymore

                data = new char[strlen(other.data) + 1];
                strcpy(data, other.data);
            }

            //Move Constructor
            String(String&& other) noexcept
            {
                std::cout << "Move Constructor\n";
                // like a shallow copy but not dangerous since we take the ownership doing other.data = nullptr;
                data = other.data;
                other.data = nullptr;
            }

            // Copy assignment operator
            String& operator=(const String& other)
            {
                std::cout << "Copy assignment\n";
                if(this != &other)
                {
                    delete[] data;
                    data = new char[strlen(other.data) + 1];
                    strcpy(data, other.data);
                }
                return *this;
            }

            // Move assignment operator
            String& operator=(String&& other) noexcept
            {
                std::cout << "Move assignment\n";
                if(this != &other)
                {
                    delete[] data;
                    data = other.data;
                    other.data = nullptr;
                }
                return *this;
            }
    };

    void run()
    {
        String a("Yalin");      // Constructor
        String b = a;           // Copy constructor
        String c = std::move(a); // Move constructor
        b = c;                  // Copy assignment
        c = String("GPT");      // Move assignment
    }
 
}

namespace CopyElision
{   
    /*
        eskiden:
        --------
        String makeString() {
         String s("hello");
         return s;
        }

        Derleyici şunu yapardı:
        1- s nesnesini oluşturur.
        2- return s; derken bir copy constructor çağırır (geri dönen nesneye).
        3- Çağıran fonksiyonda da bir kopya daha oluşabilir.

        Yani 2–3 kez aynı veriyi kopyalardı. Bu da büyük objelerde ciddi performans kaybıydı.
    
    */
    
    struct A {
        A() { std::cout << "Default ctor\n"; }
        A(const A&) { std::cout << "Copy ctor\n"; }
        A(A&&) noexcept { std::cout << "Move ctor\n"; }
        ~A() { std::cout << "Dtor\n"; }
    };

    // 1️⃣ RVO (Return Value Optimization) / unnamed temporary
    A make1() {
        return A(); // unnamed temporary -> RVO
    }

    // 2️⃣ NRVO (Named Return Value Optimization) / named variable
    A make2() {
        A a;
        return a;   // named variable -> NRVO
    }

    // 3️⃣ std::move
    A make3() {
        A a;
        return std::move(a); // RVO devre dışı, move ctor çağrılır
    }

    void run()
    {
        std::cout << "--- make1 ---\n";
        A x1 = make1();

        std::cout << "--- make2 ---\n";
        A x2 = make2();

        std::cout << "--- make3 ---\n";
        A x3 = make3();
    }
}

namespace Threads
{
    namespace deneme_bir
    {
        void hello()
        {
            std::cout << "hello from thread\n";
        }

        void func1()
        {
            std::thread t(hello);
            t.join(); // Thread'in bitmesini bekler
        }
    }

    namespace deneme_iki
    {
        int compute()
        {
            return 42;
        }
        void func2()
        {
            std::future<int> result = std::async(std::launch::async, compute);
            std::cout << "Result: " << result.get() << std::endl; // get() sonucu alır
        }
    }

    namespace deneme_uc
    {
        std::mutex mtx;
        int shared_data = 0;

        void increment() {
            mtx.lock();
            ++shared_data;
            mtx.unlock();
        }

        void safe_increment() {
            std::lock_guard<std::mutex> lock(mtx); // RAII: unlock otomatik
            ++shared_data;
        }

        void func3()
        {
            std::thread t1(safe_increment);
            std::thread t2(safe_increment);
            t1.join();
            t2.join();
            std::cout << shared_data << std::endl; // 2
        }

    }

    namespace deneme_dort
    {
        void backgroundTask() {
            std::cout << "Background task başlıyor...\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "Background task bitti!\n";
        }

        void func4() {
            std::thread t(backgroundTask);
            t.detach();  // ✅ Artık t bağımsız çalışacak
            std::cout << "Main thread bitti ama background hala çalışıyor olabilir!\n";

            //detach() edilen thread bağımsız hale gelir → kontrol edemezsin, bekleyemezsin, çıktısını bilemezsin.
            //Bu yüzden sadece arka plan loglama veya network dinleme gibi işler için uygundur.
        }
    }

    namespace deneme_bes
    {   
        void increment(int &x) {
            ++x;
        }

        void printNumber(int n) {
            std::cout << "Sayı: " << n << "\n";
        }

        int counter = 0;
        std::mutex mtx;

        void increment2() 
        {
            for (int i = 0; i < 100000; ++i)
            {
                std::lock_guard<std::mutex> lock(mtx); 
                ++counter;
            }
        }
        

        std::condition_variable cv;
        std::queue<int> dataQueue;
        bool done = false;

        void producer()
        {
            for (int i = 1; i<=5; ++i)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    dataQueue.push(i);
                    std::cout << "[Producer] Üretilen veri: " << i << "\n";
                }
                cv.notify_one();
            }

            {
                std::lock_guard<std::mutex> lock(mtx);
                done = true;
            }

            cv.notify_all();
        }

        void consumer(int id)
        {
            while(true)
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [](){return !dataQueue.empty() || done;});
                if (!dataQueue.empty()) {
                    int value = dataQueue.front();
                    dataQueue.pop();
                    lock.unlock(); // uzun işlemden önce kilidi açıyoruz
                    std::cout << "--- [Consumer " << id << "] Veriyi işliyor: " << value << "\n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
                }
                else if (done) {
                    break; // üretici bitti, kuyruk boş → çık
                }
            }
            std::cout << "  [Consumer " << id << "] İş bitti.\n";
        }



        void func5()
        {
            /*
            std::thread t([](){
                std::cout << "Lambda thread çalışıyor!\n";
            });
            t.join();

            std::thread t2(printNumber, 42); 
            if (t2.joinable())
            {
                t2.join();
            }

            int value = 10;
            std::thread t3(increment, std::ref(value));
            if (t3.joinable())
            {
                t3.join();
            }
            std::cout << "Sonuç: " << value << "\n";
            */

            /*
            std::thread t4(increment2);
            std::thread t5(increment2);
            t4.join();
            t5.join();
            std::cout << "Counter: " << counter << "\n";
            */

            std::thread prod(producer);
            std::thread cons1(consumer, 1);
            std::thread cons2(consumer, 2);
            prod.join();
            cons1.join();
            cons2.join();
            std::cout << "Tüm işler tamamlandı.\n";
        }
    }

    namespace deneme_sensors
    {
        std::mutex mtx;
        std::condition_variable cv;
        std::queue<float> sensorDataQueue;
        bool done = false;

        float readSensor()
        {
            static std::default_random_engine generator(std::random_device{}());
            static std::uniform_real_distribution<float> distribution(20.0, 30.0);
            return distribution(generator);
        }

        void sensorThread()
        {
            for(int i = 0; i < 10; ++i)
            {
                float value = readSensor();
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    sensorDataQueue.push(value);
                    std::cout << "[Sensor] Veri okundu: " << value << " °C\n";
                }
                cv.notify_one();
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
            }
        
            {
                std::lock_guard<std::mutex> lock(mtx);
                done = true;
            }
        }

        void processorThread() {
            std::vector<float> window;

            const size_t windowSize = 3;
        
            while(true)
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [](){return !sensorDataQueue.empty() || done;});

                if(!sensorDataQueue.empty())
                {
                    float data = sensorDataQueue.front();
                    sensorDataQueue.pop();
                    lock.unlock();

                    window.push_back(data);
                    if(window.size() > windowSize)
                    {
                        window.erase(window.begin());
                    }

                    
                    float sum = 0.0f;
                    for (float v : window) sum += v;
                    float avg = sum / window.size(); 

                    std::cout << "  [Processor] Ortalama (" << window.size() << " örnek): " << avg << " °C\n";

                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // işleme süresi
                }
                else if (done) {
                    break;
                }
            }
            std::cout << "  [Processor] İşleme tamamlandı.\n";
        
        }
        
        void run()
        {
            std::thread sensor(sensorThread);
            std::thread processor(processorThread);
            sensor.join();
            processor.join();

            std::cout << "Tüm sensör işlemleri tamamlandı.\n";
        }

    }

    namespace lockguard_vs_uniquelock
    {
        std::mutex mtx;
        void lockguard(int &x) {
            std::lock_guard<std::mutex> lock(mtx);  // otomatik lock
            ++x;
            // scope bitince otomatik unlock
            /*
            🔹 Kullanımı çok kolaydır.
            🔹 Hızlıdır.
            🔹 Ama kilidi manuel açmak veya condition_variable ile beklemek mümkün değildir.
            */
        }

        void uniquelock() {
            std::unique_lock<std::mutex> lock(mtx);  // lock()
            std::cout << "Kritik bölge\n";
            lock.unlock();  // manuel unlock
            std::cout << "Artık kilit serbest\n";
        }

        // Example2

        std::condition_variable cv;
        bool ready;

        void worker()
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [] { return ready; });  // 🔹 kilidi geçici bırakır, sonra tekrar alır
            std::cout << "Çalışmaya başladım!\n";
        }

        void test()
        {
            std::thread t(worker);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            {
                std::lock_guard<std::mutex> lock(mtx);
                ready = true;
            }
            cv.notify_one();
            t.join();
        }

        // Locking strategies
        // 1- defer_lock
        // 2- try_to_lock
        // 3- adopt_lock

        // EXAMPLE 1
        int buffer = 0;
        void task(const char* threadNumber, int loopFor)
        {   
            // part1
            /*
            std::unique_lock<std::mutex> lock(mtx); //automatically calls lock on mutex mtx
            for(int i = 0; i < loopFor; ++i)
            {
                buffer++;
                std::cout << threadNumber <<  " " << buffer << std::endl;
            }
            */

            //part2
            // does the same thing, so std::unique_lock<std::mutex> lock(mtx) does the same thing, it locks and unlocks
            /*
            mtx.lock();
            for(int i = 0; i < loopFor; ++i)
            {
                buffer++;
                std::cout << threadNumber <<  " " << buffer << std::endl;
            }
            mtx.unlock();
            */
            
            //part3
            std::unique_lock<std::mutex> lock(mtx, std::defer_lock); // does not call lock on mutex mtx
            lock.lock();
            for(int i = 0; i < loopFor; ++i)
            {
                buffer++;
                std::cout << threadNumber <<  " " << buffer << std::endl;
            }

        }

        void run()
        {
            std::thread t1(task, "T1", 10);
            std::thread t2(task, "T2", 10);
            t1.join();
            t2.join();
        }

    }

    namespace conditional_variables
    {
        std::condition_variable cv;
        std::mutex m;
        long balance = 0;

        void addMoney(int money)
        {
            std::lock_guard<std::mutex> lg(m); // 3- acquire the mutex m;
            balance += money;
            std::cout << "Amount added current balance: " << balance << "\n";
            cv.notify_one(); // bu satiri sil ve neler oldugunu kontrol et // 4- notify whoever waiting this thread
        }

        void withdrowMoney(int money)
        {   
            // we will wait here if the m is locked by another function
            std::unique_lock<std::mutex> lu(m); // 1- locked the mutex m

            //cv.wait is like an while loop
            cv.wait(lu, []{return balance!=0 ? true:false;}); // 2- releasing the mutex m // 5- now the condition is true

            // we could also use wait_for or wait_until instead wait
            //cv.wait_until(lu, std::chrono::system_clock::now() + std::chrono::seconds(2), []{return balance != 0 ? true:false;});
            //cv.wait_for(lu, std::chrono::milliseconds(0), []{return balance != 0 ? true:false;});
            //6- now condition is true
            if(balance >= money)
            {
                balance -= money;
                std::cout << "Amount deducted: " << money << "\n";
            }
            else
            {
                std::cout << "Amount cant be deducted, current balance less than" << money << "\n";
            }
            std::cout << "current balance is: " << balance << "\n";
        }

        void run()
        {
            std::thread t1(withdrowMoney, 500);
            //std::this_thread::sleep_for(std::chrono::seconds(1));
            std::thread t2(addMoney, 500);

            t1.join();
            t2.join();
        }
    }

    void run()
    {
        //deneme_bir::func1();
        //deneme_iki::func2();
        //deneme_uc::func3();
        //deneme_dort::func4();
        //deneme_bes::func5();
        //deneme_sensors::run();
        //lockguard_vs_uniquelock::run();
        conditional_variables::run();
    }

}

namespace Sensors
{

    struct SensorData
    {
        double value;
        std::chrono::system_clock::time_point timestamp;
    };

    SensorData sensorA, sensorB, sensorC;
    std::mutex mtxA, mtxB, mtxC;


    void sensorTask(const std::string& name, SensorData& data, std::mutex& mtx, int period_ms, double step)
    {
        double value {0.0};
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            value += step;

            {
                std::lock_guard<std::mutex> lock(mtx);
                data.value = value;
                data.timestamp = std::chrono::system_clock::now();
            }

            std::cout << "[" << name << "] Yeni veri: " << value << " (" << 1000.0 / period_ms << " Hz)\n";
        }
    }

    void readerTask(int period_ms)
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            SensorData a, b, c;
            {
                std::lock_guard<std::mutex> lockA(mtxA);
                a = sensorA;
            }

            {
                std::lock_guard<std::mutex> lockB(mtxB);
                b = sensorB;
            }
            {
                std::lock_guard<std::mutex> lockC(mtxC);
                c = sensorC; 
            }
            auto now = std::chrono::system_clock::now();
            auto ageA = std::chrono::duration_cast<std::chrono::milliseconds>(now - a.timestamp).count();
            auto ageB = std::chrono::duration_cast<std::chrono::milliseconds>(now - b.timestamp).count();
            auto ageC = std::chrono::duration_cast<std::chrono::milliseconds>(now - c.timestamp).count();


            std::cout << "\n=== [Reader] Sensör verileri (" << 1000.0 / period_ms << " Hz) ===\n";
            std::cout << "A: " << a.value << " (age: " << ageA << " ms)\n";
            std::cout << "B: " << b.value << " (age: " << ageB << " ms)\n";
            std::cout << "C: " << c.value << " (age: " << ageC << " ms)\n";
            std::cout << "==========================================\n\n";
        }
    }

    void run()
    {
        std::thread tA(sensorTask, "SensorA", std::ref(sensorA), std::ref(mtxA), 100, 0.5);   // 10 Hz
        std::thread tB(sensorTask, "SensorB", std::ref(sensorB), std::ref(mtxB), 200, 1.0);   // 5 Hz
        std::thread tC(sensorTask, "SensorC", std::ref(sensorC), std::ref(mtxC), 500, 2.0);   // 2 Hz
        std::thread tReader(readerTask, 500);                                                 // 2 Hz

        tA.join();
        tB.join();
        tC.join();
        tReader.join();
    }
}

namespace Fusion
{

    struct TempData {
        float temperature;
        std::string unit;
    };

    struct ImuData {
        float accelX, accelY, accelZ;
    };

    struct PressureData {
        int pressure;
    };

    template<typename T>
    class ThreadSafeQueue {
    private:
        std::queue<T> q;
        mutable std::mutex mtx;
    public:
        void push(const T& value) {
            std::lock_guard<std::mutex> lock(mtx);
            q.push(value);
        }

        std::optional<T> pop() {
            std::lock_guard<std::mutex> lock(mtx);
            if (q.empty())
                return std::nullopt;
            T value = q.front();
            q.pop();
            return value;
        }

        bool empty() const {
            std::lock_guard<std::mutex> lock(mtx);
            return q.empty();
        }

        size_t size() const {
            std::lock_guard<std::mutex> lock(mtx);
            return q.size();
        }
    };

    
    void sensorA(ThreadSafeQueue<TempData>& q) {
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<float> dist(20.0, 25.0);

        while (true) {
            TempData data{dist(gen), "°C"};
            q.push(data);
            std::cout << "[SensorA] Temp: " << data.temperature << data.unit << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
        }
    }

    void sensorB(ThreadSafeQueue<ImuData>& q) {
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<float> dist(-1.0, 1.0);

        while (true) {
            ImuData data{dist(gen), dist(gen), dist(gen)};
            q.push(data);
            std::cout << "[SensorB] IMU: (" << data.accelX << ", "
                    << data.accelY << ", " << data.accelZ << ")\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5 Hz
        }
    }

    void sensorC(ThreadSafeQueue<PressureData>& q) {
        std::mt19937 gen(std::random_device{}());
        std::uniform_int_distribution<int> dist(1000, 1020);

        while (true) {
            PressureData data{dist(gen)};
            q.push(data);
            std::cout << "[SensorC] Pressure: " << data.pressure << " hPa\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 2 Hz
        }
    }



    void fusion(ThreadSafeQueue<TempData>& qA,
                ThreadSafeQueue<ImuData>& qB,
                ThreadSafeQueue<PressureData>& qC) {

        while (true) {
            auto temp = qA.pop();
            auto imu = qB.pop();
            auto pres = qC.pop();

            std::cout << "[Fusion] ";

            if (temp) std::cout << "T=" << temp->temperature << temp->unit << " ";
            else std::cout << "T=--- ";

            if (imu) std::cout << "IMU=(" << imu->accelX << "," << imu->accelY << "," << imu->accelZ << ") ";
            else std::cout << "IMU=(---) ";

            if (pres) std::cout << "P=" << pres->pressure << "hPa\n";
            else std::cout << "P=---\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    void run()
    {
        ThreadSafeQueue<TempData> qTemp;
        ThreadSafeQueue<ImuData> qImu;
        ThreadSafeQueue<PressureData> qPress;

        std::thread tA(sensorA, std::ref(qTemp));
        std::thread tB(sensorB, std::ref(qImu));
        std::thread tC(sensorC, std::ref(qPress));
        std::thread tFusion(fusion, std::ref(qTemp), std::ref(qImu), std::ref(qPress));

        tA.join();
        tB.join();
        tC.join();
        tFusion.join();
    }
}

int main()
{   

    //ClassProblems::DiamondProblem::run();
    //ClassProblems::Virtual::run();

    //STL::STL_vector::run();
    //STL::STL_list::run();
    //STL::STL_deque::run();
    //STL::STL_stack::run();
    //STL::STL_queue::run();
    
    //STL_algorithms::Sort::run();
    //STL_algorithms::BinarySearch::run();
    //STL_algorithms::Bound::run();
    //STL_algorithms::Count::run();
    //STL_algorithms::Transform::run();

    //Algorithms::DFS::run();
    //Algorithms::BFS::run();

    //Pattern::Command::run();

    //Polymorphism::run();
    //CopyandMoveConstructor::run();
    CopyElision::run();

    //Threads::run();
    //Sensors::run();
    //Fusion::run();




    //learn topics in future
        // type_traits
        // std::forward and forwarding references (https://www.youtube.com/watch?v=RW9KnqszYj4&t=87s)

    return 0;
}