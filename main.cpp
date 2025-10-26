#include <iostream>
#include <vector>
#include <algorithm>
#include <list>
#include <deque>
#include <stack>
#include <queue>
#include <limits>
#include <memory>

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

namespace Pattern
{
    namespace Command
    {
        class RobotArm
        {
            public:
                void grip() {
                    std::cout << "ARM: Gripping the object tightly." << std::endl;
                }

                void release() {
                    std::cout << "ARM: Releasing the object." << std::endl;
                }
        };

        class RobotWheels {
            public:
                void moveForward(int distance) {
                    std::cout << "WHEELS: Moving forward " << distance << " units." << std::endl;
                }

                void moveBackward(int distance) {
                    std::cout << "WHEELS: Moving backward " << distance << " units." << std::endl;
                }
        };

        class Command {
            public:
                virtual ~Command() = default;
                virtual void execute() = 0;
                virtual void undo() = 0;
        };  
        
        
        // Concrete Command 1: Move Forward
        class MoveForwardCommand : public Command {
            private:
                RobotWheels& wheels_;
                int distance_;

            public:
                MoveForwardCommand(RobotWheels& wheels, int distance)
                    : wheels_(wheels), distance_(distance) {}

                void execute() override {
                    wheels_.moveForward(distance_);
                }

                void undo() override {
                    // Undoing a move forward command means moving backward.
                    wheels_.moveBackward(distance_); 
                }
        };

        // Concrete Command 2: Grip an Object
        class GripCommand : public Command {
            private:
                RobotArm& arm_;

            public:
                GripCommand(RobotArm& arm) : arm_(arm) {}

                void execute() override {
                    arm_.grip();
                }

                void undo() override {
                    // Undoing a grip command means releasing.
                    arm_.release(); 
                }
        };

        class RobotTaskQueue {
            private:
                // Using std::unique_ptr ensures RAII is applied to Command objects.
                std::vector<std::unique_ptr<Command>> tasks_;
                std::vector<std::unique_ptr<Command>> completedTasks_; // For undo history

            public:
                void addTask(std::unique_ptr<Command> command) {
                    tasks_.push_back(std::move(command));
                }

                void executeQueue() {
                    std::cout << "\n--- Task Queue Started ---" << std::endl;
                    while (!tasks_.empty()) {
                        std::unique_ptr<Command> command = std::move(tasks_.front());
                        tasks_.erase(tasks_.begin());
                        
                        command->execute();
                        completedTasks_.push_back(std::move(command)); // Move to history
                    }
                    std::cout << "--- Task Queue Finished ---" << std::endl;
                }

                void undoLastCommand() {
                    if (completedTasks_.empty()) {
                        std::cout << "No commands to undo." << std::endl;
                        return;
                    }

                    std::unique_ptr<Command> lastCommand = std::move(completedTasks_.back());
                    completedTasks_.pop_back();

                    std::cout << "\n--- Undoing Last Command ---" << std::endl;
                    lastCommand->undo();
                    
                    // The undone command is discarded in this simple model.
                }
        };

        int run()
        {
            RobotWheels wheels;
            RobotArm arm;
            
            // 2. Create the Invoker (Task Queue)
            RobotTaskQueue taskQueue;

            // 3. The Client creates and configures Concrete Commands.
            
            // Task 1: Move 5 units forward
            taskQueue.addTask(std::make_unique<MoveForwardCommand>(wheels, 5)); 
            
            // Task 2: Grip the object
            taskQueue.addTask(std::make_unique<GripCommand>(arm));
            
            // Task 3: Move 2 units forward
            taskQueue.addTask(std::make_unique<MoveForwardCommand>(wheels, 2)); 
            
            // 4. The Invoker executes the sequence.
            taskQueue.executeQueue();
            
            // 5. Undo Functionality Test
            // Undoing the last command ("Move 2 units forward") will trigger moveBackward(2).
            taskQueue.undoLastCommand(); 

            /* Expected Output:

            --- Task Queue Started ---
            WHEELS: Moving forward 5 units.
            ARM: Gripping the object tightly.
            WHEELS: Moving forward 2 units.
            --- Task Queue Finished ---

            --- Undoing Last Command ---
            WHEELS: Moving backward 2 units.
            */
        }

    }

    /*
    namespace State
    {
        // Forward declaration
        class RobotContext; 

        // 1. State Interface
        class RobotState {
            public:
                virtual ~RobotState() = default;
                
                // Actions whose behavior changes based on the state
                virtual void handleMovement(RobotContext* context) = 0;
                virtual void handlePower(RobotContext* context) = 0;
                
                // For debugging/logging
                virtual std::string getStateName() const = 0;
        };
        
        // 2. Context
        class RobotContext {
            private:
                std::unique_ptr<RobotState> currentState_;

            public:
                RobotContext(std::unique_ptr<RobotState> initialState) : currentState_(std::move(initialState)) {
                    std::cout << "Robot initialized in state: " << currentState_->getStateName() << std::endl;
                }

                // Method to change the robot's state (transition)
                void changeState(std::unique_ptr<RobotState> newState) {
                    std::cout << "--- TRANSITION: " << currentState_->getStateName() << " -> " << newState->getStateName() << " ---" << std::endl;
                    currentState_ = std::move(newState);
                }
                
                // Delegate actions to the current state object
                void move() {
                    currentState_->handleMovement(this);
                }

                void managePower() {
                    currentState_->handlePower(this);
                }
        };

        // Concrete State 1: Robot is Moving and Performing Tasks
        class NavigatingState : public RobotState {
            public:
                std::string getStateName() const override { 
                    return "NAVIGATING"; 
                }

                void handleMovement(RobotContext* context) override {
                    std::cout << "NAVIGATING: Moving at full speed, running pathfinding algorithms." << std::endl;
                    // Navigation logic...
                }

                void handlePower(RobotContext* context) override {
                    // If battery is low, transition to ChargingState
                    int batteryLevel = 15; // Simulated low battery
                    if (batteryLevel < 20) {
                        std::cout << "NAVIGATING: Battery low, seeking charger." << std::endl;
                        // State transition logic
                        context->changeState(std::make_unique<class ChargingState>()); 
                    } else {
                        std::cout << "NAVIGATING: Power consumption is high." << std::endl;
                    }
                }
        };

        // Concrete State 2: Robot is Charging
        class ChargingState : public RobotState {
            public:
                std::string getStateName() const override { return "CHARGING"; }

                void handleMovement(RobotContext* context) override {
                    std::cout << "CHARGING: Movement is disabled to maintain connection." << std::endl;
                }

                void handlePower(RobotContext* context) override {
                    std::cout << "CHARGING: Actively drawing power and monitoring temperature." << std::endl;
                    // If battery is full, transition to SleepingState
                    int batteryLevel = 99; // Simulated full battery
                    if (batteryLevel > 95) {
                        std::cout << "CHARGING: Charge complete. Entering standby." << std::endl;
                        // State transition logic
                        context->changeState(std::make_unique<class SleepingState>());
                    }
                }
        };

        // Concrete State 3: Robot is in Low-Power Standby
        class SleepingState : public RobotState {
        public:
            std::string getStateName() const override { return "SLEEPING"; }

            void handleMovement(RobotContext* context) override {
                std::cout << "SLEEPING: All motors powered off. No movement allowed." << std::endl;
            }

            void handlePower(RobotContext* context) override {
                std::cout << "SLEEPING: Ultra-low power mode. Monitoring external wake-up signals." << std::endl;
                // If a new task comes in, transition to NavigatingState
                bool newTaskReceived = true; // Simulated event
                if (newTaskReceived) {
                    std::cout << "SLEEPING: New task received! Waking up." << std::endl;
                    // State transition logic
                    context->changeState(std::make_unique<NavigatingState>()); 
                }
            }
        };

        void run()
        {
            // 1. Initialize the robot with an initial state (e.g., Navigating)
            auto initial_state = std::make_unique<NavigatingState>();
            RobotContext robot(std::move(initial_state));

            std::cout << "\n--- Scenario 1: Running Task ---\n";
            robot.move();      // Executes NavigatingState::handleMovement
            robot.managePower(); // Executes NavigatingState::handlePower -> Causes transition!

            std::cout << "\n--- Scenario 2: Charging Cycle ---\n";
            robot.move();      // Executes ChargingState::handleMovement
            robot.managePower(); // Executes ChargingState::handlePower -> Causes transition!
            
            std::cout << "\n--- Scenario 3: Sleeping and Waking ---\n";
            robot.move();      // Executes SleepingState::handleMovement
            robot.managePower(); // Executes SleepingState::handlePower -> Causes transition!

            std::cout << "\n--- Scenario 4: Final State Check ---\n";
            robot.move();      // Back in NavigatingState
        }
    } // namespace State
    
*/


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
    return 0;
}