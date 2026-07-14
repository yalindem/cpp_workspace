#include <iostream>
#include <utility>

struct Data
{
    explicit Data()
    {}

    Data& operator=(const Data& other)
    {
        std::cout << "copy aaasigment\n";
        return *this;
    }

    Data& operator=(Data&& other)
    {
        std::cout << "Move assigment\n";
        return *this;
    }
};

class Container{
    public:
        /*
        void Put(const Data& data)
        {
            std::cout << "here1\n";
            data_ = data;
        }
        
        void Put(Data&& data)
        {
            std::cout << "here2\n";
            data_ = std::move(data);
        }
        */
        template<typename T>
        void Put(T&& data)
        {
            data_ = std::forward<T>(data);
        }

    private:
        Data data_;
};


int main()
{
    Container con;
    Data data;
    con.Put(data);
    con.Put(Data{});
    return 0;
}