#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

//基础目标类
class BaseObject{
    public:
    virtual void search(){cout<<"发现目标"<<endl;}
    
};

class Armor:public BaseObject{
    public:
    virtual void search(){cout<<"发现装甲板"<<endl;}
    
    

};

class Buff:public BaseObject{
    public:
    virtual void search(){cout<<"发现大神符"<<endl;}
};

class Link{
    public:
    void run();

    private:
    shared_ptr<Buff> buff = make_shared<Buff>();
    shared_ptr<Armor>armor = make_shared<Armor>();
};

void Link::run(){
    shared_ptr<BaseObject> base = buff;
    base->search();
}

int main()
{
    unique_ptr<Link>Run = make_unique<Link>();
    Run->run();

    shared_ptr<int>first = make_shared<int>(10);
    cout<<"first使用次数"<<first.use_count()<<endl;
    
    shared_ptr<int>second = make_shared<int>(100);
    second = first;
    cout<<"first使用次数"<<first.use_count()<<endl;
    cout<<"second使用次数"<<second.use_count()<<endl;
    first.reset();
    cout<<"second = "<<*second<<endl;
    cout<<"first使用次数"<<first.use_count()<<endl;
    cout<<"second使用次数"<<second.use_count()<<endl;

    system("pause");
}