//斐波那契数列
#include<iostream>
using namespace std;
int value(int b)
{
    if (b<0) return -1;
    if (b==1 || b==2) return 1;
    else return value(b-1)+value(b-1);
}
int main()
{
    int n;
    cin >>n;
    cout <<value(n)<<endl;
    return 0;
}