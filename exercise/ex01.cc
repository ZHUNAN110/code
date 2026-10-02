//斐波那契数列
#include<iostream>
using namespace std;
int value(int a)
{
    if (a<0) return -1;
    if (a==1 || a==2) return 1;
    else return value(a-1)+value(a-1);
}
int main()
{
    int n;
    cin >>n;
    cout <<value(n)<<endl;
    return 0;
}