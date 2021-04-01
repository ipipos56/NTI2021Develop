#include <bits/stdc++.h>

using namespace std;

int main()
{
    freopen("input.txt","r",stdin);
    double x[100], y[100];
    int otv[100];
    for(int i = 0;i<100;i++)
    {
        cin>>x[i];
        if(x[i]!= -1)
            cin>>y[i];
        else
            y[i] = -1;
        cin>>otv[i];
    }
    int sum = 0;
    for(int i =0;i<100;i++)
    {
        double chX,chY;
        cin>>chX;
        if(chX !=-1)
            cin>>chY;
        else
            chY=-1;
        if(abs(x[i]-chX)<0.001 && abs(y[i]-chY)<0.001)
            sum++;
        else
            cout<<otv[i]<<" "<<x[i]<<" "<<y[i]<<" "<<chX<<" "<<chY<<endl;
    }
    cout<<endl;
    cout<<sum;
}
