#include <bits/stdc++.h>

using namespace std;

double sqr(double x)
{
    return x*x;
}

double pointXFind(double a, double c, double x1,double y1, double x2, double y2)
{
    return (1/2)*((y1-y2)*sqrt(-1 *(sqr(-1 * x1)+2*x2*x1-sqr(x2)+(-c+a-y1+y2)*(-c+a+y1-y2))*(sqr(-1 * x1)+2*x2*x1-sqr(x2)+(c+a-y1+y2)*(c+a+y1-y2))*sqr(x1-x2))+(sqr(x1)*x1-sqr(x1)*x2+(sqr(y2)-2*y1*y2-sqr(c)+sqr(y1)+sqr(a)-sqr(x2))*x1-x2*(sqr(a)-sqr(c)-sqr(x2)-sqr(y2)+2*y1*y2-sqr(y1)))*(x1-x2))/((x1-x2)*(sqr(x1)-2*x2*x1+sqr(x2)+sqr(y1-y2)));
}

double pointYFind(double a, double c, double x1,double y1, double x2, double y2)
{
    return (-1 * sqrt(-1 * (sqr(-x1)+2*x2*x1-sqr(x2)+(-c+a-y1+y2)*(-c+a+y1-y2))*(sqr(-x1)+2*x2*x1-sqr(x2)+(c+a-y1+y2)*(c+a+y1-y2))*sqr(x1-x2))+sqr(y1)*y1-sqr(y1)*y2+(sqr(a)+sqr(x1)-sqr(c)+sqr(x2)-2*x2*x1-sqr(y2))*y1+sqr(y2)*y2+(sqr(x2)-2*x2*x1+sqr(c)-sqr(a)+sqr(x1))*y2)/(2*sqr(y1)-4*y1*y2+2*sqr(y2)+2*sqr(x1-x2));
}

int main()
{
    freopen("input.txt","r",stdin);
    freopen("output.txt","w",stdout);
    for(int co = 0;co<100;co++)
    {
        double beta_min,beta_max,gamma_min,gamma_max;
        double l1,l2;
        double x0,y0;
        cin>>beta_min>>beta_max>>gamma_min>>gamma_max>>l1>>l2>>x0>>y0;
        //cout<<beta_min<<" "<<beta_max<<" "<<gamma_min<<" "<<gamma_max<<" "<<l1<<" "<<l2<<" "<<x0<<" "<<y0<<endl;

        //beta_max = beta_max * 180 / M_PI;
        //beta_min = beta_min * 180 / M_PI;

        //gamma_max = gamma_max * 180 / M_PI;
        //gamma_min = gamma_min * 180 / M_PI;

        double beta = -1,gamma = -1;

        double l3 = sqrt(x0 * x0 + y0 * y0);
        //cout<<l3<<endl;
        double tt = ((l1 * l1 + l2 * l2 - l3 * l3)/(2 * l1 * l2));
        //cout<<tt<<endl;
        gamma = acos(tt);
        double oldgamma = gamma;
        //cout<<gamma * 180 / M_PI<<endl;
        //cout<<M_PI * 2 - gamma<<endl;
        bool adder = true;
        if(!(gamma_min < gamma && gamma_max > gamma))
        {
            gamma = M_PI * 2 - gamma;
            adder = false;
        }
        //cout<<gamma<<endl;
        if(!(gamma_min < gamma && gamma_max > gamma))
        {
            cout<<"-1"<<endl<<endl;
            continue;
        }
        double alfa = asin(l2 * sin(oldgamma)/l3);
        //beta = asin(l2 * sin(oldgamma)/l3) - atan(y0 / x0);
        //double x1,y1;
        //x1 = pointXFind(l2,l1,0,0,x0,y0);
        //y1 = pointYFind(l2,l1,0,0,x0,y0);
        beta = atan(abs(y0)/abs(x0));
        //cout<<y0<<" "<<x0<<endl;
        if(y0 < 0 && x0 > 0)
            beta = 2 * M_PI - beta;
        else if(y0 < 0 && x0 < 0)
            beta = M_PI + beta;
        else if(y0 > 0 && x0 < 0)
            beta = M_PI - beta;
        if(adder)
            beta+=alfa;
        else
            beta-=alfa;
        //cout<<x1<<" "<<y1<<" "<<beta<<endl;
        //cout<<beta<<endl;
    //    double D = x0 * y1 - y0 * x1;
     //   if(D < 0)
     //       beta += alfa;
     //   else if(D > 0)
     //       beta -= alfa;
        if(!(beta_min < beta && beta_max > beta))
        {
            cout<<"-1"<<endl<<endl;
            continue;
        }
        cout<<beta<<" "<<gamma<<endl<<endl;
    }
}
