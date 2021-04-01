#include <bits/stdc++.h>

using namespace std;

#define double long double

double sqr(double x)
{
    return x*x;
}

double ax=-1,ay=-1,bx=-1,by=-1;

void ThirdPointFinder(double r,double a,double b,double c)
{
    double xt = -a*c/(a*a+b*b),  yt = -b*c/(a*a+b*b);

    double d = r*r - c*c/(a*a+b*b);
    double mult = sqrt (d / (a*a+b*b));

    ax = xt + b * mult;
    bx = xt - b * mult;
    ay = yt - a * mult;
    by = yt + a * mult;
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

        double beta = -1,gamma = -1;

        double l3 = sqrt(x0 * x0 + y0 * y0);
        double tt = ((l1 * l1 + l2 * l2 - l3 * l3)/(2 * l1 * l2));
        gamma = acos(tt);
        double oldgamma = gamma;
        bool adder = true;
        if(!(gamma_min < gamma && gamma_max > gamma))
        {
            gamma = M_PI * 2 - gamma;
            adder = false;
        }
        if(!(gamma_min < gamma && gamma_max > gamma))
        {
            cout<<"-1"<<endl<<endl;
            //return 0;
            continue;
        }
        double x1,y1;
        ThirdPointFinder(l1,-2*x0,-2*y0,sqr(x0)+sqr(y0)+sqr(l1)-sqr(l2));
        beta = atan(abs(ay)/abs(ax));
        if(ay < 0 && ax > 0)
            beta = 2 * M_PI - beta;
        else if(ay < 0 && ax < 0)
            beta = M_PI + beta;
        else if(ay > 0 && ax < 0)
            beta = M_PI - beta;
        if(!(beta_min < beta && beta_max > beta))
        {
            beta = atan(abs(by)/abs(bx));
            if(by < 0 && bx > 0)
                beta = 2 * M_PI - beta;
            else if(by < 0 && bx < 0)
                beta = M_PI + beta;
            else if(by > 0 && bx < 0)
                beta = M_PI - beta;
        }
        else
        {
            double beta2 = atan(abs(by)/abs(bx));
            if(by < 0 && bx > 0)
                beta2 = 2 * M_PI - beta2;
            else if(by < 0 && bx < 0)
                beta2 = M_PI + beta2;
            else if(by > 0 && bx < 0)
                beta2 = M_PI - beta2;
            if(beta_min < beta2 && beta_max > beta2)
            {
                // l1 * cos(alpha) + l2 * sin(alpha + beta)  = x0
                // l1 * sin(alpha) + l2 * cos(alpha + beta) = y0
                //cout<<"Super "<<beta<<" "<<beta2<<endl;
                if(!adder)
                {
                    beta = beta2;
                }
            }
        }
        if(!(beta_min < beta && beta_max > beta) || beta == -1000)
        {
            cout<<"-1"<<endl<<endl;
            //return 0;
            continue;
        }
        if(abs((l1 * cos(beta) - l2 * cos(beta + gamma)) - x0) > 0.001 && abs((l1 * sin(beta) - l2 * sin(beta + gamma)) - y0) > 0.001)
        {
            cout<<"-1"<<endl<<endl;
            //return 0;
            continue;
        }
        cout<<beta<<" "<<gamma<<endl<<endl;
    }
}
