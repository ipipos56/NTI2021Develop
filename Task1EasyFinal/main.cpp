#include <bits/stdc++.h>

using namespace std;

#define double long double

double sqr(double x){return x*x;}

double ax=-1,ay=-1,bx=-1,by=-1;

void ThirdPointsFinder(double r,double a,double b,double c)
{
    double xt = -a*c/(a*a+b*b),  yt = -b*c/(a*a+b*b);

    double d = r*r - c*c/(a*a+b*b);
    double mult = sqrt (d / (a*a+b*b));

    ax = xt + b * mult;
    bx = xt - b * mult;
    ay = yt - a * mult;
    by = yt + a * mult;
}

double FindAngle(double _x, double _y)
{
    double locbeta = atan(abs(_y)/abs(_x));
    if(_y < 0 && _x > 0)
        locbeta = 2 * M_PI - locbeta;
    else if(_y < 0 && _x < 0)
        locbeta = M_PI + locbeta;
    else if(_y > 0 && _x < 0)
        locbeta = M_PI - locbeta;
    return locbeta;
}

int main()
{
    double beta_min,beta_max,gamma_min,gamma_max;
    double l1,l2;
    double x0,y0;
    cin>>beta_min>>beta_max>>gamma_min>>gamma_max>>l1>>l2>>x0>>y0;

    double beta = -1,gamma = -1;

    double l3 = sqrt(x0 * x0 + y0 * y0);
    double tt = ((l1 * l1 + l2 * l2 - l3 * l3)/(2 * l1 * l2));
    gamma = acos(tt);
    bool FirstPoint = true;
    if(!(gamma_min < gamma && gamma_max > gamma))
    {
        gamma = M_PI * 2 - gamma;
        FirstPoint = false;
    }
    if(!(gamma_min < gamma && gamma_max > gamma))
    {
        cout<<"-1"<<endl;
        return 0;
    }
    ThirdPointsFinder(l1,-2*x0,-2*y0,sqr(x0)+sqr(y0)+sqr(l1)-sqr(l2));
    beta = FindAngle(ax,ay);
    if(!(beta_min < beta && beta_max > beta))
        beta = FindAngle(bx,by);
    else
    {
        double beta2 = FindAngle(bx,by);
        if(beta_min < beta2 && beta_max > beta2)
            if(!FirstPoint)
                beta = beta2;
    }
    if(!(beta_min < beta && beta_max > beta) || beta == -1000)
    {
        cout<<"-1"<<endl;
        return 0;
    }
    if(abs((l1 * cos(beta) - l2 * cos(beta + gamma)) - x0) > 0.001 && abs((l1 * sin(beta) - l2 * sin(beta + gamma)) - y0) > 0.001)
    {
        cout<<"-1"<<endl;
        return 0;
    }
    cout<<beta<<" "<<gamma<<endl<<endl;
}
