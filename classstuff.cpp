#include <iostream>

using namespace std;

class Gains
{
    float status = 10;

    public:
        float kp = 0.1;
        float ki = 0.05;
        Gains()
        {

        }

        Gains(float M, float z)
        {
            this->kp = 0.1*M;
            this->ki = 0.05*z;
        }
};

int main()
{
    Gains mine(1,100);

    cout << mine.kp << "\t" << mine.ki << endl;

    return 0;
}