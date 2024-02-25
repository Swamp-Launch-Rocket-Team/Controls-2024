#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;


int main()
{

double theta_0 = 13;      //Theta value, this will be an input into this function from the IMU

    vector<double> m_theta{ 0.000525089, 0.000690884, 0.001009584, 0.001398228, 0.001801924 };    //Slopes for linear region, determined in excel

    vector<int> theta_region(8501);     //Size of theta region

    for (size_t i = 0; i < theta_region.size(); i++)        //Sets theta region from altitude of 2500 ft o 11k feet
    {
        theta_region[i] = i + 2500;
    }

    vector<double> theta_vector(theta_region.size());       //initializes theta vector, same size as theta region

    //Linear fit region, 2.5k ft to 7k ft
    double b;

    if (theta_0 <= 7)        //All of the if statements for theta_0
    {
        b = theta_0 - m_theta[0] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[0] * theta_region[i] + b;
        }
    }
    else if (theta_0 < 7 && theta_0 < 10)
    {
        b = theta_0 - m_theta[1] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[1] * theta_region[i] + b;
        }
    }
    else if (theta_0 >= 10 && theta_0 < 14)
    {
        b = theta_0 - m_theta[2] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[2] * theta_region[i] + b;
        }
    }
    else if (theta_0 >= 14 && theta_0 < 19)
    {
        b = theta_0 - m_theta[3] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[3] * theta_region[i] + b;
        }
    }
    else
    {
        b = theta_0 - m_theta[4] * 2500;
        for (int i = 0; i < 4501; i++)
        {
            theta_vector[i] = m_theta[4] * theta_region[i] + b;
        }
    }
    //End of Linear fit region, ends at index 4500 at an altitude of 7k feet

    //Start of the Quadratic fit region, 7k ft to 10k ft
    vector<double> a_theta{ 8.26652482191255e-7, 1.03558936423213e-6, 1.53275631191493e-6, 2.17922684530253e-6, 2.92066636707301e-6 };

    int h_theta = 0;        //Parabola parameter for quadratic region
    double k_theta = theta_vector[4500];        //Initial value of quadratic region

    if (theta_0 < 7)     //if statements for the different initial thetas
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[0] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 < 7 && theta_0 < 10)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[1] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 >= 10 && theta_0 < 14)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[2] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else if (theta_0 >= 14 && theta_0 < 19)
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[3] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    else
    {
        for (int i = 4501; i < 7501; i++)
        {
            theta_vector[i] = a_theta[4] * pow(((theta_region[i] - 7000) - -h_theta), 2) + k_theta;
        }
    }
    //End of Quadratic fit region, ends at index 7500 at an altitude of 10k feet

    //Region after Quadratic region, increase linearly until 90 degrees at a steep slope
    double inc = 0.1;        //increment for the linear section past the quadratic region
    vector<double> int_vec(1000);      //interval vector initialization

    for (size_t i = 0; i < int_vec.size(); i++)     //interval vector definition, 1:1:1000
    {
        int_vec[i] = i + 1.0;
    }

    for (size_t i = 7501; i < theta_vector.size(); i++)     //adds the last linear section past quadratic region
    {
        theta_vector[i] = theta_vector[7500] + inc * int_vec[i - 7501];
    }

    for (int i = 0; i < theta_vector.size(); i++)
    {
        //cout << theta_vector[i] << endl;
    }


    cout << theta_region.size() << endl;
    cout << theta_vector.size() << endl;
    return 0;
}

