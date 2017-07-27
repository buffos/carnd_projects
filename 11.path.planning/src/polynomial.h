#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

struct Polynomial{
    vector<double> coefficients;
    vector<double> d_coefficients; // first derivative
    vector<double> dd_coefficients; // second derivative
    vector<double> ddd_coefficients; // third derivative

    Polynomial();
    Polynomial(const Polynomial &other);
    Polynomial(vector<double> const& coeff);
    double evalAt(double x, int order);
};

#endif //! POLYNOMIAL_H