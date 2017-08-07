/**
 * @file polynomial.h
 * @brief A simple polynomial object that gets coefficients and calculates values
 * at points of the polynomial and its derivatives
 * *
 * @author  Kostas Oreopoulos
 */
#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

struct Polynomial {
  vector<double> coefficients;
  vector<double> d_coefficients;   ///< first derivative
  vector<double> dd_coefficients;  ///< second derivative
  vector<double> ddd_coefficients; ///< third derivative

  Polynomial();
  Polynomial(const Polynomial &other);
  Polynomial(vector<double> const &coefficient);
  /**
   * Evaluate the polynomial at position X
   * @param x  The position we want to evaluate the polynomial
   * @param order The derivate we want to evaluate (0 = original poly)
   * @return The value at position X
   */
  double evalAt(double x, int order);
};

#endif //! POLYNOMIAL_H