/**
 * @file polynomial.cpp
 * @brief Implementation of the polynomial class
 *
 *
 * @author  Kostas Oreopoulos
 */

#include "polynomial.h"

Polynomial::Polynomial() {
}

Polynomial::Polynomial(const Polynomial &other) {
  coefficients = other.coefficients;
  d_coefficients = other.d_coefficients;
  dd_coefficients = other.dd_coefficients;
  ddd_coefficients = other.ddd_coefficients;
}

Polynomial::Polynomial(vector<double> const &coeff) {
  for (unsigned int i = 0; i < coeff.size(); i++) {
    coefficients.push_back(coeff[i]);
    if (i > 0) {
      d_coefficients.push_back(i * coeff[i]); // derivative
    }
    if (i > 1) {
      dd_coefficients.push_back(i * (i - 1) * coeff[i]); // second derivative
    }
    if (i > 2) {
      ddd_coefficients.push_back(i * (i - 1) * (i - 2) * coeff[i]); // second derivative
    }
  }
}

double Polynomial::evalAt(double x, int order) {
  double result = 0.0;
  vector<double> *c = &coefficients;
  if (order == 1) {
    c = &d_coefficients;
  } else if (order == 2) {
    c = &dd_coefficients;
  } else if (order == 3) {
    c = &ddd_coefficients;
  }
  for (unsigned int i = 0; i < (*c).size(); i++) {
    result += (*c)[i] * pow(x, i);
  }
  c = nullptr;
  return result;
}