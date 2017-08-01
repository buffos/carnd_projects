#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "polynomial.h"

using namespace std;
using namespace testing;

TEST(PolynomialTests, correctlyDefaultConstructor) {
	Polynomial p;
	EXPECT_THAT(p.coefficients, IsEmpty());
	EXPECT_THAT(p.d_coefficients, IsEmpty());
	EXPECT_THAT(p.dd_coefficients, IsEmpty());
	EXPECT_THAT(p.ddd_coefficients, IsEmpty());
}

TEST(PolynomialTests, correctlyInitilizesFromCoefficients) {
	vector<double> c{ 1.0, 1.0, 1.0 }; // 1 + x + x^2
	Polynomial p(c);
	EXPECT_THAT(p.coefficients, ElementsAre(1.0, 1.0, 1.0));
	EXPECT_THAT(p.d_coefficients, ElementsAre(1.0, 2.0));
	EXPECT_THAT(p.dd_coefficients, ElementsAre(2.0));
	EXPECT_THAT(p.ddd_coefficients, IsEmpty());
}


TEST(PolynomialTests, correctlyInitilizesFromCoefficients2) {
	vector<double> c{ 1.0, 2.0, 3.0, 4.0 }; // 1 + 2x + 3x^2 + 4x^3
	Polynomial p(c);
	EXPECT_THAT(p.coefficients, ElementsAre(1.0, 2.0, 3.0, 4.0));
	EXPECT_THAT(p.d_coefficients, ElementsAre(2.0, 6.0, 12.0));
	EXPECT_THAT(p.dd_coefficients, ElementsAre(6.0, 24.0));
	EXPECT_THAT(p.ddd_coefficients, ElementsAre(24.0));
}

TEST(PolynomialTests, correctlyEvaluatesPolynomial) {
	vector<double> c{ 1.0, 2.0, 3.0, 4.0 }; // 1 + 2x + 3x^2 + 4x^3
	Polynomial p(c);
	EXPECT_EQ(p.evalAt(1.0, 0), 10.0);
	EXPECT_EQ(p.evalAt(1.0, 1), 20.0);
	EXPECT_EQ(p.evalAt(1.0, 2), 30.0);
	EXPECT_EQ(p.evalAt(1.0, 3), 24.0);
}
