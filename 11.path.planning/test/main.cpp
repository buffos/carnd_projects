#include "gtest/gtest.h"
#include "gmock/gmock.h"


// using namespace ::testing;


TEST(FIRST_TEST_CATEGORY, TEST_NAME) {
	EXPECT_EQ(3, 3);
}


int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}