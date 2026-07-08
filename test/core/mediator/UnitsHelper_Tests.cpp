#include "gtest/gtest.h"

#include "core/mediator/UnitsHelper.hpp"

class UnitsHelper_Test : public ::testing::Test {

    public:
        void SetUp() override {
        }

    protected:

        UnitsHelper_Test() {
        }

        ~UnitsHelper_Test() {
        }

};

TEST_F(UnitsHelper_Test, TestDoAConversion){
    ASSERT_NEAR(32.0, UnitsHelper::get_converted_value("degC", 0, "degF"), 0.000000001);
    ASSERT_NEAR(0.0, UnitsHelper::get_converted_value("degF", 32, "degC"), 0.000000001);

}

TEST_F(UnitsHelper_Test, TestConvertArray){
    std::vector<double> data = {1,2,3,4};
    std::vector<double> expected = {1000, 2000, 3000, 4000};
    //Call the converter, updates data in place
    UnitsHelper::convert_values("m", data.data(), "mm", data.data(), data.size());
    ASSERT_EQ( expected,  data);

}

// For coverage completeness...
TEST_F(UnitsHelper_Test, TestConvertArrayNoOp){
    std::vector<double> data = {1,2,3,4};
    std::vector<double> expected = {1, 2, 3, 4};
    //Call the converter, updates data in place
    UnitsHelper::convert_values("m", data.data(), "m", data.data(), data.size());
    ASSERT_EQ( expected,  data);
}

TEST_F(UnitsHelper_Test, TestConvertArrayDontModifyInput){
    std::vector<double> data = {1,2,3,4};
    std::vector<double> data2 = {2,4,6,8};
    std::vector<double> expected = {1000, 2000, 3000, 4000};
    //Call the converter, updates data in place
    UnitsHelper::convert_values("m", data.data(), "mm", data2.data(), data.size());
    ASSERT_EQ( expected,  data2);
    ASSERT_EQ( data.at(2), 3);
}

TEST_F(UnitsHelper_Test, TestConvertArrayDontModifyInputNoOp){
    std::vector<double> data = {1,2,3,4};
    std::vector<double> data2 = {2,4,6,8};
    std::vector<double> expected = {1, 2, 3, 4};
    //Call the converter, updates data in place
    UnitsHelper::convert_values("m", data.data(), "m", data2.data(), data.size());
    ASSERT_EQ( expected,  data2);
    ASSERT_EQ( data.at(2), 3);
}

// An unconvertible pair throws a unit_conversion_exception carrying the units
// and the input value, so a requester can report and fall back to it.
TEST_F(UnitsHelper_Test, TestUnconvertiblePairThrowsAndKeepsValue){
    try {
        UnitsHelper::get_converted_value("m", 5.0, "kg");
        FAIL() << "expected a unit_conversion_exception";
    } catch (UnitsHelper::unit_conversion_exception& uce) {
        ASSERT_EQ(uce.unconverted_values.size(), 1u);
        EXPECT_DOUBLE_EQ(uce.unconverted_values[0], 5.0);
        EXPECT_EQ(uce.provider_units, "m");
        EXPECT_EQ(uce.to_units, "kg");
    }
}

// Unparseable units likewise throw (rather than silently converting).
TEST_F(UnitsHelper_Test, TestUnparseableUnitsThrow){
    EXPECT_THROW(UnitsHelper::get_converted_value("not_a_unit", 1.0, "m"),
                 UnitsHelper::unit_conversion_exception);
}

// Identical units short-circuit and return the value unchanged.
TEST_F(UnitsHelper_Test, TestSameUnitsShortCircuit){
    EXPECT_DOUBLE_EQ(5.0, UnitsHelper::get_converted_value("m", 5.0, "m"));
}

// The none-ish spellings are all treated as the dimensionless unit "1", and a
// none-ish requested output against real input units passes the value through.
TEST_F(UnitsHelper_Test, TestNoneIshTreatedAsDimensionless){
    for (const std::string& noneish : {std::string(""), std::string("none"),
                                       std::string("unitless"), std::string("dimensionless"),
                                       std::string("-")}) {
        EXPECT_DOUBLE_EQ(5.0, UnitsHelper::get_converted_value(noneish, 5.0, "1"))
            << "input units '" << noneish << "' should be dimensionless";
        EXPECT_DOUBLE_EQ(5.0, UnitsHelper::get_converted_value("m", 5.0, noneish))
            << "none-ish requested output '" << noneish << "' should pass through";
    }
}
