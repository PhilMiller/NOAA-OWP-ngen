#include "gtest/gtest.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "core/mediator/UnitsHelper.hpp"
#include "core/mediator/UnitConversionMediator.hpp"

// Tests for the fire-once unit-conversion handling in UnitConversionMediator:
// convert attempts a failing conversion only once per distinct
// mismatch, then passes the value(s) through unconverted without throwing again.
//
// The mediator's recorded-errors set is process-global, so each test uses a
// unique requester label to stay isolated from the others.

class UnitConversionMediatorTest : public ::testing::Test {
protected:
    // Build an exception as the conversion layer would: carrying the raw value(s).
    static UnitsHelper::unit_conversion_exception make_uce(std::vector<double> raw)
    {
        UnitsHelper::unit_conversion_exception uce{"Unable to convert as requested", "furlong", "m"};
        uce.provider_model_name = "mock_provider";
        uce.provider_var_name = "V";
        uce.unconverted_values = std::move(raw);
        return uce;
    }
};

TEST_F(UnitConversionMediatorTest, KnownBadBecomesTrueOnlyAfterLog)
{
    UnitConversionMediator::conversion_request req{"mediator-known-bad", "cat-1", "V", "", "m"};
    EXPECT_FALSE(UnitConversionMediator::known_bad(req));

    UnitConversionMediator::log(req, make_uce({9.0}));
    EXPECT_TRUE(UnitConversionMediator::known_bad(req));
}

TEST_F(UnitConversionMediatorTest, ScalarAttemptsFailingConversionOnlyOnce)
{
    UnitConversionMediator::conversion_request req{"mediator-scalar", "cat-1", "V", "", "m"};

    std::string last_units = "<none>";
    int throwing_attempts = 0;

    auto fetch = [&](std::string const& units) -> double {
        last_units = units;
        if (!units.empty()) {           // a real conversion attempt
            ++throwing_attempts;
            throw make_uce({42.0});
        }
        return 42.0;                    // "" == pass-through returns the raw value
    };

    // First call: requests the desired units, fails, logs, returns raw value.
    double first = UnitConversionMediator::convert(req, fetch);
    EXPECT_DOUBLE_EQ(first, 42.0);
    EXPECT_EQ(last_units, "m");
    EXPECT_TRUE(UnitConversionMediator::known_bad(req));

    // Second call: known-bad, so it takes the pass-through path and never throws.
    double second = UnitConversionMediator::convert(req, fetch);
    EXPECT_DOUBLE_EQ(second, 42.0);
    EXPECT_EQ(last_units, "");
    EXPECT_EQ(throwing_attempts, 1);    // fire-once: only ever attempted the throwing path once
}

TEST_F(UnitConversionMediatorTest, ArrayAttemptsFailingConversionOnlyOnce)
{
    UnitConversionMediator::conversion_request req{"mediator-array", "cat-1", "V", "", "m"};

    const std::vector<double> raw = {1.0, 2.0, 3.0};
    std::string last_units = "<none>";
    int throwing_attempts = 0;

    auto fetch = [&](std::string const& units) -> std::vector<double> {
        last_units = units;
        if (!units.empty()) {
            ++throwing_attempts;
            throw make_uce(raw);
        }
        return raw;
    };

    std::vector<double> first = UnitConversionMediator::convert(req, fetch);
    EXPECT_EQ(first, raw);
    EXPECT_EQ(last_units, "m");

    std::vector<double> second = UnitConversionMediator::convert(req, fetch);
    EXPECT_EQ(second, raw);
    EXPECT_EQ(last_units, "");
    EXPECT_EQ(throwing_attempts, 1);
}

TEST_F(UnitConversionMediatorTest, DistinctRequestsAreTrackedIndependently)
{
    UnitConversionMediator::conversion_request a{"mediator-distinct", "cat-1", "A", "", "m"};
    UnitConversionMediator::conversion_request b{"mediator-distinct", "cat-1", "B", "", "m"};

    auto fail = [](std::string const& units) -> double {
        if (!units.empty()) throw make_uce({1.0});
        return 1.0;
    };

    UnitConversionMediator::convert(a, fail);
    EXPECT_TRUE(UnitConversionMediator::known_bad(a));
    EXPECT_FALSE(UnitConversionMediator::known_bad(b)); // b never attempted
}

TEST_F(UnitConversionMediatorTest, SuccessfulConversionIsNotRecorded)
{
    UnitConversionMediator::conversion_request req{"mediator-success", "cat-1", "V", "", "m"};
    auto ok = [](std::string const&) -> double { return 7.0; };

    EXPECT_DOUBLE_EQ(UnitConversionMediator::convert(req, ok), 7.0);
    EXPECT_FALSE(UnitConversionMediator::known_bad(req));
}

TEST_F(UnitConversionMediatorTest, ScalarFallsBackToNanWhenExceptionCarriesNoValue)
{
    UnitConversionMediator::conversion_request req{"mediator-nan", "cat-1", "V", "", "m"};
    auto fetch = [](std::string const& units) -> double {
        if (!units.empty()) throw make_uce({}); // no unconverted values carried
        return std::numeric_limits<double>::quiet_NaN();
    };
    EXPECT_TRUE(std::isnan(UnitConversionMediator::convert(req, fetch)));
}
