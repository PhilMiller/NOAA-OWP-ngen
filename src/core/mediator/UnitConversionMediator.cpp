#include "UnitConversionMediator.hpp"

#include <limits>
#include <sstream>
#include <tuple>

#include "utilities/logging_utils.h"

std::set<UnitConversionMediator::error_log_key> UnitConversionMediator::errors_reported;

bool UnitConversionMediator::error_log_key::operator<(error_log_key const& rhs) const
{
    return std::tie(requester_label, variable, units)
         < std::tie(rhs.requester_label, rhs.variable, rhs.units);
}

UnitConversionMediator::error_log_key UnitConversionMediator::make_key(conversion_request const& request)
{
    return error_log_key{request.requester_label, request.variable, request.units};
}

bool UnitConversionMediator::known_bad(conversion_request const& request)
{
    return !errors_reported.empty() && errors_reported.count(make_key(request)) != 0;
}

void UnitConversionMediator::log(conversion_request const& request,
                                    UnitsHelper::unit_conversion_exception const& uce)
{
    const bool new_error = errors_reported.insert(make_key(request)).second;
    if (!new_error) {
        return;
    }

    const double raw_value = uce.unconverted_values.empty()
        ? std::numeric_limits<double>::quiet_NaN()
        : uce.unconverted_values.front();

    std::stringstream ss;
    ss << "Unit conversion failure:"
       << " requester {'" << request.requester_label
       << "' feature '" << request.feature_id
       << "' variable '" << request.variable << "'";
    if (!request.alias.empty()) {
        ss << " (alias '" << request.alias << "')";
    }
    ss << " units '" << request.units << "'}"
       << " provider {'" << uce.provider_model_name
       << "' source variable '" << uce.provider_var_name << "'"
       << " raw value " << raw_value << "}"
       << " message \"" << uce.what() << "\"";
    logging::warning(ss.str().c_str());
}
