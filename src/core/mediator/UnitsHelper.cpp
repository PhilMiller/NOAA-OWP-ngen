#include "UnitsHelper.hpp"
#include <cstring>
#include <mutex>

ut_system* UnitsHelper::unit_system;
std::once_flag UnitsHelper::unit_system_inited;
std::map<std::string, std::shared_ptr<cv_converter>> UnitsHelper::converters;
std::mutex UnitsHelper::converters_mutex;

std::shared_ptr<cv_converter> UnitsHelper::get_converter(const std::string& in_units, const std::string& out_units, utEncoding in_encoding, utEncoding out_encoding ){
    if(in_units == "" || out_units == ""){
        throw std::runtime_error("Unable to process empty units value for pairing \"" + in_units + "\" \"" + out_units + "\"");
    }

    const std::lock_guard<std::mutex> lock(converters_mutex);

    std::string key = in_units + "|" + out_units; //Better solution? Good enough? Bother with nested maps?
    if(converters.count(key) == 1){
        if(converters[key] == nullptr){
            // same as last throw below
            throw std::runtime_error("Unable to convert " + in_units + " to " + out_units);
        }
        return converters[key];
    } else {
        ut_unit* from = ut_parse(unit_system, in_units.c_str(), in_encoding);
        if (from == NULL)
        {
            throw std::runtime_error("Unable to parse in_units value " + in_units);
        }
        ut_unit* to = ut_parse(unit_system, out_units.c_str(), out_encoding);
        if (to == NULL)
        {
            ut_free(from);
            throw std::runtime_error("Unable to parse out_units value " + out_units);
        }
        cv_converter* conv = ut_get_converter(from, to);
        if (conv == NULL)
        {
            ut_free(from);
            ut_free(to);
            converters[key] = nullptr;
            throw std::runtime_error("Unable to convert " + in_units + " to " + out_units);
        }
        auto c = std::shared_ptr<cv_converter>(
            conv,
            [from,to](cv_converter* p) {
                cv_free(p);
                ut_free(from); // Captured via closure!
                ut_free(to); // Captured via closure!
            }
        );
        converters[key] = c;

        return c;
    }
}

double UnitsHelper::get_converted_value(const std::string &in_units, const double &value, const std::string &out_units)
{
    if(in_units == out_units){
        return value; // Early-out optimization
    }
    std::call_once(unit_system_inited, init_unit_system);

    auto converter = get_converter(in_units, out_units);

    double r = cv_convert_double(converter.get(), value);
    return r;
}

void UnitsHelper::convert_values(const std::string &in_units, boost::span<double> in_values, const std::string &out_units, boost::span<double> out_values)
{
    if (in_values.size() != out_values.size()) {
        throw std::runtime_error("Mismatched span sizes in unit conversion");
    }
    auto count = in_values.size();

    if (in_units == out_units) {
        // Early-out optimization
        if(in_values.data() == out_values.data()) {
            return;
        } else {
            memcpy(out_values.data(), in_values.data(), sizeof(double)*count);
            return;
        }
    }

    std::call_once(unit_system_inited, init_unit_system);

    auto converter = get_converter(in_units, out_units);
    cv_convert_doubles(converter.get(), in_values.data(), count, out_values.data());
}

