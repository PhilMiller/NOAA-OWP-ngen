#ifndef NGEN_UNITCONVERSIONMEDIATOR_H
#define NGEN_UNITCONVERSIONMEDIATOR_H

#include <limits>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "UnitsHelper.hpp"

/**
 * Mediates unit conversions between models and their data providers.
 *
 * UnitsHelper attempts the conversion and throws a typed
 * UnitsHelper::unit_conversion_exception on failure.
 * The mediator logs that failure _once_ per distinct mismatch, remembers it,
 * and skips any future attempt to convert the same _request_.
 */
class UnitConversionMediator {

    public:

    /**
     * Details identifying the requested a unit conversion, used both
     * to build the human-readable failure report and to track
     * repeated failures. Everything here is knowable before attempting the
     * conversion, so the same request identifies a failure before and after.
     */
    struct conversion_request {
        std::string requester_label;  //!< Human-readable identity of the requester
        std::string feature_id;       //!< Feature the request was made on behalf of
        std::string variable;         //!< Variable name the requester asked for
        std::string alias;            //!< Mapped alias for @ref variable, or "" when not applicable
        std::string units;            //!< Units the value was requested to be converted to
    };

    /**
     * @return whether the conversion described by @p request has already failed
     *         (and been logged) once.
     */
    static bool known_bad(conversion_request const& request);

    /**
     * Record and log a unit-conversion failure, once per unique failure
     */
    static void log(conversion_request const& request,
                       UnitsHelper::unit_conversion_exception const& uce);

    /**
     * Fetch a value (scalar or array) with unit conversion, attempting the
     * (throwing) conversion at most once per distinct mismatch.
     *
     * @p fetch is a callable taking the output units to request and returning
     * the value(s); passing "" requests no conversion.
     * 
     * The first time a given @p request is seen its requested units
     * are passed to the fetch method; if the conversion fails 
     * (with a unit_conversion_exception) it is logged once and remembered,
     * and every subsequent call for the same request forwards the request with ""
     * as a no-conversion, known-bad request.
     *
     * On success the converted value(s) are returned. When the conversion is
     * skipped (already known-bad) or fails on this attempt, the unconverted
     * value(s) are returned instead. The return type matches @p fetch.
     */
    template <typename Fetch>
    static auto convert(conversion_request const& request, Fetch fetch)
        -> decltype(fetch(std::string{}))
    {
        using result_type = decltype(fetch(std::string{}));
        if (known_bad(request)) {
            // skip conversion by passing "" as the requested unit
            return fetch(std::string{});
        }
        try {
            return fetch(request.units);
        }
        catch (UnitsHelper::unit_conversion_exception& uce) {
            log(request, uce);
            if constexpr (std::is_same_v<result_type, std::vector<double>>) {
                return std::move(uce.unconverted_values); // array: fall back to unconverted values
            } else {
                return uce.unconverted_values.empty()     // scalar: fall back to the single raw value
                    ? std::numeric_limits<double>::quiet_NaN()
                    : uce.unconverted_values.front();
            }
        }
    }

    private:

    /**
     * Key structure for tracking bad conversion requests.
     * Fields are a subset of the @ref conversion_request fields.
     * 
     * This key can be extended to track failures more granularly
     * at the expense of more log entries.
     */
    struct error_log_key {
        std::string requester_label;
        std::string variable;
        std::string units;

        bool operator<(error_log_key const& rhs) const;
    };

    // Marshal a conversion_request into an error_log_key for tracking
    static error_log_key make_key(conversion_request const& request);

    // assumes single-threaded access. Guard with a mutex
    // if this is ever accessed concurrently.
    static std::set<error_log_key> errors_reported;
};

#endif //NGEN_UNITCONVERSIONMEDIATOR_H
