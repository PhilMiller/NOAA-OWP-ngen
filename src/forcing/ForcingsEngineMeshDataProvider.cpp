#include "ForcingsEngineMeshDataProvider.hpp"
#include <bmi/Bmi_Py_Adapter.hpp>

namespace data_access {

using BaseProvider = ForcingsEngineDataProvider<double, MeshPointsSelector>;
using Provider = ForcingsEngineMeshDataProvider;

//! Mesh Forcings Engine instances storage
template<>
std::unordered_map<std::string, std::unique_ptr<BaseProvider>> BaseProvider::instances_{};

Provider::ForcingsEngineMeshDataProvider(
    const std::string& init,
    std::size_t time_begin_seconds,
    std::size_t time_end_seconds
)
  : BaseProvider(init, time_begin_seconds, time_end_seconds)
{

    // Initialize the value cache
    const auto id_dim   = static_cast<std::size_t>(bmi_->GetVarNbytes("CAT-ID") / bmi_->GetVarItemsize("CAT-ID"));
    const auto var_dim  = get_available_variable_names().size();

    bmi_->Update();
    this->increment_time();

    var_cache_ = decltype(var_cache_){{ 2, id_dim, var_dim }};

    // Cache initial iteration
    update_value_storage_();
    // this->increment_time(); // TODO: why???
}

BaseProvider* Provider::lumped_instance(
    const std::string& init,
    const std::string& time_begin,
    const std::string& time_end,
    const std::string& time_fmt
)
{
    auto time_begin_epoch = static_cast<size_t>(parse_time(time_begin, time_fmt));
    auto time_end_epoch = static_cast<size_t>(parse_time(time_end, time_fmt));

    auto provider = std::unique_ptr<ForcingsEngineMeshDataProvider>{
        new Provider{init, time_begin_epoch, time_end_epoch}
    };

    return set_instance(init, std::move(provider));
}

std::size_t Provider::variable_index(const std::string& variable) noexcept
{
    auto vars = this->get_available_variable_names();
    const auto* pos = std::find_if(
        vars.begin(),
        vars.end(),
        [&](const std::string& var) -> bool {
            // Checks for a variable name, plus the case where only the prefix is given, i.e.
            // both variable_index("U2D") and variable_index("U2D_ELEMENT") will work.
            return var == variable || var == (variable + "_ELEMENT");
        }
    );
    
    if (pos == vars.end()) {
        return bad_index;
    }

    // implicit cast to unsigned
    return std::distance(vars.begin(), pos);
}

bool Provider::next()
{
    bmi_->Update();
    this->increment_time();
    this->update_value_storage_();
    return true;
}

void Provider::update_value_storage_()
{
    const auto outputs = this->get_available_variable_names();
    const auto size = var_divides_.size();
    for (std::size_t vi = 0; vi < outputs.size(); vi++) {
        // Get current values for each variable
        const auto var_span = boost::span<const double>{
            static_cast<double*>(bmi_->GetValuePtr(outputs[vi])),
            size
        };

        for (std::size_t di = 0; di < size; di++) {
            // Move 0 time (current) to 1 (previous), and set the
            // current values from var_span
            var_cache_.at({{1, di, vi}}) = var_cache_.at({{0, di, vi}});
            var_cache_.at({{0, di, vi}}) = var_span[di];
        }
    }
}

double Provider::at(
    const std::string& divide_id,
    const std::string& variable,
    bool               previous
)
{
    const auto d_idx = divide_index(divide_id);
    const auto v_idx = variable_index(variable);

    if (d_idx == bad_index || v_idx == bad_index) {
        const auto shape = var_cache_.shape();
        const auto shape_str = std::to_string(shape[0]) + ", " +
                               std::to_string(shape[1]) + ", " +
                               std::to_string(shape[2]);

        const auto index_str = (d_idx == bad_index ? "?" : std::to_string(d_idx)) + ", " +
                               (v_idx == bad_index ? "?" : std::to_string(v_idx));

        throw std::out_of_range{
            "Failed to get ForcingsEngineMeshDataProvider value at index {" + index_str + "}" +
            "\n  Shape    : {" + shape_str + "}" +
            "\n  Divide   : `" + divide_id + "`" +
            "\n  Variable : `" + variable  + "`"
        };
    }

    return at(d_idx, v_idx, previous);
}

double Provider::at(
    std::size_t divide_idx,
    std::size_t variable_idx,
    bool      previous
)
{

    if (divide_idx == bad_index || variable_idx == bad_index) {
        const auto shape = var_cache_.shape();
        const auto shape_str = std::to_string(shape[0]) + ", " +
                               std::to_string(shape[1]) + ", " +
                               std::to_string(shape[2]);

        const auto index_str = (previous ? "1" : "0") + std::string{", "} +
                               (divide_idx == bad_index ? "?" : std::to_string(divide_idx)) + ", " +
                               (variable_idx == bad_index ? "?" : std::to_string(variable_idx));

        throw std::out_of_range{
            "Failed to get ForcingsEngineMeshDataProvider value at index {" + index_str + "}" +
            "\n  Shape          : {" + shape_str + "}"
        };
    }
    
    return var_cache_.at({{previous ? 1U : 0U, divide_idx, variable_idx}});
}

double Provider::get_value(
    const MeshSelector& selector,
    ReSampleMethod m
)
{
    throw std::runtime_error{"ForcingsEngineMeshDataProvider::get_value (scalar version) not implemented"};
}

std::vector<double> Provider::get_values(
    const MeshSelector& selector,
    ReSampleMethod m
)
{
    const std::string& var   = selector.get_variable_name();
    const auto start        = clock_type::from_time_t(selector.get_init_time());
    const auto end          = std::chrono::seconds{selector.get_duration_secs()} + start;
    const auto step         = std::chrono::seconds{this->record_duration()};

    size_t count;

    if (auto p = boost::get<all_points>(selector.points)) {
        switch (selector.location) {
        case MeshLocation::NODE:    count = num_nodes;    break;
        case MeshLocation::EDGE:    count = num_edges;    break;
        case MeshLocation::ELEMENT: count = num_elements; break;
        default:
            throw std::runtime_error{"Invalid/unhandled MeshLocation value"};
        }
    } else if (auto p = boost::get<std::vector<int>>(selector.points)) {
        count = p->size();
        throw std::runtime_error{"Point subset selection not implemented yet"};
    } else {
        throw std::runtime_error{"Unexpected variant for points selection"};
    }
    
    
    const auto var_span = boost::span<const double>{
        static_cast<double*>(bmi_->GetValuePtr(outputs[vi])),
        size
    };

    std::vector<double> values;
    values.reserve((end - start) / step);
    for (auto current_time = start; current_time < end; current_time += step) {
        values.push_back(this->at(divide_index, var_index));
        this->next();
    }

    return values;
}

} // namespace data_access
