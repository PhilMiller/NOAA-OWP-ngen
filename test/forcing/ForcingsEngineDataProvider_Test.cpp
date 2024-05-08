#include <gtest/gtest.h>

#include "ForcingsEngineDataProvider.hpp"
#include "NGenConfig.h"
#if NGEN_WITH_MPI
#include <mpi.h>
#endif

#include "AorcForcing.hpp"
#include "DataProvider.hpp"
#include "ForcingsEngineLumpedDataProvider.hpp"
#include "utilities/python/InterpreterUtil.hpp"

#include <boost/range/combine.hpp>

struct mpi_info {
    int initialized = 0;
    int finalized   = 0;
    int size        = 1;
    int rank        = 0;
};

class ForcingsEngineDataProviderTest : public testing::Test
{
  protected:
    // Compile-time data
    static constexpr const char* config_file = "extern/ngen-forcing/NextGen_Forcings_Engine_BMI/NextGen_Forcings_Engine/config.yml";
    static std::shared_ptr<utils::ngenPy::InterpreterUtil> gil_;
    static data_access::ForcingsEngineLumpedDataProvider*   provider;
    static mpi_info                                        mpi;
    forcing_params                                         params;

  public:
    // Members
    ForcingsEngineDataProviderTest()
      : params("", "ForcingsEngine", "2024-01-17 01:00:00", "2024-01-17 06:00:00")
    {
        #if NGEN_WITH_MPI
        MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);
        MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
        #endif
    };

    static void SetUpTestSuite()
    {
        #if NGEN_WITH_MPI
        MPI_Init(nullptr, nullptr);
        #endif

        ForcingsEngineDataProviderTest::gil_ = utils::ngenPy::InterpreterUtil::getInstance();
        ForcingsEngineDataProviderTest::provider = data_access::ForcingsEngineLumpedDataProvider{
            config_file,
            "2024-01-17 01:00:00",
            "2024-01-17 06:00:00"
        };

        data_access::assert_forcings_engine_requirements();
    }

    static void TearDownTestSuite()
    {
        data_access::ForcingsEngineLumpedDataProvider::finalize_all();
        gil_.reset();

        #if NGEN_WITH_MPI
        PMPI_Finalize();
        #endif
    }    
};

std::shared_ptr<utils::ngenPy::InterpreterUtil> ForcingsEngineDataProviderTest::gil_{};
data_access::ForcingsEngineLumpedDataProvider* ForcingsEngineDataProviderTest::provider{};
mpi_info ForcingsEngineDataProviderTest::mpi{};

TEST_F(ForcingsEngineDataProviderTest, Engine)
{
    std::cout << "Getting instance\n";
    data_access::ForcingsEngineLumpedDataProvider inst{config_file, 0, 3600};
    std::cout << "Got instance\n";

    EXPECT_EQ(inst.variable_index("U2D_ELEMENT"), 0);
    EXPECT_EQ(inst.variable_index("Q2D_ELEMENT"), 5);
    EXPECT_EQ(inst.divide_index("cat-1015786"), 13868);
    EXPECT_EQ(inst.record_duration(), 3600);
}

// Tests that the forcings engine data provider correctly indexes epochs
// to unitless indices along its given temporal domain.
TEST_F(ForcingsEngineDataProviderTest, Timestepping)
{
    EXPECT_EQ(this->provider.get_ts_index_for_time(this->params.simulation_start_t), 0);
    EXPECT_EQ(this->provider.get_ts_index_for_time(this->params.simulation_end_t), 5);
}

// Tests that the forcings engine correctly initializes and performs
// a call to `get_values`.
TEST_F(ForcingsEngineDataProviderTest, HydrofabricGridType)
{
    const auto outputs_test = provider.get_available_variable_names();
    constexpr std::array<const char*, 8> expected_variables = {
        "U2D_ELEMENT",
        "V2D_ELEMENT",
        "LWDOWN_ELEMENT",
        "SWDOWN_ELEMENT",
        "T2D_ELEMENT",
        "Q2D_ELEMENT",
        "PSFC_ELEMENT",
        "RAINRATE_ELEMENT"
    };
    
    for (decltype(auto) output : boost::combine(outputs_test, expected_variables)) {
        EXPECT_EQ(output.get<0>(), output.get<1>());
    }

    const long duration = provider.get_ts_index_for_time(params.simulation_start_t) + (3600 * 2);
    const auto selector = CatchmentAggrDataSelector{
        "cat-1015786",
        "RAINRATE",
        provider.get_data_start_time(), // start time
        duration, // duration
        "seconds"
    };

    const auto result = provider.get_values(
        selector,
        data_access::ReSampleMethod::SUM
    );

    for (const auto& r : result) {
        EXPECT_NEAR(r, 5.51193e-07, 1e6);
    }
}

TEST_F(ForcingsEngineDataProviderTest, Lookback)
{
    // Both selectors encompass the entire temporal domain,
    // and since they're AOIs, they should have different values.
    // This means, to test lookback, we can evaluate one selector,
    // then attempt to use the second selector. If the values match
    // then lookback did not work correctly. Otherwise, lookback worked.
    //
    // Note that this assumes the variable values are different for the SAME variable.

    const auto first_selector = CatchmentAggrDataSelector{
        "cat-1015786",
        "RAINRATE",
        this->provider.get_data_start_time(),
        static_cast<long>(this->provider.get_ts_index_for_time(this->params.simulation_start_t) + (3600 * 2)),
        "seconds"
    };

    const auto second_selector = CatchmentAggrDataSelector{
        "cat-1015786",
        "U2D",
        this->provider.get_data_start_time(),
        static_cast<long>(this->provider.get_ts_index_for_time(this->params.simulation_start_t) + (3600 * 2)),
        "seconds"
    };

    const auto first_result = this->provider.get_value(first_selector, data_access::ReSampleMethod::SUM);
    const auto second_result = this->provider.get_value(second_selector, data_access::ReSampleMethod::SUM);

    if (std::fabs(first_result - second_result) < 1e-6) {
        FAIL() << std::to_string(first_result) << " == " << std::to_string(second_result) << " within 1e-6 epsilon.";
    }

    SUCCEED();
}

// Disabled -- revisit when BMI parallel proposal is closer to finalizing.
TEST_F(ForcingsEngineDataProviderTest, DISABLED_MPICommunicators) {
#if !NGEN_WITH_MPI
    GTEST_SKIP() << "Test is not MPI-enabled, or only has 1 process";
#else
    if (mpi.rank == 0) {
        // provider.set_communicator(MPI_Comm_c2f(MPI_COMM_SELF));
    }
    // ------------------------------------------------------------------------
    // Further calls to the provider should only use rank 0.

    // ------------------------------------------------------------------------
    // If this test gets called before other tests, let's reset back to MPI_COMM_WORLD
    if (mpi.rank == 0) {
        // provider.set_communicator(MPI_Comm_c2f(MPI_COMM_WORLD));
    }
#endif
}
