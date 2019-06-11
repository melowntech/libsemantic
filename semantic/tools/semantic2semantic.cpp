#include "service/cmdline.hpp"

#include <cstdlib>
#include <iostream>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "service/cmdline.hpp"

#include "semantic/io.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Semantic2Semantic : public service::Cmdline
{
public:
    Semantic2Semantic()
        : service::Cmdline("semantic2semantic", BUILD_TARGET_VERSION)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        override;

    virtual void configure(const po::variables_map &vars)
        override;

    virtual bool help(std::ostream &out, const std::string &what) const
        override;

    virtual int run() override;

    fs::path input_;
    fs::path output_;
};

void Semantic2Semantic::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to the input file with semantic data.")
        ("output", po::value(&output_)->required()
         , "Path to the output OBJ file.")
        ;

    pd.add("input", 1).add("output", 1);

    (void) config;
}

void Semantic2Semantic::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Semantic2Semantic::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(semantic2semantic
usage
    semantic2semantic input output [OPTIONS]

)RAW";
    }
    return false;
}

int Semantic2Semantic::run()
{
    auto world(semantic::load(input_));
    semantic::save(world, output_);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Semantic2Semantic()(argc, argv);
}
