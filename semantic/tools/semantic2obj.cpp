#include "service/cmdline.hpp"

#include <cstdlib>
#include <iostream>
#include <queue>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/implicit-value.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"
#include "geometry/mesh-polygonization.hpp"

#include "semantic/io.hpp"
#include "semantic/mesh.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Semantic2Obj : public service::Cmdline
{
public:
    Semantic2Obj()
        : service::Cmdline("semantic2obj", BUILD_TARGET_VERSION)
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

    semantic::MeshConfig meshConfig_;
    std::vector<fs::path> input_;
    fs::path output_;
    bool multipolygonal_;
};

void Semantic2Obj::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to the input file with semantic data.")
        ("output", po::value(&output_)->required()
         , "Path to the output OBJ file.")
        ("closedSurface"
         , utility::implicit_value(&meshConfig_.closedSurface, true)
         ->default_value(false)
         , "Generate closed surface.")
        ("worldCrs"
         , utility::implicit_value(&meshConfig_.worldCrs, true)
         ->default_value(false)
         , "Keep the mesh in world's CRS (i.e. do not shift by world origin).")
        ("repairMesh", utility::implicit_value(&meshConfig_.repairMesh, true)
         ->default_value(false)
         , "Remove zero-area and non-manifold faces from building roof meshes. "
           "Also sets `vertexMergeEps` to some non-zero value if it's zero (to "
           "merge very close vertices).")
        ("vertexMergeEps", po::value(&meshConfig_.vertexMergeEps)
         ->default_value(meshConfig_.vertexMergeEps)
         ,"Min distance between two distinct verticies (vertices are merged "
          "otherwise). Used in building roofs. Set to zero to disable merging.")
        ("multipolygonal", utility::implicit_value(&multipolygonal_, true)
         ->default_value(false)
         , "Save mesh in multipolygonal format.")
        ;

    pd.add("output", 1).add("input", -1);

    (void) config;
}

void Semantic2Obj::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Semantic2Obj::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(semantic2obj
usage
    semantic2obj output input* [OPTIONS]

If multiple input files are used their SRS must be the same.

)RAW";
    }
    return false;
}


/**
 * Find connected components of faces with the same angle.
 */
void classifyFaces(const geometry::Mesh& mesh,
                   std::vector<int>& regions,
                   std::vector<semantic::Material>& regionMaterials) {

    auto ffTable = geometry::getFaceFaceTableNonManifold(mesh);

    regions.resize(mesh.faces.size());
    std::fill(regions.begin(), regions.end(), -1);

    int comp { 0 };
    for (std::size_t seed = 0; seed < mesh.faces.size(); ++seed) {
        if (regions[seed] != -1) { continue; } // skip assigned
        regions[seed] = comp;                  // mark seed
        regionMaterials.push_back(
            static_cast<semantic::Material>(mesh.faces[seed].imageId));

        std::queue<int> q; // faces added to comp in last round
        q.push(seed);

        while (!q.empty()) {
            auto fI { q.front() };
            q.pop();

            // check neighbors
            for (const auto& nI : ffTable[fI]) {
                //if (nI < 0) { continue; }
                if (regions[nI] != -1) { continue; } // skip enqueued

                // different materials (colors)
                if (mesh.faces[fI].imageId != mesh.faces[nI].imageId) {
                    continue;
                }

                auto faceNormal { mesh.normal(mesh.faces[seed]) };
                auto neighborNormal { mesh.normal(mesh.faces[nI]) };
                auto angle { math::angle(faceNormal, neighborNormal) };

                if (!std::isfinite(angle)) {
                    LOGTHROW(err4, std::runtime_error)
                        << "Angle between faces not finite";
                }

                if (std::abs(angle) < 1e-8) { // should be OK for buildings <10km
                    // Add neighbor to component & enqueue it
                    regions[nI] = comp;
                    q.push(nI);
                }
            }
        }

        // no new faces to add
        ++comp;
    }
}


int Semantic2Obj::run()
{
    boost::optional<geo::SrsDefinition> srs;

    geometry::Mesh mesh;
    for (const auto &input : input_) {
        const auto world(semantic::load(input));

        if (!srs) {
            srs = world.srs;
        } else if (!areSame(*srs, world.srs)) {
            LOG(fatal)
                << "Different SRS in input world file "
                << input << ": '" << world.srs << "'.";
            return EXIT_FAILURE;
        }

        meshConfig_.repairMesh = true; // required for polygonization
        append(mesh, semantic::mesh(world, meshConfig_));
    }

    const auto &setStream([&, srsWritten=false](std::ostream &os)
        mutable
    {
        os << std::fixed;
        if (!srsWritten) {
            os << "### SRS: " << srs.value() << "\n\n";
            srsWritten = true;
        }
        return true;
    });

    const auto mtlPath(utility::addExtension(output_, ".mtl"));
    geometry::ObjMaterial mtl(mtlPath.filename().generic_string());
    mtl.names = semantic::materials();

    if (!multipolygonal_) {

        LOG(info3) << "Saving triangular mesh.";

        // write mtl
        semantic::writeMtl(mtlPath);
        geometry::saveAsObj(mesh, output_, mtl, setStream);

    } else {

        std::vector<int> regions;
        std::vector<semantic::Material> regionMaterials;
        classifyFaces(mesh, regions, regionMaterials);

        LOG(info3) << "Creating multipolygonal mesh from triangular mesh.";
        auto polyMesh { geometry::polygonizeMesh(mesh, regions) };

        // translate region idx to material idx
        for (auto& idx : polyMesh.faceLabels) {
            idx = +regionMaterials[idx];
        }

        LOG(info3) << "Saving multipolygonal mesh.";

        // write mtl
        semantic::writeMtl(mtlPath);
        geometry::saveAsObj(polyMesh, output_, mtl, setStream);
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Semantic2Obj()(argc, argv);
}
