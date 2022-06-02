/**
 * Copyright (c) 2018 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ogr_core.h>

#include <sstream>
#include <string>
#include <vector>
#include <mutex>

#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/handle.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <stdint.h>

#include "dbglog/dbglog.hpp"

#undef PYSUPPORT_MODULE_IMPORT_API
#define PYSUPPORT_MODULE_IMPORT_API 2
#include "pysupport/package.hpp"
#undef PYSUPPORT_MODULE_IMPORT_API

#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"
#include "pysupport/converters.hpp"
#include "pysupport/array.hpp"
#include "pysupport/vector.hpp"
#include "pysupport/variant.hpp"

#include "../world.hpp"
#include "../io.hpp"
#include "../mesh.hpp"
#include "semanticmodule.hpp"

namespace fs = boost::filesystem;
namespace bp = boost::python;
namespace bpc = boost::python::converter;

namespace semantic { namespace py {

template <typename T>
T& instance(semantic::roof::Instance &i)
{
    if (auto *value = boost::get<T>(&i)) {
        return *value;
    }

    i = T();
    return boost::get<T>(i);
}

template <typename T>
T& roofInstance(semantic::roof::Roof &r)
{
    return instance<T>(r.instance);
}

World load(const boost::filesystem::path &path)
{
    return semantic::load(path);
}

void save2(const World &world, const boost::filesystem::path &path)
{
    return semantic::save(world, path);
}

void save3(const World &world, const boost::filesystem::path &path
          , const SaveOptions &options)
{
    return semantic::save(world, path, options);
}

// "namespace"
struct lod2 {};

geometry::Mesh meshRoof2(const roof::Roof &roof, const MeshConfig &config
                         , const math::Point3 &origin)
{
    return semantic::lod2::mesh(roof, config, origin);
}

geometry::Mesh meshCircularRoof2(const roof::Circular &roof
                                 , const MeshConfig &config
                                 , const math::Point3 &origin)
{
    return semantic::lod2::mesh(roof, config, origin);
}

geometry::Mesh meshRectangularRoof2(const roof::Rectangular &roof
                                    , const MeshConfig &config
                                    , const math::Point3 &origin)
{
    return semantic::lod2::mesh(roof, config, origin);
}

geometry::Mesh meshBuilding2(const Building &building, const MeshConfig &config
                             , const math::Point3 &origin)
{
    return semantic::lod2::mesh(building, config, origin);
}

geometry::Mesh meshTree2(const Tree &tree, const MeshConfig &config
                         , const math::Point3 &origin)
{
    return semantic::lod2::mesh(tree, config, origin);
}

geometry::Mesh meshPole2(const Pole &pole, const MeshConfig &config
                         , const math::Point3 &origin)
{
    return semantic::lod2::mesh(pole, config, origin);
}

geometry::Mesh meshWorld2(const World &world, const MeshConfig &config)
{
    return semantic::mesh(world, config, 2);
}

template <typename Class>
std::string classRepr(const Class &c)
{
    std::ostringstream os;

    auto s(semantic::serialize(c));
    if (!s.empty() && (s.back() == '\n')) { s.pop_back(); }

    os << '<' << Class::cls << ": " << s << '>';

    return os.str();
}

template <typename Class>
void addCommon(bp::class_<Class> &cls)
{
    cls
        .def_readonly("cls", &Class::cls)
        .def_readwrite("id", &Class::id)
        .def_readwrite("descriptor", &Class::descriptor)
        .def_readwrite("origin", &Class::origin)
        .def("__repr__", &classRepr<Class>
             , "Returns class representation")
        ;
}

} } // namespace semantic::py

BOOST_PYTHON_MODULE(melown_semantic)
{
    using namespace bp;
    namespace py = semantic::py;

    const return_internal_reference<> InternalRef;
    const return_value_policy<return_by_value> ByValue;

    // world/entities

    auto World = class_<semantic::World>
        ("World", init<const semantic::World&>())
        .def(init<>())

        .def_readwrite("srs", &semantic::World::srs)
        .def_readwrite("origin", &semantic::World::origin)
        .def_readwrite("buildings", &semantic::World::buildings)
        .def_readwrite("trees", &semantic::World::trees)
        .def_readwrite("poles", &semantic::World::poles)
        .def_readwrite("railways", &semantic::World::railways)
        ;

    pysupport::fillEnum<semantic::Class>
        ("Class", "Semantic classes.");

    auto Tree = class_<semantic::Tree>("Tree", init<const semantic::Tree&>())
        .def(init<>())

        .def_readwrite("type", &semantic::Tree::type)
        .def_readwrite("a", &semantic::Tree::a)
        .def_readwrite("b", &semantic::Tree::b)
        .def_readwrite("harmonics", &semantic::Tree::harmonics)
        ;
    py::addCommon(Tree);

    {
        bp::scope scope(Tree);

        pysupport::fillEnum<semantic::Tree::Type>
            ("Type", "Tree type.");

        pysupport::vector<semantic::Tree::list>("list");
    }

    auto Building = class_<semantic::Building>
        ("Building", init<const semantic::Building&>())
        .def(init<>())

        .def_readwrite("roofs", &semantic::Building::roofs)
        ;
    py::addCommon(Building);

    {
        bp::scope scope(Building);

        pysupport::vector<semantic::Building::list>("list");
    }

    auto Roof = class_<semantic::roof::Roof>
        ("Roof", init<const semantic::roof::Roof&>())
        .def(init<>())

        .def_readwrite("center", &semantic::roof::Roof::center)

        .add_property("circular"
                      , make_function
                      (&py::roofInstance<semantic::roof::Circular>
                       , InternalRef))
        .add_property("rectangular"
                      , bp::make_function
                      (&py::roofInstance<semantic::roof::Rectangular>
                       , InternalRef))

        .add_property("type", bp::make_function(&semantic::roof::Roof::type
                                                , ByValue))
        ;

    {
        bp::scope scope(Roof);

        pysupport::fillEnum<semantic::roof::Type>
            ("Type", "Roof type.");

        pysupport::vector<semantic::roof::Roof::list>("list");

        auto Circular = class_<semantic::roof::Circular>
            ("Circular", init<const semantic::roof::Circular&>())
            .def(init<>())

            .def_readwrite("radius", &semantic::roof::Circular::radius)
            .def_readwrite("curb", &semantic::roof::Circular::curb)
            .def_readwrite("ridgeHeight"
                           , &semantic::roof::Circular::ridgeHeight)
            .def_readwrite("curbHeight", &semantic::roof::Circular::curbHeight)
            .def_readwrite("eaveHeight", &semantic::roof::Circular::eaveHeight)
            ;

        auto Rectangular = class_<semantic::roof::Rectangular>
            ("Rectangular", init<const semantic::roof::Rectangular&>())
            .def(init<>())

            .def_readwrite("size", &semantic::roof::Rectangular::size)
            .def_readwrite("skew", &semantic::roof::Rectangular::skew)
            .def_readwrite("azimuth", &semantic::roof::Rectangular::azimuth)
            .def_readwrite("curb", &semantic::roof::Rectangular::curb)
            .def_readwrite("hip", &semantic::roof::Rectangular::hip)
            .def_readwrite("ridgeHeight"
                           , &semantic::roof::Rectangular::ridgeHeight)
            .def_readwrite("curbHeight"
                           , &semantic::roof::Rectangular::curbHeight)
            .def_readwrite("eaveHeight"
                           , &semantic::roof::Rectangular::eaveHeight)
            ;

        {
            bp::scope scope(Rectangular);

            pysupport::fillEnum<semantic::roof::Rectangular::Key>
                ("Key", "Key in rectangular roof data.");
        }

        auto Instance = class_<semantic::roof::Instance>
            ("Instance", init<const semantic::roof::Instance&>())
            .def(init<>())

            .add_property("circular"
                          , bp::make_function
                          (&py::instance<semantic::roof::Circular>
                           , bp::return_internal_reference<>()))
            .add_property("rectangular"
                          , bp::make_function
                          (&py::instance<semantic::roof::Rectangular>
                           , bp::return_internal_reference<>()))
            ;
    }

    auto Railway = class_<semantic::Railway>
        ("Railway", init<const semantic::Railway&>())
        .def(init<>())

        .def_readwrite("vertices", &semantic::Railway::vertices)
        .def_readwrite("lines", &semantic::Railway::lines)
        ;
    py::addCommon(Railway);

    {
        bp::scope scope(Railway);

        pysupport::vector<semantic::Railway::list>("list");
        pysupport::vector<semantic::Railway::Line
                          , return_value_policy<return_by_value>
                          >("Line");
        pysupport::vector<semantic::Railway::Lines>("Lines");
    }

    auto Pole = class_<semantic::Pole>("Pole", init<const semantic::Pole&>())
        .def(init<>())

        .def_readwrite("direction", &semantic::Pole::direction)
        .def_readwrite("length", &semantic::Pole::length)
        .def_readwrite("distanceToGround", &semantic::Pole::distanceToGround)
        .def_readwrite("radius", &semantic::Pole::radius)
        ;
    py::addCommon(Pole);

    {
        bp::scope scope(Pole);

        pysupport::vector<semantic::Pole::list>("list");
    }

    // IO

    pysupport::fillEnum<semantic::Format>
        ("Format", "File format.");

    auto SaveOptions = class_<semantic::SaveOptions>
        ("SaveOptions", init<const semantic::SaveOptions&>())
        .def(init<>())

        .def_readwrite("compress", &semantic::SaveOptions::compress)
        .def_readwrite("format", &semantic::SaveOptions::format)
        ;

    def("load", py::load);
    def("save", py::save2);
    def("save", py::save3);

    // TODO: serialization


    // meshing

    auto MeshConfig = class_<semantic::MeshConfig>
        ("MeshConfig", init<const semantic::MeshConfig&>())
        .def(init<>())

        .def_readwrite("maxCircleSegment"
                       , &semantic::MeshConfig::maxCircleSegment)
        .def_readwrite("minSegmentCount"
                       , &semantic::MeshConfig::minSegmentCount)
        .def_readwrite("closedSurface"
                       , &semantic::MeshConfig::closedSurface)
        .def_readwrite("worldCrs"
                       , &semantic::MeshConfig::worldCrs)
        ;

    pysupport::fillEnum<semantic::Material>
        ("Material", "Mesh material.");

    def("materials", &semantic::materials);

    auto lod2 = class_<py::lod2>("lod2", init<>());

    {
        bp::scope scope(lod2);

        def("mesh", &py::meshRoof2);
        def("mesh", &py::meshCircularRoof2);
        def("mesh", &py::meshRectangularRoof2);
        def("mesh", &py::meshBuilding2);
        def("mesh", &py::meshTree2);
        def("mesh", &py::meshPole2);
        def("mesh", &py::meshWorld2);
    }
}

namespace semantic { namespace py {
PYSUPPORT_MODULE_IMPORT(semantic)
} } // namespace semantic::py
