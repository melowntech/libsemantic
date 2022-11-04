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

// tree variant support
template <typename T>
T& treeInstance(semantic::tree::Instance &i)
{
    if (auto *value = boost::get<T>(&i)) {
        return *value;
    }

    i = T();
    return boost::get<T>(i);
}

template <typename T>
T& treeInstanceFromTree(semantic::Tree &r)
{
    return treeInstance<T>(r.instance);
}

// roof variant support
template <typename T>
T& roofInstance(semantic::roof::Instance &i)
{
    if (auto *value = boost::get<T>(&i)) {
        return *value;
    }

    i = T();
    return boost::get<T>(i);
}

template <typename T>
T& roofInstanceFromRoof(semantic::roof::Roof &r)
{
    return roofInstance<T>(r.instance);
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

geometry::Mesh meshLamp2(const Lamp &lamp, const MeshConfig &config
                         , const math::Point3 &origin)
{
    return semantic::lod2::mesh(lamp, config, origin);
}

geometry::Mesh meshManhole2(const Manhole &manhole, const MeshConfig &config
                         , const math::Point3 &origin)
{
    return semantic::lod2::mesh(manhole, config, origin);
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
        .def_readwrite("laneLines", &semantic::World::laneLines)
        .def_readwrite("lamps", &semantic::World::lamps)
        .def_readwrite("manholes", &semantic::World::manholes)
        .def_readwrite("trafficSigns", &semantic::World::trafficSigns)
        ;

    pysupport::fillEnum<semantic::Class>
        ("Class", "Semantic classes.");

    auto Tree = class_<semantic::Tree>("Tree", init<const semantic::Tree&>())
        .def(init<>())

        .add_property("aerial"
                      , make_function
                      (&py::treeInstanceFromTree<semantic::tree::Aerial>
                       , InternalRef))
        .add_property("groundLevel"
                      , bp::make_function
                      (&py::treeInstanceFromTree<semantic::tree::GroundLevel>
                       , InternalRef))

        .add_property("kind", bp::make_function(&semantic::Tree::kind
                                                , ByValue))
        ;
    py::addCommon(Tree);

    {
        bp::scope scope(Tree);

        auto Aerial = class_<semantic::tree::Aerial>
            ("Aerial", init<const semantic::tree::Aerial&>())
            .def(init<>())

            .def_readwrite("type", &semantic::tree::Aerial::type)
            .def_readwrite("a", &semantic::tree::Aerial::a)
            .def_readwrite("b", &semantic::tree::Aerial::b)
            .def_readwrite("harmonics", &semantic::tree::Aerial::harmonics)
            ;

        pysupport::fillEnum<semantic::tree::Aerial::Type>
            ("Type", "Tree type.");

        auto GroundLevel = class_<semantic::tree::GroundLevel>
            ("GroundLevel", init<const semantic::tree::GroundLevel&>())
            .def(init<>())

            .def_readwrite("trunk", &semantic::tree::GroundLevel::trunk)
            .def_readwrite("crown", &semantic::tree::GroundLevel::crown)
            .def_readwrite("height", &semantic::tree::GroundLevel::height)
            ;

        {
            bp::scope scope(GroundLevel);
            auto Circle = class_<semantic::tree::GroundLevel::Circle>
                ("Circle", init<semantic::tree::GroundLevel::Circle&>())
                .def(init<>())
                .def_readwrite
                ("center", &semantic::tree::GroundLevel::Circle::center)
                .def_readwrite
                ("radius", &semantic::tree::GroundLevel::Circle::radius)
                ;
        }

        auto Instance = class_<semantic::tree::Instance>
            ("Instance", init<const semantic::tree::Instance&>())
            .def(init<>())

            .add_property("aerial"
                          , bp::make_function
                          (&py::treeInstance<semantic::tree::Aerial>
                           , bp::return_internal_reference<>()))
            .add_property("groundLevel"
                          , bp::make_function
                          (&py::treeInstance<semantic::tree::GroundLevel>
                           , bp::return_internal_reference<>()))
            ;

        pysupport::vector<semantic::Tree::list>("list");

        pysupport::fillEnum<semantic::tree::Kind>
            ("Kind", "Tree kind.");
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
                      (&py::roofInstanceFromRoof<semantic::roof::Circular>
                       , InternalRef))
        .add_property("rectangular"
                      , bp::make_function
                      (&py::roofInstanceFromRoof<semantic::roof::Rectangular>
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
                          (&py::roofInstance<semantic::roof::Circular>
                           , bp::return_internal_reference<>()))
            .add_property("rectangular"
                          , bp::make_function
                          (&py::roofInstance<semantic::roof::Rectangular>
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

    auto LaneLine = class_<semantic::LaneLine>
        ("LaneLine", init<const semantic::LaneLine&>())
        .def(init<>())

        .def_readwrite("vertices", &semantic::LaneLine::vertices)
        .def_readwrite("lines", &semantic::LaneLine::lines)
        ;
    py::addCommon(LaneLine);

    {
        bp::scope scope(LaneLine);

        pysupport::vector<semantic::LaneLine::list>("list");
        pysupport::vector<semantic::LaneLine::Lines>("Lines");

        auto Line = class_<semantic::LaneLine::Line>
            ("Line", init<const semantic::LaneLine::Line&>())
            .def(init<>())

            .def_readwrite("id", &semantic::LaneLine::Line::id)
            .def_readwrite("polyline", &semantic::LaneLine::Line::polyline)
            .def_readwrite("isDashed", &semantic::LaneLine::Line::isDashed)
            .def_readwrite("isDouble", &semantic::LaneLine::Line::isDouble)
            ;
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

    auto Lamp = class_<semantic::Lamp>("Lamp", init<const semantic::Lamp&>())
        .def(init<>())

        .def_readwrite("mount", &semantic::Lamp::mount)
        .def_readwrite("dimensions", &semantic::Lamp::dimensions)
        ;
    py::addCommon(Lamp);

    {
        bp::scope scope(Lamp);

        pysupport::vector<semantic::Lamp::list>("list");
    }

    auto Manhole = class_<semantic::Manhole>("Manhole", init<const semantic::Manhole&>())
        .def(init<>())

        .def_readwrite("shape", &semantic::Manhole::shape)
        .def_readwrite("angle", &semantic::Manhole::angle)
        .def_readwrite("size", &semantic::Manhole::size)
        ;
    py::addCommon(Manhole);

    {
        bp::scope scope(Manhole);

        pysupport::vector<semantic::Manhole::list>("list");
    }

    auto TrafficSign = class_<semantic::TrafficSign>("TrafficSign",
                              init<const semantic::TrafficSign&>())
        .def(init<>())

        .def_readwrite("normal", &semantic::TrafficSign::normal)
        .def_readwrite("size", &semantic::TrafficSign::size)
        .def_readwrite("className", &semantic::TrafficSign::className)
        ;
    py::addCommon(TrafficSign);

    {
        bp::scope scope(TrafficSign);

        pysupport::vector<semantic::TrafficSign::list>("list");
        pysupport::vector<semantic::TrafficSign::Views>("Views");

        auto View = class_<semantic::TrafficSign::View>
            ("View", init<const semantic::TrafficSign::View&>())
            .def(init<>())

            .def_readwrite("path", &semantic::TrafficSign::View::path)
            .def_readwrite("boundingBox",
                          &semantic::TrafficSign::View::boundingBox)
            ;
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
        def("mesh", &py::meshLamp2);
        def("mesh", &py::meshManhole2);
        def("mesh", &py::meshWorld2);
    }
}

namespace semantic { namespace py {
PYSUPPORT_MODULE_IMPORT(semantic)
} } // namespace semantic::py
