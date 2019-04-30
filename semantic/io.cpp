/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/io.hpp"
#include "jsoncpp/as.hpp"

#include "io.hpp"

namespace fs = boost::filesystem;

namespace semantic {

namespace {

void parse(math::Size2f &size, const Json::Value &value)
{
    Json::unpack(value, "Size2f", size.width, size.height);
}

void parse(math::Point2 &point, const Json::Value &value)
{
    Json::unpack(value, "Point3", point(0), point(1));
}

void parse(math::Point3 &point, const Json::Value &value)
{
    Json::unpack(value, "Point3", point(0), point(1), point(2));
}

void parse(roof::Rectangular &r, const Json::Value &value)
{
    parse(r.size, Json::check(value, "size", Json::arrayValue));
    parse(r.skew, Json::check(value, "skew", Json::arrayValue));
    Json::get(r.azimuth, value, "azimuth");
    Json::get(r.curb, value, "curb");

    const auto &height(Json::check(value, "height", Json::objectValue
                                   , "height"));
    Json::get(r.ridgeHeight, height, "ridge");
    Json::get(r.curbHeight, height, "curb");
    Json::get(r.eaveHeight, height, "eave");
}

void parse(roof::Circular &r, const Json::Value &value)
{
    Json::get(r.radius, value, "radius");
    Json::get(r.curb, value, "curb");

    const auto &height(Json::check(value, "height", Json::objectValue
                                   , "height"));
    Json::get(r.ridgeHeight, height, "ridge");
    Json::get(r.curbHeight, height, "curb");
    Json::get(r.eaveHeight, height, "eave");
}

void parse(roof::Type &r, const Json::Value &value)
{
    struct Visitor : public boost::static_visitor<void> {
        const Json::Value &value;
        Visitor(const Json::Value &value) : value(value) {}

        void operator()(roof::Rectangular &r) {
            parse(r, value);
        }

        void operator()(roof::Circular &r) {
            parse(r, value);
        }
    } v(value);
    boost::apply_visitor(v, r);
}

void parse(Roof &r, const Json::Value &value)
{
    Json::get(r.type, value, "type");
    parse(r.center, Json::check(value, "center", Json::arrayValue));

    switch (r.type) {
    case Roof::Type::rectangular: r.instance = roof::Rectangular(); break;
    case Roof::Type::circular: r.instance = roof::Circular(); break;
    }

    parse(r.instance, value);
}

void parse(Roof::list &roofs, const Json::Value &value)
{
    roofs.reserve(value.size());
    for (const auto &item : value) {
        roofs.emplace_back();
        parse(roofs.back(), item);
    }
}

void parse(Building &building, const Json::Value &value)
{
    parse(building.origin, Json::check(value, "origin", Json::arrayValue));
    parse(building.roofs, Json::check(value, "roofs", Json::arrayValue));
}

void parse(Building::list &buildings, const Json::Value &value)
{
    buildings.reserve(value.size());
    for (const auto &item : value) {
        buildings.emplace_back();
        parse(buildings.back(), item);
    }
}

void parse(World &world, const Json::Value &value)
{
    {
        std::string srs;
        Json::get(srs, value, "srs");
        world.srs = geo::SrsDefinition::fromString(srs);
    }
    world.adjustVertical = false;
    Json::getOpt(world.adjustVertical, value, "adjustVertical");
    parse(world.origin, Json::check(value, "origin", Json::arrayValue));

    parse(world.buildings, Json::check(value, "buildings", Json::arrayValue));
}

void build(Json::Value &value, const World &world)
{
    // TODO: implement me
    (void) world;
    (void) value;
}

World load(std::istream &is, const fs::path &path)
{
    const auto value(Json::read(is, path));
    World world;
    parse(world, value);
    return world;
}

void save(const World &world, std::ostream &os, const fs::path &path)
{
    Json::Value value;
    build(value, world);
    Json::write(os, value, true); // TODO: make configurable
    (void) path;
}

} // namespace

World load(const fs::path &path)
{
    LOG(info1) << "Loading world from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string());
    const auto world(load(f, path));
    f.close();
    return world;
}

void save(const World &world, const fs::path &path)
{
    LOG(info1) << "Saving world to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), (std::ios_base::out | std::ios_base::trunc));
    save(world, f, path);
    f.close();
}

} // namespace semantic
