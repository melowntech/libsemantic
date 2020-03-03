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

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/io.hpp"
#include "jsoncpp/as.hpp"

#include "io.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;

namespace semantic {

namespace {

/* ------------------------------------------------------------------------ */
/* ---- Parsing ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void parse(math::Size2f &size, const Json::Value &value)
{
    Json::unpack(value, "Size2f", size.width, size.height);
}

void parse(math::Point2 &point, const Json::Value &value)
{
    Json::unpack(value, "Point2", point(0), point(1));
}

void parse(math::Point3 &point, const Json::Value &value)
{
    Json::unpack(value, "Point3", point(0), point(1), point(2));
}

void parse(math::Points3 &points, const Json::Value &value)
{
    points.resize(value.size());
    auto ipoints(points.begin());
    for (const auto &item : value) {
        parse(*ipoints++, item);
    }
}

void parse(roof::Rectangular &r, const Json::Value &value)
{
    parse(r.size, Json::check(value, "size", Json::arrayValue));
    parse(r.skew, Json::check(value, "skew", Json::arrayValue));
    Json::get(r.azimuth, value, "azimuth");
    Json::get(r.curb, value, "curb");
    if (!Json::getOpt(r.hip, value, "hip")) {
        r.hip = roof::Rectangular::defaultHip;
    }

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

void parse(roof::Instance &r, const Json::Value &value)
{
    struct Visitor : public boost::static_visitor<void> {
        const Json::Value &value;
        Visitor(const Json::Value &value) : value(value) {}
        void operator()(roof::Rectangular &r) const { parse(r, value); }
        void operator()(roof::Circular &r) const { parse(r, value); }
    } v(value);
    boost::apply_visitor(v, r);
}

void parse(roof::Roof &r, const Json::Value &value)
{
    roof::Type type;
    Json::get(type, value, "type");
    parse(r.center, Json::check(value, "center", Json::arrayValue));

    switch (type) {
    case roof::Type::rectangular:
        r.instance = roof::Rectangular(); break;
    case roof::Type::circular:
        r.instance = roof::Circular(); break;
    }

    parse(r.instance, value);
}

void parse(roof::Roof::list &roofs, const Json::Value &value)
{
    roofs.reserve(value.size());
    for (const auto &item : value) {
        roofs.emplace_back();
        parse(roofs.back(), item);
    }
}

void parse(Entity &entity, const Json::Value &value)
{
    Json::get(entity.id, value, "id");
    Json::getOpt(entity.descriptor, value, "descriptor");
    parse(entity.origin, Json::check(value, "origin", Json::arrayValue));
}

void parse(Building &building, const Json::Value &value)
{
    parse(static_cast<Entity&>(building), value);
    parse(building.roofs, Json::check(value, "roofs", Json::arrayValue));
}

void parse(Tree &tree, const Json::Value &value)
{
    parse(static_cast<Entity&>(tree), value);
    parse(tree.center, Json::check(value, "center", Json::arrayValue));
    Json::get(tree.a, value, "a");
    if (!Json::getOpt(tree.b, value, "b")) { tree.b = tree.a; }
    Json::get(tree.harmonics, value, "harmonics");
    if (!Json::getOpt(tree.type, value, "type")) {
        tree.type = Tree::Type::deciduous;
    }
}

void parse(Railway::Lines &lines, const Json::Value &value)
{
    lines.reserve(value.size());
    for (const auto &item : value) {
        lines.emplace_back();
        Json::get(lines.back(), item, "line");
    }
}

void parse(Railway &railway, const Json::Value &value)
{
    parse(static_cast<Entity&>(railway), value);
    parse(railway.vertices, Json::check(value, "vertices", Json::arrayValue));
    parse(railway.lines, Json::check(value, "lines", Json::arrayValue));
}

template <typename EntityType>
void parse(std::vector<EntityType> &entities, const Json::Value &container
           , const char *name)
{
    if (!container.isMember(name)) { return; }
    const auto &value(check(container[name], Json::arrayValue, name));

    entities.reserve(value.size());
    for (const auto &item : value) {
        entities.emplace_back();
        parse(entities.back(), item);
    }
}

void parse(World &world, const Json::Value &value)
{
    {
        std::string srs;
        Json::get(srs, value, "srs");
        world.srs = geo::SrsDefinition::fromString(srs);
    }
    parse(world.origin, Json::check(value, "origin", Json::arrayValue));

    parse(world.buildings, value, "buildings");
    parse(world.trees, value, "trees");
    parse(world.railways, value, "railways");
}

/* ------------------------------------------------------------------------ */
/* ---- Building ---------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void build(Json::Value &value, const math::Size2f &size)
{
    value = Json::arrayValue;
    value.append(size.width);
    value.append(size.height);
}

void build(Json::Value &value, const math::Point2 &point)
{
    value = Json::arrayValue;
    value.append(point(0));
    value.append(point(1));
}

void build(Json::Value &value, const math::Point3 &point)
{
    value = Json::arrayValue;
    value.append(point(0));
    value.append(point(1));
    value.append(point(2));
}

void build(Json::Value &value, const math::Points3 &points)
{
    for (const auto &point : points) {
        build(value.append(Json::arrayValue), point);
    }
}

template <std::size_t N>
void build(Json::Value &value, const std::array<double, N> &array)
{
    value = Json::arrayValue;
    for (const auto &item : array) { value.append(item); }
}

void build(Json::Value &value, const roof::Rectangular &r)
{
    value = Json::objectValue;
    build(value["size"], r.size);
    build(value["skew"], r.skew);
    value["azimuth"] = r.azimuth;
    build(value["curb"], r.curb);
    if (r.hip != roof::Rectangular::defaultHip) {
        build(value["hip"], r.hip);
    }

    auto &height(value["height"] = Json::objectValue);
    height["ridge"] = r.ridgeHeight;
    height["curb"] = r.curbHeight;
    build(height["eave"], r.eaveHeight);
}

void build(Json::Value &value, const roof::Circular &r)
{
    value = Json::objectValue;
    value["radius"] = r.radius;
    value["curb"] = r.curb;

    auto &height(value["height"] = Json::objectValue);
    height["ridge"] = r.ridgeHeight;
    height["curb"] = r.curbHeight;
    height["eave"] = r.eaveHeight;
}

void build(Json::Value &value, const roof::Roof &r)
{
    // build instance
    struct Visitor : public boost::static_visitor<void> {
        Json::Value &value;
        Visitor(Json::Value &value) : value(value) {}
        void operator()(const roof::Rectangular &r) { build(value, r); }
        void operator()(const roof::Circular &r) { build(value, r); }
    } v(value);
    boost::apply_visitor(v, r.instance);

    // add common stuff
    value["type"] = boost::lexical_cast<std::string>(r.type());
    build(value["center"], r.center);
}

void build(Json::Value &value, const roof::Roof::list &roofs)
{
    value = Json::arrayValue;
    for (const auto &roof : roofs) {
        auto &item(value.append(Json::objectValue));
        build(item, roof);
    }
}

void build(Json::Value &value, const Entity &entity
           , const math::Point3 &shift)
{
    value = Json::objectValue;
    value["id"] = entity.id;
    if (!entity.descriptor.empty()) {
        value["descriptor"] = entity.descriptor;
    }
    build(value["origin"], math::Point3(entity.origin + shift));
}

void build(Json::Value &value, const Building &building
           , const math::Point3 &shift)
{
    build(value, static_cast<const Entity&>(building), shift);
    build(value["roofs"], building.roofs);
}

void build(Json::Value &value, const Tree &tree
           , const math::Point3 &shift)
{
    build(value, static_cast<const Entity&>(tree), shift);
    build(value["center"], tree.center);
    value["a"] = tree.a;
    if (tree.a != tree.b) { value["b"] = tree.b; }
    auto &harmonics(value["harmonics"] = Json::arrayValue);
    for (auto harmonic : tree.harmonics) { harmonics.append(harmonic); }

    if (tree.type != Tree::Type::deciduous) {
        value["type"] = boost::lexical_cast<std::string>(tree.type);
    }

}

void build(Json::Value &value, const Railway::Lines &lines)
{
    value = Json::arrayValue;
    for (const auto &line : lines) {
        auto &item(value.append(Json::arrayValue));
        for (auto index : line) {
            item.append(index);
        }
    }
}

void build(Json::Value &value, const Railway &railway
           , const math::Point3 &shift)
{
    build(value, static_cast<const Entity&>(railway), shift);

    build(value, railway.vertices);
    build(value, railway.lines);
}

template <typename EntityType>
void build(Json::Value &container, const char *name
           , const std::vector<EntityType> &entities
           , const math::Point3 &shift = math::Point3())
{
    if (entities.empty()) { return; }
    auto &value(container[name] = Json::arrayValue);
    for (const auto &entity : entities) {
        auto &item(value.append(Json::objectValue));
        build(item, entity, shift);
    }
}

void build(Json::Value &value, const World &world)
{
    value = Json::objectValue;

    value["srs"] = world.srs.as(geo::SrsDefinition::Type::proj4).toString();

    build(value["origin"], world.origin);
    build(value, "buildings", world.buildings);
    build(value, "trees", world.trees);
    build(value, "railways", world.railways);
}

World load(std::istream &is, const fs::path &path)
{
    const auto value(Json::read(is, path));
    World world;
    parse(world, value);
    return world;
}

void save(const World &world, std::ostream &os
          , const SerializationOptions &options)
{
    switch (options.format) {
    case Format::json: {
        Json::Value value;
        build(value, world);
        Json::write(os, value, true); // TODO: make configurable
    } break;

    case Format::binary: {
        LOGTHROW(err2, std::runtime_error)
            << "Binary format not implemented yet.";
    } break;
    }
}

template <typename T>
void serializeEntityImpl(std::ostream &os, const T &entity
                         , const SerializationOptions &options)
{
    switch (options.format) {
    case Format::json: {
        Json::Value value;
        build(value, entity, options.shift);
        Json::write(os, value, false);
    } break;

    case Format::binary: {
        LOGTHROW(err2, std::runtime_error)
            << "Binary format not implemented yet.";
    } break;
    }
}

template <typename T>
void deserializeEntityImpl(std::istream &is, T &entity)
{
    // TODO: detect format, use JSON for now
    Json::Value value;
    Json::read(is, value);
    parse(entity, value);
}

template <typename T>
void serializeEntity(std::ostream &os, const T &entity
                     , const SerializationOptions &options)
{
    if (options.compress) {
        bio::filtering_ostream gz;
        gz.push(bio::gzip_compressor(bio::gzip_params(9), 1 << 16));
        gz.push(os);
        gz.exceptions(os.exceptions());
        serializeEntityImpl(gz, entity, options);
        gz.flush();
        return;
    }

    serializeEntityImpl(os, entity, options);
}

template <typename T>
void deserializeEntity(std::istream &is, T &entity)
{
    if (is.peek() == 0x1f) {
        // gzipped
        bio::filtering_istream gz;
        gz.push(bio::gzip_decompressor
                (bio::gzip_params().window_bits, 1 << 16));
        gz.push(is);
        gz.exceptions(is.exceptions());

        deserializeEntityImpl(gz, entity);
        return;
    }

    deserializeEntityImpl(is, entity);
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

void save(const World &world, const fs::path &path
          , const SaveOptions &options)
{
    LOG(info1) << "Saving world to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), (std::ios_base::out | std::ios_base::trunc));

    if (options.compress) {
        bio::filtering_ostream gz;
        gz.push(bio::gzip_compressor(bio::gzip_params(9), 1 << 16));
        gz.push(f);
        save(world, gz, options);
        gz.flush();
    } else {
        save(world, f, options);
    }

    f.close();
}

#define SEMANTIC_DEFINE_ENTITY_IO_PAIR(ENTITY)              \
    void serialize(std::ostream &os, const ENTITY &entity   \
                   , const SerializationOptions &options)   \
    {                                                       \
        serializeEntity(os, entity, options);               \
    }                                                       \
    void deserialize(std::istream &is, ENTITY &entity)      \
    {                                                       \
        deserializeEntity(is, entity);                      \
    }

SEMANTIC_DEFINE_ENTITY_IO_PAIR(Building)
SEMANTIC_DEFINE_ENTITY_IO_PAIR(Tree)
SEMANTIC_DEFINE_ENTITY_IO_PAIR(Railway)

} // namespace semantic
