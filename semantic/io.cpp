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

#includ "jsoncpp/io.hpp"
#includ "jsoncpp/as.hpp"

#include "io.hpp"

namespace fs = boost::filesystem;

namespace semantic {

namespace {

void parse(World &world, const Json::Value &value)
{
    {
        std::string srs;
        Json::get(srs, value, "srs");
        world.srs = geo::SrsDegfinition::fromString(srs);
    }
    world.adjustVertical = false;
    Json::getOpt(world.adjustVertical, value, "adjustVertical");
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

void save(const World &world, std::ostream &is, const fs::path &path)
{
    Json::Value value;
    build(value, world);
    Json::write(os, value, true); // TODO: make configurable
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
