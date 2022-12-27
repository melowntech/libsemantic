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

#ifndef semantic_io_hpp_included_
#define semantic_io_hpp_included_

#include <sstream>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include <boost/filesystem/path.hpp>

#include "world.hpp"

namespace semantic {

UTILITY_GENERATE_ENUM(Format,
                      ((json))
                      ((binary))
                      )

struct SaveOptions {
    bool compress = false;
    Format format = Format::json;
};

World load(const boost::filesystem::path &path);

void save(const World &world, const boost::filesystem::path &path
          , const SaveOptions &options = {});

/** Entity serialization.
 */

struct SerializationOptions : SaveOptions {
    math::Point3 shift;

    SerializationOptions() = default;
    SerializationOptions(const SaveOptions &so
                         , const math::Point3 &shift = {})
        : SaveOptions(so), shift(shift)
    {}
};

#define SEMANTIC_DECLARE_ENTITY_IO_PAIR(ENTITY)                 \
    void serialize(std::ostream &os, const ENTITY &entity       \
                   , const SerializationOptions &options = {}); \
    void deserialize(std::istream &is, ENTITY &entity)

SEMANTIC_DECLARE_ENTITY_IO_PAIR(Building);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(Tree);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(Railway);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(LaneLine);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(Pole);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(Lamp);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(Manhole);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(TrafficSign);
SEMANTIC_DECLARE_ENTITY_IO_PAIR(TrafficLight);

#undef SEMANTIC_DECLARE_ENTITY_IO_PAIR

template <typename T>
std::string serialize(const T &entity
                      , const SerializationOptions &options = {});

template <typename T>
void deserialize(const T &entity, const void *data, std::size_t size);

// inlines

template <typename T>
std::string serialize(const T &entity, const SerializationOptions &options)
{
    std::ostringstream os;
    serialize(os, entity, options);
    return os.str();
}

template <typename T>
void deserialize(const void *data, std::size_t size, T &entity)
{
    namespace bio = boost::iostreams;
    const auto *cdata(static_cast<const char*>(data));
    bio::stream_buffer<bio::array_source> buffer(cdata, cdata + size);
    std::istream is(&buffer);
    is.exceptions(std::ios::badbit | std::ios::failbit);
    deserialize(is, entity);
}

} // namespace semantic

#endif // semantic_io_hpp_included_
