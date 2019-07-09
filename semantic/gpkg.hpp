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

#ifndef semantic_gpkg_hpp_included_
#define semantic_gpkg_hpp_included_

#include <memory>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "geo/srsdef.hpp"

#include "world.hpp"

namespace semantic {

class GeoPackage {
public:
    /** Opens existing (read-only) dataset.
     */
    GeoPackage(const boost::filesystem::path &path);

    /** Creates new dataset.
     */
    GeoPackage(const boost::filesystem::path &path
               , const geo::SrsDefinition &srs);

    ~GeoPackage();

    void add(const World &world);

    /** Query descriptor.
     */
    struct Query {
        Query() {};

        /** Extents. Defaults to whole world. Extents are in srs.
         */
        boost::optional<math::Extents2> extents;

        /** Query SRS. Defaults to dataset SRS.
         */
        boost::optional<geo::SrsDefinition> srs;

        /** Used when query SRS is different from dataset SRS.
         */
        std::size_t segmentsPerExtentEdge = 4;
    };

    World world(const Query &query = {}) const;

    struct Detail;

private:
    std::unique_ptr<Detail> detail_;
};

} // namespace semantic

#endif // semantic_gpkg_hpp_included_
