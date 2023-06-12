/**
 * Copyright (c) 2023 Melown Technologies SE
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

#include <algorithm>

#include "dbglog/dbglog.hpp"
#include "geometry/mesh.hpp"
#include "geometry/meshop.hpp"

#include "math/geometry.hpp"

/**
 * Remove null (zero-area) faces while preserving topology. Written specifically
 * for the case of lod2 roofs.
 */

namespace semantic
{
namespace lod2
{

namespace
{

using Mesh = geometry::Mesh;
using FFTable = geometry::FaceFaceTable;
using Face = geometry::Face;
using FIdx = std::size_t;
using VIdx = geometry::Face::index_type;

/// Min vertex distance from a line formed by two other vertices; should be
/// lower than vertexMergeEps
constexpr double NullFaceThresh { 1e-4 };

/// Max number of passes to remove null faces
constexpr std::size_t MaxEdgeflipIters { 100 };

/// Checks if face contains edge v1-v2 (in this order)
bool hasEdge(const Face& f, const VIdx v1, const VIdx v2)
{
    return (f.a == v1 && f.b == v2) || (f.b == v1 && f.c == v2)
           || (f.c == v1 && f.a == v2);
}

/// Get the other vertex of face. NB: vertices must be given in ccw order!
VIdx theOtherVertex(const Face& f, const VIdx v1, const VIdx v2)
{
    if (f.a == v1 && f.b == v2) { return f.c; }
    if (f.b == v1 && f.c == v2) { return f.a; }
    if (f.c == v1 && f.a == v2) { return f.b; }
    LOGTHROW(err4, std::runtime_error)
        << "Cannot find given vertices in the face";
    throw;
}

/// Check null face = vertices almost on one line
bool isNullFace(const Mesh& mesh, const Face& f)
{
    auto& a { mesh.vertices[f.a] };
    auto& b { mesh.vertices[f.b] };
    auto& c { mesh.vertices[f.c] };
    return math::pointLineDistance(c, math::Line3(a, b - a)) < NullFaceThresh;
}

/// Flip edge v1-v2 incident with face fI1
bool edgeFlip(Mesh& mesh,
              const FFTable& ffTable,
              const FIdx fI1,
              const VIdx v1,
              const VIdx v2)
{
    // find the opposite face
    FIdx fI2;
    bool found { false };
    for (const auto& n : ffTable[fI1]) {
        if (hasEdge(mesh.faces[n], v2, v1)) {
            fI2 = n;
            found = true;
            break;
        }
    }

    if (!found) {
        LOGTHROW(err4, std::runtime_error)
            << "Cannot find edge in any neighbouring faces.";
    }

    if (std::count(ffTable[fI1].begin(), ffTable[fI1].end(), fI2) > 1) {
        // the triangles share multiple edges (should not happen)
        return false;
    }

    // Verify that there are not any common neighbours between the two faces
    auto neigh1 { ffTable[fI1] };
    std::sort(neigh1.begin(), neigh1.end());
    auto neigh2 { ffTable[fI2] };
    std::sort(neigh2.begin(), neigh2.end());
    std::vector<std::size_t> commonNeighbours;
    std::set_intersection(neigh1.begin(),
                          neigh1.end(),
                          neigh2.begin(),
                          neigh2.end(),
                          std::back_inserter(commonNeighbours));
    if (commonNeighbours.size() > 0) { return false; }

    // Construct the new faces
    auto& f1 { mesh.faces[fI1] };
    auto& f2 { mesh.faces[fI2] };
    auto a { theOtherVertex(f1, v1, v2) };
    auto b { theOtherVertex(f2, v2, v1) };
    auto nf1 { Face(a, b, v2, f2.imageId) };
    auto nf2 { Face(b, a, v1, f2.imageId) };

    // Check they are not null
    if (isNullFace(mesh, nf1) || isNullFace(mesh, nf2)) { return false; }

    mesh.faces[fI1] = nf1;
    mesh.faces[fI2] = nf2;
    return true;
}

/// Flip the longest edge - works in our case
bool removeNullFaceByEdgeFlip(Mesh& mesh, const FFTable& ffTable, const FIdx fI)
{
    auto& f { mesh.faces[fI] };

    // flip the longest edge
    auto lab { math::length(mesh.vertices[f.a] - mesh.vertices[f.b]) };
    auto lbc { math::length(mesh.vertices[f.b] - mesh.vertices[f.c]) };
    auto lca { math::length(mesh.vertices[f.c] - mesh.vertices[f.a]) };

    if (lab > lbc && lab > lca) {
        return edgeFlip(mesh, ffTable, fI, f.a, f.b);
    } else {
        if (lbc > lca) { return edgeFlip(mesh, ffTable, fI, f.b, f.c); }
        else { return edgeFlip(mesh, ffTable, fI, f.c, f.a); }
    }
}


bool attemptToRemoveNullFacesByEdgeFlip(Mesh& mesh)
{
    std::size_t flipped { 0 };
    auto ffTable { geometry::getFaceFaceTableNonManifold(mesh) };
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
        if (isNullFace(mesh, mesh.faces[fI])) {
            if (removeNullFaceByEdgeFlip(mesh, ffTable, fI)) {
                // meshes are small, OK to just recompute it all
                ffTable = geometry::getFaceFaceTableNonManifold(mesh);
                ++flipped;
            }
        }
    }
    LOG(info1) << "Flipped " << flipped
               << " edges while attempting to remove null faces.";
    return flipped > 0;
}

/// Remove faces that are adjacent to each other on all three sides
void removeSplicedFaces(Mesh& mesh)
{
    auto ffTable { geometry::getFaceFaceTableNonManifold(mesh) };
    std::vector<bool> removeFace(mesh.faces.size(), false);
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
        std::map<FIdx, std::size_t> neighCnt;
        for (const auto& n : ffTable[fI]) {
            if (neighCnt.count(n)) { neighCnt[n]++; }
            else { neighCnt[n] = 1; }
        }

        for (const auto& nc : neighCnt) {
            if (nc.second >= 3) {
                removeFace[fI] = true;
                removeFace[nc.first] = true;
            }
        }
    }

    Face::list newFaces;
    std::size_t count { 0 };
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
        if (!removeFace[fI]) { newFaces.push_back(mesh.faces[fI]); }
        else { ++count; }
    }
    mesh.faces = newFaces;
    LOG(info1) << "Removed " << count << " spliced faces";
}


Mesh removeUnusedVertices(Mesh& mesh)
{
    Mesh res;
    std::vector<int> old2new(mesh.vertices.size(), -1);
    auto getNewIdx = [&](VIdx oldIdx) {
        if (old2new[oldIdx] != -1) { return old2new[oldIdx]; }
        old2new[oldIdx] = res.vertices.size();
        res.vertices.push_back(mesh.vertices[oldIdx]);
        return old2new[oldIdx];
    };

    for (const auto& f : mesh.faces) {
        res.faces.emplace_back(getNewIdx(f.a),
                               getNewIdx(f.b),
                               getNewIdx(f.c),
                               f.imageId);
    }
    LOG(info1) << "Removed " << mesh.vertices.size() - res.vertices.size()
               << " unused vertices";

    return res;
}

} // namespace

/**
 * Repairs mesh produced by circular/rectangular roof mesh generators so that
 * the topology is correct
 *
 * - removes faces that are adjacent to each other from all three sides
 * - removes null faces (collinear vertices) by flipping their longest edge
 * - removes unused vertices
 *
 * @param[in,out] mesh
 */
void repairRoofMesh(geometry::Mesh& mesh)
{
    removeSplicedFaces(mesh);

    std::size_t iter { 0 };
    while (attemptToRemoveNullFacesByEdgeFlip(mesh)) {
        ++iter;
        if (iter > MaxEdgeflipIters) {
            LOG(warn4) << "Unable to remove null faces by edge flipping - "
                          "might cause problems later.";
        }
    }
    LOG(info1) << "Removed null faces in " << iter << " iterations.";

    mesh = removeUnusedVertices(mesh);
}

} // namespace lod2
} // namespace semantic
