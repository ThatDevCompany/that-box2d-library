"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2CollidePolygons = void 0;
const b2Settings_1 = require("../Common/b2Settings");
const b2Math_1 = require("../Common/b2Math");
const b2Collision_1 = require("./b2Collision");
const B2EdgeSeparation_s_normal1World = new b2Math_1.B2Vec2();
const B2EdgeSeparation_s_normal1 = new b2Math_1.B2Vec2();
const B2EdgeSeparation_s_v1 = new b2Math_1.B2Vec2();
const B2EdgeSeparation_s_v2 = new b2Math_1.B2Vec2();
function B2EdgeSeparation(poly1, xf1, edge1, poly2, xf2) {
    /// const count1: number = poly1.m_count;
    const vertices1 = poly1.m_vertices;
    const normals1 = poly1.m_normals;
    const count2 = poly2.m_count;
    const vertices2 = poly2.m_vertices;
    /// b2Assert(0 <= edge1 && edge1 < count1);
    // Convert normal from poly1's frame into poly2's frame.
    const normal1World = b2Math_1.B2Rot.MulRV(xf1.q, normals1[edge1], B2EdgeSeparation_s_normal1World);
    const normal1 = b2Math_1.B2Rot.MulTRV(xf2.q, normal1World, B2EdgeSeparation_s_normal1);
    // Find support vertex on poly2 for -normal.
    let index = 0;
    let minDot = b2Settings_1.B2_maxFloat;
    for (let i = 0; i < count2; ++i) {
        const dot = b2Math_1.B2Vec2.DotVV(vertices2[i], normal1);
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }
    const v1 = b2Math_1.B2Transform.MulXV(xf1, vertices1[edge1], B2EdgeSeparation_s_v1);
    const v2 = b2Math_1.B2Transform.MulXV(xf2, vertices2[index], B2EdgeSeparation_s_v2);
    const separation = b2Math_1.B2Vec2.DotVV(b2Math_1.B2Vec2.SubVV(v2, v1, b2Math_1.B2Vec2.s_t0), normal1World);
    return separation;
}
const B2FindMaxSeparation_s_d = new b2Math_1.B2Vec2();
const B2FindMaxSeparation_s_dLocal1 = new b2Math_1.B2Vec2();
function B2FindMaxSeparation(edgeIndex, poly1, xf1, poly2, xf2) {
    const count1 = poly1.m_count;
    const normals1 = poly1.m_normals;
    // Vector pointing from the centroid of poly1 to the centroid of poly2.
    const d = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Transform.MulXV(xf2, poly2.m_centroid, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Transform.MulXV(xf1, poly1.m_centroid, b2Math_1.B2Vec2.s_t1), B2FindMaxSeparation_s_d);
    const dLocal1 = b2Math_1.B2Rot.MulTRV(xf1.q, d, B2FindMaxSeparation_s_dLocal1);
    // Find edge normal on poly1 that has the largest projection onto d.
    let edge = 0;
    let maxDot = -b2Settings_1.B2_maxFloat;
    for (let i = 0; i < count1; ++i) {
        const dot = b2Math_1.B2Vec2.DotVV(normals1[i], dLocal1);
        if (dot > maxDot) {
            maxDot = dot;
            edge = i;
        }
    }
    // Get the separation for the edge normal.
    let s = B2EdgeSeparation(poly1, xf1, edge, poly2, xf2);
    // Check the separation for the previous edge normal.
    const prevEdge = (edge + count1 - 1) % count1;
    const sPrev = B2EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
    // Check the separation for the next edge normal.
    const nextEdge = (edge + 1) % count1;
    const sNext = B2EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
    // Find the best edge and the search direction.
    let bestEdge = 0;
    let bestSeparation = 0;
    let increment = 0;
    if (sPrev > s && sPrev > sNext) {
        increment = -1;
        bestEdge = prevEdge;
        bestSeparation = sPrev;
    }
    else if (sNext > s) {
        increment = 1;
        bestEdge = nextEdge;
        bestSeparation = sNext;
    }
    else {
        edgeIndex[0] = edge;
        return s;
    }
    // Perform a local search for the best edge normal.
    while (true) {
        if (increment === -1) {
            edge = (bestEdge + count1 - 1) % count1;
        }
        else {
            edge = (bestEdge + 1) % count1;
        }
        s = B2EdgeSeparation(poly1, xf1, edge, poly2, xf2);
        if (s > bestSeparation) {
            bestEdge = edge;
            bestSeparation = s;
        }
        else {
            break;
        }
    }
    edgeIndex[0] = bestEdge;
    return bestSeparation;
}
const B2FindIncidentEdge_s_normal1 = new b2Math_1.B2Vec2();
function B2FindIncidentEdge(c, poly1, xf1, edge1, poly2, xf2) {
    /// const count1: number = poly1.m_count;
    const normals1 = poly1.m_normals;
    const count2 = poly2.m_count;
    const vertices2 = poly2.m_vertices;
    const normals2 = poly2.m_normals;
    /// b2Assert(0 <= edge1 && edge1 < count1);
    // Get the normal of the reference edge in poly2's frame.
    const normal1 = b2Math_1.B2Rot.MulTRV(xf2.q, b2Math_1.B2Rot.MulRV(xf1.q, normals1[edge1], b2Math_1.B2Vec2.s_t0), B2FindIncidentEdge_s_normal1);
    // Find the incident edge on poly2.
    let index = 0;
    let minDot = b2Settings_1.B2_maxFloat;
    for (let i = 0; i < count2; ++i) {
        const dot = b2Math_1.B2Vec2.DotVV(normal1, normals2[i]);
        if (dot < minDot) {
            minDot = dot;
            index = i;
        }
    }
    // Build the clip vertices for the incident edge.
    const i1 = index;
    const i2 = (i1 + 1) % count2;
    const c0 = c[0];
    b2Math_1.B2Transform.MulXV(xf2, vertices2[i1], c0.v);
    const cf0 = c0.id.cf;
    cf0.indexA = edge1;
    cf0.indexB = i1;
    cf0.typeA = 1 /* e_face */;
    cf0.typeB = 0 /* e_vertex */;
    const c1 = c[1];
    b2Math_1.B2Transform.MulXV(xf2, vertices2[i2], c1.v);
    const cf1 = c1.id.cf;
    cf1.indexA = edge1;
    cf1.indexB = i2;
    cf1.typeA = 1 /* e_face */;
    cf1.typeB = 0 /* e_vertex */;
}
const B2CollidePolygons_s_incidentEdge = b2Collision_1.B2ClipVertex.MakeArray(2);
const B2CollidePolygons_s_clipPoints1 = b2Collision_1.B2ClipVertex.MakeArray(2);
const B2CollidePolygons_s_clipPoints2 = b2Collision_1.B2ClipVertex.MakeArray(2);
const B2CollidePolygons_s_edgeA = [0];
const B2CollidePolygons_s_edgeB = [0];
const B2CollidePolygons_s_localTangent = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_localNormal = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_planePoint = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_normal = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_tangent = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_ntangent = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_v11 = new b2Math_1.B2Vec2();
const B2CollidePolygons_s_v12 = new b2Math_1.B2Vec2();
function B2CollidePolygons(manifold, polyA, xfA, polyB, xfB) {
    manifold.pointCount = 0;
    const totalRadius = polyA.m_radius + polyB.m_radius;
    const edgeA = B2CollidePolygons_s_edgeA;
    edgeA[0] = 0;
    const separationA = B2FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
    if (separationA > totalRadius) {
        return;
    }
    const edgeB = B2CollidePolygons_s_edgeB;
    edgeB[0] = 0;
    const separationB = B2FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
    if (separationB > totalRadius) {
        return;
    }
    let poly1; // reference polygon
    let poly2; // incident polygon
    let xf1, xf2;
    let edge1 = 0; // reference edge
    let flip = 0;
    const k_relativeTol = 0.98;
    const k_absoluteTol = 0.001;
    if (separationB > k_relativeTol * separationA + k_absoluteTol) {
        poly1 = polyB;
        poly2 = polyA;
        xf1 = xfB;
        xf2 = xfA;
        edge1 = edgeB[0];
        manifold.type = 2 /* e_faceB */;
        flip = 1;
    }
    else {
        poly1 = polyA;
        poly2 = polyB;
        xf1 = xfA;
        xf2 = xfB;
        edge1 = edgeA[0];
        manifold.type = 1 /* e_faceA */;
        flip = 0;
    }
    const incidentEdge = B2CollidePolygons_s_incidentEdge;
    B2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
    const count1 = poly1.m_count;
    const vertices1 = poly1.m_vertices;
    const iv1 = edge1;
    const iv2 = (edge1 + 1) % count1;
    const local_v11 = vertices1[iv1];
    const local_v12 = vertices1[iv2];
    const localTangent = b2Math_1.B2Vec2.SubVV(local_v12, local_v11, B2CollidePolygons_s_localTangent);
    localTangent.Normalize();
    const localNormal = b2Math_1.B2Vec2.CrossVOne(localTangent, B2CollidePolygons_s_localNormal);
    const planePoint = b2Math_1.B2Vec2.MidVV(local_v11, local_v12, B2CollidePolygons_s_planePoint);
    const tangent = b2Math_1.B2Rot.MulRV(xf1.q, localTangent, B2CollidePolygons_s_tangent);
    const normal = b2Math_1.B2Vec2.CrossVOne(tangent, B2CollidePolygons_s_normal);
    const v11 = b2Math_1.B2Transform.MulXV(xf1, local_v11, B2CollidePolygons_s_v11);
    const v12 = b2Math_1.B2Transform.MulXV(xf1, local_v12, B2CollidePolygons_s_v12);
    // Face offset.
    const frontOffset = b2Math_1.B2Vec2.DotVV(normal, v11);
    // Side offsets, extended by polytope skin thickness.
    const sideOffset1 = -b2Math_1.B2Vec2.DotVV(tangent, v11) + totalRadius;
    const sideOffset2 = b2Math_1.B2Vec2.DotVV(tangent, v12) + totalRadius;
    // Clip incident edge against extruded edge1 side edges.
    const clipPoints1 = B2CollidePolygons_s_clipPoints1;
    const clipPoints2 = B2CollidePolygons_s_clipPoints2;
    let np;
    // Clip to box side 1
    const ntangent = b2Math_1.B2Vec2.NegV(tangent, B2CollidePolygons_s_ntangent);
    np = b2Collision_1.B2ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1);
    if (np < 2) {
        return;
    }
    // Clip to negative box side 1
    np = b2Collision_1.B2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);
    if (np < 2) {
        return;
    }
    // Now clipPoints2 contains the clipped points.
    manifold.localNormal.Copy(localNormal);
    manifold.localPoint.Copy(planePoint);
    let pointCount = 0;
    for (let i = 0; i < b2Settings_1.B2_maxManifoldPoints; ++i) {
        const cv = clipPoints2[i];
        const separation = b2Math_1.B2Vec2.DotVV(normal, cv.v) - frontOffset;
        if (separation <= totalRadius) {
            const cp = manifold.points[pointCount];
            b2Math_1.B2Transform.MulTXV(xf2, cv.v, cp.localPoint);
            cp.id.Copy(cv.id);
            if (flip) {
                // Swap features
                const cf = cp.id.cf;
                cp.id.cf.indexA = cf.indexB;
                cp.id.cf.indexB = cf.indexA;
                cp.id.cf.typeA = cf.typeB;
                cp.id.cf.typeB = cf.typeA;
            }
            ++pointCount;
        }
    }
    manifold.pointCount = pointCount;
}
exports.B2CollidePolygons = B2CollidePolygons;
