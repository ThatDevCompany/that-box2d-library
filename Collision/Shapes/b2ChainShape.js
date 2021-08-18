"use strict";
/*
 * Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2ChainShape = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Shape_1 = require("./b2Shape");
const b2EdgeShape_1 = require("./b2EdgeShape");
///  A chain shape is a free form sequence of line segments.
///  The chain has two-sided collision, so you can use inside and outside collision.
///  Therefore, you may use any winding order.
///  Since there may be many vertices, they are allocated using B2Alloc.
///  Connectivity information is used to create smooth collisions.
///  WARNING: The chain will not collide properly if there are self-intersections.
class B2ChainShape extends b2Shape_1.B2Shape {
    constructor() {
        super(3 /* e_chainShape */, b2Settings_1.B2_polygonRadius);
        this.m_count = 0;
        this.m_prevVertex = new b2Math_1.B2Vec2();
        this.m_nextVertex = new b2Math_1.B2Vec2();
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
    }
    ///  Create a loop. This automatically adjusts connectivity.
    ///  @param vertices an array of vertices, these are copied
    ///  @param count the vertex count
    CreateLoop(vertices, count = vertices.length) {
        /// b2Assert(this.m_vertices === null && this.m_count === 0);
        /// b2Assert(count >= 3);
        /// for (let i: number = 1; i < count; ++i) {
        ///   const v1 = vertices[i - 1];
        ///   const v2 = vertices[i];
        ///   // If the code crashes here, it means your vertices are too close together.
        ///   B2Assert(B2Vec2.DistanceSquaredVV(v1, v2) > B2_linearSlop * B2_linearSlop);
        /// }
        this.m_count = count + 1;
        this.m_vertices = b2Math_1.B2Vec2.MakeArray(this.m_count);
        for (let i = 0; i < count; ++i) {
            this.m_vertices[i].Copy(vertices[i]);
        }
        this.m_vertices[count].Copy(this.m_vertices[0]);
        this.m_prevVertex.Copy(this.m_vertices[this.m_count - 2]);
        this.m_nextVertex.Copy(this.m_vertices[1]);
        this.m_hasPrevVertex = true;
        this.m_hasNextVertex = true;
        return this;
    }
    ///  Create a chain with isolated end vertices.
    ///  @param vertices an array of vertices, these are copied
    ///  @param count the vertex count
    CreateChain(vertices, count = vertices.length) {
        /// b2Assert(this.m_vertices === null && this.m_count === 0);
        /// b2Assert(count >= 2);
        /// for (let i: number = 1; i < count; ++i) {
        ///   const v1 = vertices[i - 1];
        ///   const v2 = vertices[i];
        ///   // If the code crashes here, it means your vertices are too close together.
        ///   B2Assert(B2Vec2.DistanceSquaredVV(v1, v2) > B2_linearSlop * B2_linearSlop);
        /// }
        this.m_count = count;
        this.m_vertices = b2Math_1.B2Vec2.MakeArray(count);
        for (let i = 0; i < count; ++i) {
            this.m_vertices[i].Copy(vertices[i]);
        }
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
        this.m_prevVertex.SetZero();
        this.m_nextVertex.SetZero();
        return this;
    }
    ///  Establish connectivity to a vertex that precedes the first vertex.
    ///  Don't call this for loops.
    SetPrevVertex(prevVertex) {
        this.m_prevVertex.Copy(prevVertex);
        this.m_hasPrevVertex = true;
        return this;
    }
    ///  Establish connectivity to a vertex that follows the last vertex.
    ///  Don't call this for loops.
    SetNextVertex(nextVertex) {
        this.m_nextVertex.Copy(nextVertex);
        this.m_hasNextVertex = true;
        return this;
    }
    ///  Implement B2Shape. Vertices are cloned using B2Alloc.
    Clone() {
        return new B2ChainShape().Copy(this);
    }
    Copy(other) {
        super.Copy(other);
        /// b2Assert(other instanceof B2ChainShape);
        this.CreateChain(other.m_vertices, other.m_count);
        this.m_prevVertex.Copy(other.m_prevVertex);
        this.m_nextVertex.Copy(other.m_nextVertex);
        this.m_hasPrevVertex = other.m_hasPrevVertex;
        this.m_hasNextVertex = other.m_hasNextVertex;
        return this;
    }
    ///  @see B2Shape::GetChildCount
    GetChildCount() {
        // edge count = vertex count - 1
        return this.m_count - 1;
    }
    ///  Get a child edge.
    GetChildEdge(edge, index) {
        /// b2Assert(0 <= index && index < this.m_count - 1);
        edge.m_type = 1 /* e_edgeShape */;
        edge.m_radius = this.m_radius;
        edge.m_vertex1.Copy(this.m_vertices[index]);
        edge.m_vertex2.Copy(this.m_vertices[index + 1]);
        if (index > 0) {
            edge.m_vertex0.Copy(this.m_vertices[index - 1]);
            edge.m_hasVertex0 = true;
        }
        else {
            edge.m_vertex0.Copy(this.m_prevVertex);
            edge.m_hasVertex0 = this.m_hasPrevVertex;
        }
        if (index < this.m_count - 2) {
            edge.m_vertex3.Copy(this.m_vertices[index + 2]);
            edge.m_hasVertex3 = true;
        }
        else {
            edge.m_vertex3.Copy(this.m_nextVertex);
            edge.m_hasVertex3 = this.m_hasNextVertex;
        }
    }
    ///  This always return false.
    ///  @see B2Shape::TestPoint
    TestPoint(xf, p) {
        return false;
    }
    ComputeDistance(xf, p, normal, childIndex) {
        const edge = B2ChainShape.ComputeDistance_s_edgeShape;
        this.GetChildEdge(edge, childIndex);
        return edge.ComputeDistance(xf, p, normal, 0);
    }
    RayCast(output, input, xf, childIndex) {
        /// b2Assert(childIndex < this.m_count);
        const edgeShape = B2ChainShape.RayCast_s_edgeShape;
        edgeShape.m_vertex1.Copy(this.m_vertices[childIndex]);
        edgeShape.m_vertex2.Copy(this.m_vertices[(childIndex + 1) % this.m_count]);
        return edgeShape.RayCast(output, input, xf, 0);
    }
    ComputeAABB(aabb, xf, childIndex) {
        /// b2Assert(childIndex < this.m_count);
        const vertexi1 = this.m_vertices[childIndex];
        const vertexi2 = this.m_vertices[(childIndex + 1) % this.m_count];
        const v1 = b2Math_1.B2Transform.MulXV(xf, vertexi1, B2ChainShape.ComputeAABB_s_v1);
        const v2 = b2Math_1.B2Transform.MulXV(xf, vertexi2, B2ChainShape.ComputeAABB_s_v2);
        b2Math_1.B2Vec2.MinV(v1, v2, aabb.lowerBound);
        b2Math_1.B2Vec2.MaxV(v1, v2, aabb.upperBound);
    }
    ///  Chains have zero mass.
    ///  @see B2Shape::ComputeMass
    ComputeMass(massData, density) {
        massData.mass = 0;
        massData.center.SetZero();
        massData.I = 0;
    }
    SetupDistanceProxy(proxy, index) {
        /// b2Assert(0 <= index && index < this.m_count);
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_vertices[index]);
        if (index + 1 < this.m_count) {
            proxy.m_vertices[1].Copy(this.m_vertices[index + 1]);
        }
        else {
            proxy.m_vertices[1].Copy(this.m_vertices[0]);
        }
        proxy.m_count = 2;
        proxy.m_radius = this.m_radius;
    }
    ComputeSubmergedArea(normal, offset, xf, c) {
        c.SetZero();
        return 0;
    }
    Dump(log) {
        log('    const shape: B2ChainShape = new B2ChainShape();\n');
        log('    const vs: B2Vec2[] = B2Vec2.MakeArray(%d);\n', b2Settings_1.B2_maxPolygonVertices);
        for (let i = 0; i < this.m_count; ++i) {
            log('    vs[%d].Set(%.15f, %.15f);\n', i, this.m_vertices[i].x, this.m_vertices[i].y);
        }
        log('    shape.CreateChain(vs, %d);\n', this.m_count);
        log('    shape.m_prevVertex.Set(%.15f, %.15f);\n', this.m_prevVertex.x, this.m_prevVertex.y);
        log('    shape.m_nextVertex.Set(%.15f, %.15f);\n', this.m_nextVertex.x, this.m_nextVertex.y);
        log('    shape.m_hasPrevVertex = %s;\n', this.m_hasPrevVertex ? 'true' : 'false');
        log('    shape.m_hasNextVertex = %s;\n', this.m_hasNextVertex ? 'true' : 'false');
    }
}
exports.B2ChainShape = B2ChainShape;
/// #if B2_ENABLE_PARTICLE
///  @see B2Shape::ComputeDistance
B2ChainShape.ComputeDistance_s_edgeShape = new b2EdgeShape_1.B2EdgeShape();
/// #endif
///  Implement B2Shape.
B2ChainShape.RayCast_s_edgeShape = new b2EdgeShape_1.B2EdgeShape();
///  @see B2Shape::ComputeAABB
B2ChainShape.ComputeAABB_s_v1 = new b2Math_1.B2Vec2();
B2ChainShape.ComputeAABB_s_v2 = new b2Math_1.B2Vec2();
