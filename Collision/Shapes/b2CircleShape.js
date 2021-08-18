"use strict";
/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
exports.B2CircleShape = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Shape_1 = require("./b2Shape");
///  A circle shape.
class B2CircleShape extends b2Shape_1.B2Shape {
    constructor(radius = 0) {
        super(0 /* e_circleShape */, radius);
        this.m_p = new b2Math_1.B2Vec2();
    }
    ///  Implement B2Shape.
    Clone() {
        return new B2CircleShape().Copy(this);
    }
    Copy(other) {
        super.Copy(other);
        /// b2Assert(other instanceof B2CircleShape);
        this.m_p.Copy(other.m_p);
        return this;
    }
    ///  @see B2Shape::GetChildCount
    GetChildCount() {
        return 1;
    }
    TestPoint(transform, p) {
        const center = b2Math_1.B2Transform.MulXV(transform, this.m_p, B2CircleShape.TestPoint_s_center);
        const d = b2Math_1.B2Vec2.SubVV(p, center, B2CircleShape.TestPoint_s_d);
        return b2Math_1.B2Vec2.DotVV(d, d) <= b2Math_1.B2Sq(this.m_radius);
    }
    ComputeDistance(xf, p, normal, childIndex) {
        const center = b2Math_1.B2Transform.MulXV(xf, this.m_p, B2CircleShape.ComputeDistance_s_center);
        b2Math_1.B2Vec2.SubVV(p, center, normal);
        return normal.Normalize() - this.m_radius;
    }
    RayCast(output, input, transform, childIndex) {
        const position = b2Math_1.B2Transform.MulXV(transform, this.m_p, B2CircleShape.RayCast_s_position);
        const s = b2Math_1.B2Vec2.SubVV(input.p1, position, B2CircleShape.RayCast_s_s);
        const b = b2Math_1.B2Vec2.DotVV(s, s) - b2Math_1.B2Sq(this.m_radius);
        // Solve quadratic equation.
        const r = b2Math_1.B2Vec2.SubVV(input.p2, input.p1, B2CircleShape.RayCast_s_r);
        const c = b2Math_1.B2Vec2.DotVV(s, r);
        const rr = b2Math_1.B2Vec2.DotVV(r, r);
        const sigma = c * c - rr * b;
        // Check for negative discriminant and short segment.
        if (sigma < 0 || rr < b2Settings_1.B2_epsilon) {
            return false;
        }
        // Find the point of intersection of the line with the circle.
        let a = -(c + b2Math_1.B2Sqrt(sigma));
        // Is the intersection point on the segment?
        if (0 <= a && a <= input.maxFraction * rr) {
            a /= rr;
            output.fraction = a;
            b2Math_1.B2Vec2.AddVMulSV(s, a, r, output.normal).SelfNormalize();
            return true;
        }
        return false;
    }
    ComputeAABB(aabb, transform, childIndex) {
        const p = b2Math_1.B2Transform.MulXV(transform, this.m_p, B2CircleShape.ComputeAABB_s_p);
        aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius);
        aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius);
    }
    ///  @see B2Shape::ComputeMass
    ComputeMass(massData, density) {
        const radius_sq = b2Math_1.B2Sq(this.m_radius);
        massData.mass = density * b2Settings_1.B2_pi * radius_sq;
        massData.center.Copy(this.m_p);
        // inertia about the local origin
        massData.I =
            massData.mass * (0.5 * radius_sq + b2Math_1.B2Vec2.DotVV(this.m_p, this.m_p));
    }
    SetupDistanceProxy(proxy, index) {
        proxy.m_vertices = proxy.m_buffer;
        proxy.m_vertices[0].Copy(this.m_p);
        proxy.m_count = 1;
        proxy.m_radius = this.m_radius;
    }
    ComputeSubmergedArea(normal, offset, xf, c) {
        const p = b2Math_1.B2Transform.MulXV(xf, this.m_p, new b2Math_1.B2Vec2());
        const l = -(b2Math_1.B2Vec2.DotVV(normal, p) - offset);
        if (l < -this.m_radius + b2Settings_1.B2_epsilon) {
            // Completely dry
            return 0;
        }
        if (l > this.m_radius) {
            // Completely wet
            c.Copy(p);
            return b2Settings_1.B2_pi * this.m_radius * this.m_radius;
        }
        // Magic
        const r2 = this.m_radius * this.m_radius;
        const l2 = l * l;
        const area = r2 * (b2Math_1.B2Asin(l / this.m_radius) + b2Settings_1.B2_pi / 2) + l * b2Math_1.B2Sqrt(r2 - l2);
        const com = ((-2 / 3) * b2Math_1.B2Pow(r2 - l2, 1.5)) / area;
        c.x = p.x + normal.x * com;
        c.y = p.y + normal.y * com;
        return area;
    }
    Dump(log) {
        log('    const shape: B2CircleShape = new B2CircleShape();\n');
        log('    shape.m_radius = %.15f;\n', this.m_radius);
        log('    shape.m_p.Set(%.15f, %.15f);\n', this.m_p.x, this.m_p.y);
    }
}
exports.B2CircleShape = B2CircleShape;
///  Implement B2Shape.
B2CircleShape.TestPoint_s_center = new b2Math_1.B2Vec2();
B2CircleShape.TestPoint_s_d = new b2Math_1.B2Vec2();
/// #if B2_ENABLE_PARTICLE
///  @see B2Shape::ComputeDistance
B2CircleShape.ComputeDistance_s_center = new b2Math_1.B2Vec2();
/// #endif
///  Implement B2Shape.
// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
B2CircleShape.RayCast_s_position = new b2Math_1.B2Vec2();
B2CircleShape.RayCast_s_s = new b2Math_1.B2Vec2();
B2CircleShape.RayCast_s_r = new b2Math_1.B2Vec2();
///  @see B2Shape::ComputeAABB
B2CircleShape.ComputeAABB_s_p = new b2Math_1.B2Vec2();
