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

import { B2_pi, B2_epsilon } from '../../Common/b2Settings'
import {
	B2Sq,
	B2Sqrt,
	B2Asin,
	B2Pow,
	B2Vec2,
	B2Transform
} from '../../Common/b2Math'
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision'
import { B2DistanceProxy } from '../b2Distance'
import { B2MassData } from './b2Shape'
import { B2Shape, B2ShapeType } from './b2Shape'

///  A circle shape.
export class B2CircleShape extends B2Shape {
	public m_p: B2Vec2 = new B2Vec2()

	constructor(radius: number = 0) {
		super(B2ShapeType.e_circleShape, radius)
	}

	///  Implement B2Shape.
	public Clone(): B2CircleShape {
		return new B2CircleShape().Copy(this)
	}

	public Copy(other: B2CircleShape): B2CircleShape {
		super.Copy(other)

		/// b2Assert(other instanceof B2CircleShape);

		this.m_p.Copy(other.m_p)
		return this
	}

	///  @see B2Shape::GetChildCount
	public GetChildCount(): number {
		return 1
	}

	///  Implement B2Shape.
	private static TestPoint_s_center = new B2Vec2()
	private static TestPoint_s_d = new B2Vec2()

	public TestPoint(transform: B2Transform, p: B2Vec2): boolean {
		const center: B2Vec2 = B2Transform.MulXV(
			transform,
			this.m_p,
			B2CircleShape.TestPoint_s_center
		)
		const d: B2Vec2 = B2Vec2.SubVV(p, center, B2CircleShape.TestPoint_s_d)
		return B2Vec2.DotVV(d, d) <= B2Sq(this.m_radius)
	}

	/// #if B2_ENABLE_PARTICLE
	///  @see B2Shape::ComputeDistance
	private static ComputeDistance_s_center = new B2Vec2()

	public ComputeDistance(
		xf: B2Transform,
		p: B2Vec2,
		normal: B2Vec2,
		childIndex: number
	): number {
		const center = B2Transform.MulXV(
			xf,
			this.m_p,
			B2CircleShape.ComputeDistance_s_center
		)
		B2Vec2.SubVV(p, center, normal)
		return normal.Normalize() - this.m_radius
	}

	/// #endif

	///  Implement B2Shape.
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	private static RayCast_s_position = new B2Vec2()
	private static RayCast_s_s = new B2Vec2()
	private static RayCast_s_r = new B2Vec2()

	public RayCast(
		output: B2RayCastOutput,
		input: B2RayCastInput,
		transform: B2Transform,
		childIndex: number
	): boolean {
		const position: B2Vec2 = B2Transform.MulXV(
			transform,
			this.m_p,
			B2CircleShape.RayCast_s_position
		)
		const s: B2Vec2 = B2Vec2.SubVV(
			input.p1,
			position,
			B2CircleShape.RayCast_s_s
		)
		const b: number = B2Vec2.DotVV(s, s) - B2Sq(this.m_radius)

		// Solve quadratic equation.
		const r: B2Vec2 = B2Vec2.SubVV(
			input.p2,
			input.p1,
			B2CircleShape.RayCast_s_r
		)
		const c: number = B2Vec2.DotVV(s, r)
		const rr: number = B2Vec2.DotVV(r, r)
		const sigma = c * c - rr * b

		// Check for negative discriminant and short segment.
		if (sigma < 0 || rr < B2_epsilon) {
			return false
		}

		// Find the point of intersection of the line with the circle.
		let a: number = -(c + B2Sqrt(sigma))

		// Is the intersection point on the segment?
		if (0 <= a && a <= input.maxFraction * rr) {
			a /= rr
			output.fraction = a
			B2Vec2.AddVMulSV(s, a, r, output.normal).SelfNormalize()
			return true
		}

		return false
	}

	///  @see B2Shape::ComputeAABB
	private static ComputeAABB_s_p = new B2Vec2()

	public ComputeAABB(
		aabb: B2AABB,
		transform: B2Transform,
		childIndex: number
	): void {
		const p: B2Vec2 = B2Transform.MulXV(
			transform,
			this.m_p,
			B2CircleShape.ComputeAABB_s_p
		)
		aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius)
		aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius)
	}

	///  @see B2Shape::ComputeMass
	public ComputeMass(massData: B2MassData, density: number): void {
		const radius_sq: number = B2Sq(this.m_radius)
		massData.mass = density * B2_pi * radius_sq
		massData.center.Copy(this.m_p)

		// inertia about the local origin
		massData.I =
			massData.mass * (0.5 * radius_sq + B2Vec2.DotVV(this.m_p, this.m_p))
	}

	public SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void {
		proxy.m_vertices = proxy.m_buffer
		proxy.m_vertices[0].Copy(this.m_p)
		proxy.m_count = 1
		proxy.m_radius = this.m_radius
	}

	public ComputeSubmergedArea(
		normal: B2Vec2,
		offset: number,
		xf: B2Transform,
		c: B2Vec2
	): number {
		const p: B2Vec2 = B2Transform.MulXV(xf, this.m_p, new B2Vec2())
		const l: number = -(B2Vec2.DotVV(normal, p) - offset)

		if (l < -this.m_radius + B2_epsilon) {
			// Completely dry
			return 0
		}
		if (l > this.m_radius) {
			// Completely wet
			c.Copy(p)
			return B2_pi * this.m_radius * this.m_radius
		}

		// Magic
		const r2: number = this.m_radius * this.m_radius
		const l2: number = l * l
		const area: number =
			r2 * (B2Asin(l / this.m_radius) + B2_pi / 2) + l * B2Sqrt(r2 - l2)
		const com: number = ((-2 / 3) * B2Pow(r2 - l2, 1.5)) / area

		c.x = p.x + normal.x * com
		c.y = p.y + normal.y * com

		return area
	}

	public Dump(log: (format: string, ...args: any[]) => void): void {
		log('    const shape: B2CircleShape = new B2CircleShape();\n')
		log('    shape.m_radius = %.15f;\n', this.m_radius)
		log('    shape.m_p.Set(%.15f, %.15f);\n', this.m_p.x, this.m_p.y)
	}
}
