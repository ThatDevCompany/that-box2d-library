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

import { B2_polygonRadius } from '../../Common/b2Settings'
import { B2Vec2, B2Rot, B2Transform } from '../../Common/b2Math'
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision'
import { B2DistanceProxy } from '../b2Distance'
import { B2MassData } from './b2Shape'
import { B2Shape, B2ShapeType } from './b2Shape'

///  A line segment (edge) shape. These can be connected in chains or loops
///  to other edge shapes. The connectivity information is used to ensure
///  correct contact normals.
export class B2EdgeShape extends B2Shape {
	public m_vertex1: B2Vec2 = new B2Vec2()
	public m_vertex2: B2Vec2 = new B2Vec2()
	public m_vertex0: B2Vec2 = new B2Vec2()
	public m_vertex3: B2Vec2 = new B2Vec2()
	public m_hasVertex0: boolean = false
	public m_hasVertex3: boolean = false

	constructor() {
		super(B2ShapeType.e_edgeShape, B2_polygonRadius)
	}

	///  Set this as an isolated edge.
	public Set(v1: B2Vec2, v2: B2Vec2): B2EdgeShape {
		this.m_vertex1.Copy(v1)
		this.m_vertex2.Copy(v2)
		this.m_hasVertex0 = false
		this.m_hasVertex3 = false
		return this
	}

	///  Implement B2Shape.
	public Clone(): B2EdgeShape {
		return new B2EdgeShape().Copy(this)
	}

	public Copy(other: B2EdgeShape): B2EdgeShape {
		super.Copy(other)

		/// b2Assert(other instanceof B2EdgeShape);

		this.m_vertex1.Copy(other.m_vertex1)
		this.m_vertex2.Copy(other.m_vertex2)
		this.m_vertex0.Copy(other.m_vertex0)
		this.m_vertex3.Copy(other.m_vertex3)
		this.m_hasVertex0 = other.m_hasVertex0
		this.m_hasVertex3 = other.m_hasVertex3

		return this
	}

	///  @see B2Shape::GetChildCount
	public GetChildCount(): number {
		return 1
	}

	///  @see B2Shape::TestPoint
	public TestPoint(xf: B2Transform, p: B2Vec2): boolean {
		return false
	}

	/// #if B2_ENABLE_PARTICLE
	///  @see B2Shape::ComputeDistance
	private static ComputeDistance_s_v1 = new B2Vec2()
	private static ComputeDistance_s_v2 = new B2Vec2()
	private static ComputeDistance_s_d = new B2Vec2()
	private static ComputeDistance_s_s = new B2Vec2()

	public ComputeDistance(
		xf: B2Transform,
		p: B2Vec2,
		normal: B2Vec2,
		childIndex: number
	): number {
		const v1 = B2Transform.MulXV(
			xf,
			this.m_vertex1,
			B2EdgeShape.ComputeDistance_s_v1
		)
		const v2 = B2Transform.MulXV(
			xf,
			this.m_vertex2,
			B2EdgeShape.ComputeDistance_s_v2
		)

		const d = B2Vec2.SubVV(p, v1, B2EdgeShape.ComputeDistance_s_d)
		const s = B2Vec2.SubVV(v2, v1, B2EdgeShape.ComputeDistance_s_s)
		const ds = B2Vec2.DotVV(d, s)
		if (ds > 0) {
			const s2 = B2Vec2.DotVV(s, s)
			if (ds > s2) {
				B2Vec2.SubVV(p, v2, d)
			} else {
				d.SelfMulSub(ds / s2, s)
			}
		}
		normal.Copy(d)
		return normal.Normalize()
	}

	/// #endif

	///  Implement B2Shape.
	// p = p1 + t * d
	// v = v1 + s * e
	// p1 + t * d = v1 + s * e
	// s * e - t * d = p1 - v1
	private static RayCast_s_p1 = new B2Vec2()
	private static RayCast_s_p2 = new B2Vec2()
	private static RayCast_s_d = new B2Vec2()
	private static RayCast_s_e = new B2Vec2()
	private static RayCast_s_q = new B2Vec2()
	private static RayCast_s_r = new B2Vec2()

	public RayCast(
		output: B2RayCastOutput,
		input: B2RayCastInput,
		xf: B2Transform,
		childIndex: number
	): boolean {
		// Put the ray into the edge's frame of reference.
		const p1: B2Vec2 = B2Transform.MulTXV(
			xf,
			input.p1,
			B2EdgeShape.RayCast_s_p1
		)
		const p2: B2Vec2 = B2Transform.MulTXV(
			xf,
			input.p2,
			B2EdgeShape.RayCast_s_p2
		)
		const d: B2Vec2 = B2Vec2.SubVV(p2, p1, B2EdgeShape.RayCast_s_d)

		const v1: B2Vec2 = this.m_vertex1
		const v2: B2Vec2 = this.m_vertex2
		const e: B2Vec2 = B2Vec2.SubVV(v2, v1, B2EdgeShape.RayCast_s_e)
		const normal: B2Vec2 = output.normal.Set(e.y, -e.x).SelfNormalize()

		// q = p1 + t * d
		// dot(normal, q - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		const numerator: number = B2Vec2.DotVV(
			normal,
			B2Vec2.SubVV(v1, p1, B2Vec2.s_t0)
		)
		const denominator: number = B2Vec2.DotVV(normal, d)

		if (denominator === 0) {
			return false
		}

		const t: number = numerator / denominator
		if (t < 0 || input.maxFraction < t) {
			return false
		}

		const q: B2Vec2 = B2Vec2.AddVMulSV(p1, t, d, B2EdgeShape.RayCast_s_q)

		// q = v1 + s * r
		// s = dot(q - v1, r) / dot(r, r)
		const r: B2Vec2 = B2Vec2.SubVV(v2, v1, B2EdgeShape.RayCast_s_r)
		const rr: number = B2Vec2.DotVV(r, r)
		if (rr === 0) {
			return false
		}

		const s: number = B2Vec2.DotVV(B2Vec2.SubVV(q, v1, B2Vec2.s_t0), r) / rr
		if (s < 0 || 1 < s) {
			return false
		}

		output.fraction = t
		B2Rot.MulRV(xf.q, output.normal, output.normal)
		if (numerator > 0) {
			output.normal.SelfNeg()
		}
		return true
	}

	///  @see B2Shape::ComputeAABB
	private static ComputeAABB_s_v1 = new B2Vec2()
	private static ComputeAABB_s_v2 = new B2Vec2()

	public ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void {
		const v1: B2Vec2 = B2Transform.MulXV(
			xf,
			this.m_vertex1,
			B2EdgeShape.ComputeAABB_s_v1
		)
		const v2: B2Vec2 = B2Transform.MulXV(
			xf,
			this.m_vertex2,
			B2EdgeShape.ComputeAABB_s_v2
		)

		B2Vec2.MinV(v1, v2, aabb.lowerBound)
		B2Vec2.MaxV(v1, v2, aabb.upperBound)

		const r: number = this.m_radius
		aabb.lowerBound.SelfSubXY(r, r)
		aabb.upperBound.SelfAddXY(r, r)
	}

	///  @see B2Shape::ComputeMass
	public ComputeMass(massData: B2MassData, density: number): void {
		massData.mass = 0
		B2Vec2.MidVV(this.m_vertex1, this.m_vertex2, massData.center)
		massData.I = 0
	}

	public SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void {
		proxy.m_vertices = proxy.m_buffer
		proxy.m_vertices[0].Copy(this.m_vertex1)
		proxy.m_vertices[1].Copy(this.m_vertex2)
		proxy.m_count = 2
		proxy.m_radius = this.m_radius
	}

	public ComputeSubmergedArea(
		normal: B2Vec2,
		offset: number,
		xf: B2Transform,
		c: B2Vec2
	): number {
		c.SetZero()
		return 0
	}

	public Dump(log: (format: string, ...args: any[]) => void): void {
		log('    const shape: B2EdgeShape = new B2EdgeShape();\n')
		log('    shape.m_radius = %.15f;\n', this.m_radius)
		log(
			'    shape.m_vertex0.Set(%.15f, %.15f);\n',
			this.m_vertex0.x,
			this.m_vertex0.y
		)
		log(
			'    shape.m_vertex1.Set(%.15f, %.15f);\n',
			this.m_vertex1.x,
			this.m_vertex1.y
		)
		log(
			'    shape.m_vertex2.Set(%.15f, %.15f);\n',
			this.m_vertex2.x,
			this.m_vertex2.y
		)
		log(
			'    shape.m_vertex3.Set(%.15f, %.15f);\n',
			this.m_vertex3.x,
			this.m_vertex3.y
		)
		log('    shape.m_hasVertex0 = %s;\n', this.m_hasVertex0)
		log('    shape.m_hasVertex3 = %s;\n', this.m_hasVertex3)
	}
}
