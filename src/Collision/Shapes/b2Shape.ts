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

import {B2Vec2, B2Transform} from '../../Common/b2Math';
import {B2AABB, B2RayCastInput, B2RayCastOutput} from '../b2Collision';
import {B2DistanceProxy} from '../b2Distance';

///  This holds the mass data computed for a shape.
export class B2MassData {
	///  The mass of the shape, usually in kilograms.
	public mass: number = 0;

	///  The position of the shape's centroid relative to the shape's origin.
	public center: B2Vec2 = new B2Vec2(0, 0);

	///  The rotational inertia of the shape about the local origin.
	public I: number = 0;
}

export const enum B2ShapeType {
	e_unknown = -1,
	e_circleShape = 0,
	e_edgeShape = 1,
	e_polygonShape = 2,
	e_chainShape = 3,
	e_shapeTypeCount = 4
}

///  A shape is used for collision detection. You can create a shape however you like.
///  Shapes used for simulation in B2World are created automatically when a B2Fixture
///  is created. Shapes may encapsulate a one or more child shapes.
export abstract class B2Shape {
	public m_type: B2ShapeType = B2ShapeType.e_unknown;
	public m_radius: number = 0;

	constructor(type: B2ShapeType, radius: number) {
		this.m_type = type;
		this.m_radius = radius;
	}

	///  Clone the concrete shape using the provided allocator.
	public abstract Clone(): B2Shape;

	public Copy(other: B2Shape): B2Shape {
		/// b2Assert(this.m_type === other.m_type);
		this.m_radius = other.m_radius;
		return this;
	}

	///  Get the type of this shape. You can use this to down cast to the concrete shape.
	///  @return the shape type.
	public GetType(): B2ShapeType {
		return this.m_type;
	}

	///  Get the number of child primitives.
	public abstract GetChildCount(): number;

	///  Test a point for containment in this shape. This only works for convex shapes.
	///  @param xf the shape world transform.
	///  @param p a point in world coordinates.
	public abstract TestPoint(xf: B2Transform, p: B2Vec2): boolean;

	/// #if B2_ENABLE_PARTICLE
	///  Compute the distance from the current shape to the specified point. This only works for convex shapes.
	///  @param xf the shape world transform.
	///  @param p a point in world coordinates.
	///  @param distance returns the distance from the current shape.
	///  @param normal returns the direction in which the distance increases.
	public abstract ComputeDistance(xf: B2Transform, p: B2Vec2, normal: B2Vec2, childIndex: number): number;

	/// #endif

	///  Cast a ray against a child shape.
	///  @param output the ray-cast results.
	///  @param input the ray-cast input parameters.
	///  @param transform the transform to be applied to the shape.
	///  @param childIndex the child shape index
	public abstract RayCast(output: B2RayCastOutput, input: B2RayCastInput, transform: B2Transform, childIndex: number): boolean;

	///  Given a transform, compute the associated axis aligned bounding box for a child shape.
	///  @param aabb returns the axis aligned box.
	///  @param xf the world transform of the shape.
	///  @param childIndex the child shape
	public abstract ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void;

	///  Compute the mass properties of this shape using its dimensions and density.
	///  The inertia tensor is computed about the local origin.
	///  @param massData returns the mass data for this shape.
	///  @param density the density in kilograms per meter squared.
	public abstract ComputeMass(massData: B2MassData, density: number): void;

	public abstract SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void;

	public abstract ComputeSubmergedArea(normal: B2Vec2, offset: number, xf: B2Transform, c: B2Vec2): number;

	public abstract Dump(log: (format: string, ...args: any[]) => void): void;
}
