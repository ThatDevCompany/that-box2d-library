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

import { B2_pi, B2_epsilon, B2MakeArray } from './b2Settings'

export const B2_pi_over_180: number = B2_pi / 180
export const B2_180_over_pi: number = 180 / B2_pi
export const B2_two_pi: number = 2 * B2_pi

export function B2Abs(n: number): number {
	return n < 0 ? -n : n
}

export function B2Min(a: number, b: number): number {
	return a < b ? a : b
}

export function B2Max(a: number, b: number): number {
	return a > b ? a : b
}

export function B2Clamp(a: number, lo: number, hi: number): number {
	return a < lo ? lo : a > hi ? hi : a
}

export function B2Swap(a: any[], b: any[]): void {
	/// b2Assert(false);
	const tmp: any = a[0]
	a[0] = b[0]
	b[0] = tmp
}

///  This function is used to ensure that a floating point number is
///  not a NaN or infinity.
export function B2IsValid(n: number): boolean {
	return isFinite(n)
}

export function B2Sq(n: number): number {
	return n * n
}

///  This is a approximate yet fast inverse square-root.
export function B2InvSqrt(n: number): number {
	return 1 / Math.sqrt(n)
}

export function B2Sqrt(n: number): number {
	return Math.sqrt(n)
}

export function B2Pow(x: number, y: number): number {
	return Math.pow(x, y)
}

export function B2DegToRad(degrees: number): number {
	return degrees * B2_pi_over_180
}

export function B2RadToDeg(radians: number): number {
	return radians * B2_180_over_pi
}

export function B2Cos(radians: number): number {
	return Math.cos(radians)
}

export function B2Sin(radians: number): number {
	return Math.sin(radians)
}

export function B2Acos(n: number): number {
	return Math.acos(n)
}

export function B2Asin(n: number): number {
	return Math.asin(n)
}

export function B2Atan2(y: number, x: number): number {
	return Math.atan2(y, x)
}

export function B2NextPowerOfTwo(x: number): number {
	x |= (x >> 1) & 0x7fffffff
	x |= (x >> 2) & 0x3fffffff
	x |= (x >> 4) & 0x0fffffff
	x |= (x >> 8) & 0x00ffffff
	x |= (x >> 16) & 0x0000ffff
	return x + 1
}

export function B2IsPowerOfTwo(x: number): boolean {
	return x > 0 && (x & (x - 1)) === 0
}

export function B2Random(): number {
	return Math.random() * 2 - 1
}

export function B2RandomRange(lo: number, hi: number): number {
	return (hi - lo) * Math.random() + lo
}

///  A 2D column vector.
export class B2Vec2 {
	public static ZERO = new B2Vec2(0, 0)
	public static UNITX = new B2Vec2(1, 0)
	public static UNITY = new B2Vec2(0, 1)

	public static s_t0 = new B2Vec2()
	public static s_t1 = new B2Vec2()
	public static s_t2 = new B2Vec2()
	public static s_t3 = new B2Vec2()

	public x: number
	public y: number

	constructor(x: number = 0, y: number = 0) {
		this.x = x
		this.y = y
	}

	public Clone(): B2Vec2 {
		return new B2Vec2(this.x, this.y)
	}

	public SetZero(): B2Vec2 {
		this.x = 0
		this.y = 0
		return this
	}

	public Set(x: number, y: number): B2Vec2 {
		this.x = x
		this.y = y
		return this
	}

	public Copy(other: B2Vec2): B2Vec2 {
		/// b2Assert(this !== other);
		this.x = other.x
		this.y = other.y
		return this
	}

	public SelfAdd(v: B2Vec2): B2Vec2 {
		this.x += v.x
		this.y += v.y
		return this
	}

	public SelfAddXY(x: number, y: number): B2Vec2 {
		this.x += x
		this.y += y
		return this
	}

	public SelfSub(v: B2Vec2): B2Vec2 {
		this.x -= v.x
		this.y -= v.y
		return this
	}

	public SelfSubXY(x: number, y: number): B2Vec2 {
		this.x -= x
		this.y -= y
		return this
	}

	public SelfMul(s: number): B2Vec2 {
		this.x *= s
		this.y *= s
		return this
	}

	public SelfMulAdd(s: number, v: B2Vec2): B2Vec2 {
		this.x += s * v.x
		this.y += s * v.y
		return this
	}

	public SelfMulSub(s: number, v: B2Vec2): B2Vec2 {
		this.x -= s * v.x
		this.y -= s * v.y
		return this
	}

	public Dot(v: B2Vec2): number {
		return this.x * v.x + this.y * v.y
	}

	public Cross(v: B2Vec2): number {
		return this.x * v.y - this.y * v.x
	}

	public Length(): number {
		const x: number = this.x,
			y: number = this.y
		return Math.sqrt(x * x + y * y)
	}

	public LengthSquared(): number {
		const x: number = this.x,
			y: number = this.y
		return x * x + y * y
	}

	public Normalize(): number {
		const length: number = this.Length()
		if (length >= B2_epsilon) {
			const inv_length: number = 1 / length
			this.x *= inv_length
			this.y *= inv_length
		}
		return length
	}

	public SelfNormalize(): B2Vec2 {
		const length: number = this.Length()
		if (length >= B2_epsilon) {
			const inv_length: number = 1 / length
			this.x *= inv_length
			this.y *= inv_length
		}
		return this
	}

	public SelfRotate(radians: number): B2Vec2 {
		const c: number = Math.cos(radians)
		const s: number = Math.sin(radians)
		const x: number = this.x
		this.x = c * x - s * this.y
		this.y = s * x + c * this.y
		return this
	}

	public IsValid(): boolean {
		return isFinite(this.x) && isFinite(this.y)
	}

	public SelfCrossVS(s: number): B2Vec2 {
		const x: number = this.x
		this.x = s * this.y
		this.y = -s * x
		return this
	}

	public SelfCrossSV(s: number): B2Vec2 {
		const x: number = this.x
		this.x = -s * this.y
		this.y = s * x
		return this
	}

	public SelfMinV(v: B2Vec2): B2Vec2 {
		this.x = B2Min(this.x, v.x)
		this.y = B2Min(this.y, v.y)
		return this
	}

	public SelfMaxV(v: B2Vec2): B2Vec2 {
		this.x = B2Max(this.x, v.x)
		this.y = B2Max(this.y, v.y)
		return this
	}

	public SelfAbs(): B2Vec2 {
		this.x = B2Abs(this.x)
		this.y = B2Abs(this.y)
		return this
	}

	public SelfNeg(): B2Vec2 {
		this.x = -this.x
		this.y = -this.y
		return this
	}

	public SelfSkew(): B2Vec2 {
		const x: number = this.x
		this.x = -this.y
		this.y = x
		return this
	}

	public static MakeArray(length: number): B2Vec2[] {
		return B2MakeArray(length, function(i: number): B2Vec2 {
			return new B2Vec2()
		})
	}

	public static AbsV(v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2AbsV(v, out)
	}

	public static MinV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MinV(a, b, out)
	}

	public static MaxV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MaxV(a, b, out)
	}

	public static ClampV(v: B2Vec2, lo: B2Vec2, hi: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2ClampV(v, lo, hi, out)
	}

	public static RotateV(v: B2Vec2, radians: number, out: B2Vec2): B2Vec2 {
		return B2RotateV(v, radians, out)
	}

	public static DotVV(a: B2Vec2, b: B2Vec2): number {
		return B2DotVV(a, b)
	}

	public static CrossVV(a: B2Vec2, b: B2Vec2): number {
		return B2CrossVV(a, b)
	}

	public static CrossVS(v: B2Vec2, s: number, out: B2Vec2): B2Vec2 {
		return B2CrossVS(v, s, out)
	}

	public static CrossVOne(v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2CrossVOne(v, out)
	}

	public static CrossSV(s: number, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2CrossSV(s, v, out)
	}

	public static CrossOneV(v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2CrossOneV(v, out)
	}

	public static AddVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2AddVV(a, b, out)
	}

	public static SubVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2SubVV(a, b, out)
	}

	public static MulSV(s: number, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulSV(s, v, out)
	}

	public static MulVS(v: B2Vec2, s: number, out: B2Vec2): B2Vec2 {
		return B2MulVS(v, s, out)
	}

	public static AddVMulSV(
		a: B2Vec2,
		s: number,
		b: B2Vec2,
		out: B2Vec2
	): B2Vec2 {
		return B2AddVMulSV(a, s, b, out)
	}

	public static SubVMulSV(
		a: B2Vec2,
		s: number,
		b: B2Vec2,
		out: B2Vec2
	): B2Vec2 {
		return B2SubVMulSV(a, s, b, out)
	}

	public static AddVCrossSV(
		a: B2Vec2,
		s: number,
		v: B2Vec2,
		out: B2Vec2
	): B2Vec2 {
		return B2AddVCrossSV(a, s, v, out)
	}

	public static MidVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MidVV(a, b, out)
	}

	public static ExtVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2ExtVV(a, b, out)
	}

	public static IsEqualToV(a: B2Vec2, b: B2Vec2): boolean {
		return B2IsEqualToV(a, b)
	}

	public static DistanceVV(a: B2Vec2, b: B2Vec2): number {
		return B2DistanceVV(a, b)
	}

	public static DistanceSquaredVV(a: B2Vec2, b: B2Vec2): number {
		return B2DistanceSquaredVV(a, b)
	}

	public static NegV(v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2NegV(v, out)
	}
}

export const B2Vec2_zero: B2Vec2 = new B2Vec2(0, 0)

function B2AbsV(v: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = B2Abs(v.x)
	out.y = B2Abs(v.y)
	return out
}

function B2MinV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = B2Min(a.x, b.x)
	out.y = B2Min(a.y, b.y)
	return out
}

function B2MaxV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = B2Max(a.x, b.x)
	out.y = B2Max(a.y, b.y)
	return out
}

function B2ClampV(v: B2Vec2, lo: B2Vec2, hi: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = B2Clamp(v.x, lo.x, hi.x)
	out.y = B2Clamp(v.y, lo.y, hi.y)
	return out
}

function B2RotateV(v: B2Vec2, radians: number, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x,
		v_y: number = v.y
	const c: number = Math.cos(radians)
	const s: number = Math.sin(radians)
	out.x = c * v_x - s * v_y
	out.y = s * v_x + c * v_y
	return out
}

function B2DotVV(a: B2Vec2, b: B2Vec2): number {
	return a.x * b.x + a.y * b.y
}

function B2CrossVV(a: B2Vec2, b: B2Vec2): number {
	return a.x * b.y - a.y * b.x
}

function B2CrossVS(v: B2Vec2, s: number, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x
	out.x = s * v.y
	out.y = -s * v_x
	return out
}

function B2CrossVOne(v: B2Vec2, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x
	out.x = v.y
	out.y = -v_x
	return out
}

function B2CrossSV(s: number, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x
	out.x = -s * v.y
	out.y = s * v_x
	return out
}

function B2CrossOneV(v: B2Vec2, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x
	out.x = -v.y
	out.y = v_x
	return out
}

function B2AddVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = a.x + b.x
	out.y = a.y + b.y
	return out
}

function B2SubVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = a.x - b.x
	out.y = a.y - b.y
	return out
}

function B2MulSV(s: number, v: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = v.x * s
	out.y = v.y * s
	return out
}

function B2MulVS(v: B2Vec2, s: number, out: B2Vec2): B2Vec2 {
	out.x = v.x * s
	out.y = v.y * s
	return out
}

function B2AddVMulSV(a: B2Vec2, s: number, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = a.x + s * b.x
	out.y = a.y + s * b.y
	return out
}

function B2SubVMulSV(a: B2Vec2, s: number, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = a.x - s * b.x
	out.y = a.y - s * b.y
	return out
}

function B2AddVCrossSV(a: B2Vec2, s: number, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x
	out.x = a.x - s * v.y
	out.y = a.y + s * v_x
	return out
}

function B2MidVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = (a.x + b.x) * 0.5
	out.y = (a.y + b.y) * 0.5
	return out
}

function B2ExtVV(a: B2Vec2, b: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = (b.x - a.x) * 0.5
	out.y = (b.y - a.y) * 0.5
	return out
}

function B2IsEqualToV(a: B2Vec2, b: B2Vec2): boolean {
	return a.x === b.x && a.y === b.y
}

function B2DistanceVV(a: B2Vec2, b: B2Vec2): number {
	const c_x: number = a.x - b.x
	const c_y: number = a.y - b.y
	return Math.sqrt(c_x * c_x + c_y * c_y)
}

function B2DistanceSquaredVV(a: B2Vec2, b: B2Vec2): number {
	const c_x: number = a.x - b.x
	const c_y: number = a.y - b.y
	return c_x * c_x + c_y * c_y
}

function B2NegV(v: B2Vec2, out: B2Vec2): B2Vec2 {
	out.x = -v.x
	out.y = -v.y
	return out
}

///  A 2D column vector with 3 elements.
export class B2Vec3 {
	public static ZERO = new B2Vec3(0, 0, 0)

	public static s_t0 = new B2Vec3()

	public x: number
	public y: number
	public z: number

	constructor(x: number = 0, y: number = 0, z: number = 0) {
		this.x = x
		this.y = y
		this.z = z
	}

	public Clone(): B2Vec3 {
		return new B2Vec3(this.x, this.y, this.z)
	}

	public SetZero(): B2Vec3 {
		this.x = 0
		this.y = 0
		this.z = 0
		return this
	}

	public SetXYZ(x: number, y: number, z: number): B2Vec3 {
		this.x = x
		this.y = y
		this.z = z
		return this
	}

	public Copy(other: B2Vec3): B2Vec3 {
		/// b2Assert(this !== other);
		this.x = other.x
		this.y = other.y
		this.z = other.z
		return this
	}

	public SelfNeg(): B2Vec3 {
		this.x = -this.x
		this.y = -this.y
		this.z = -this.z
		return this
	}

	public SelfAdd(v: B2Vec3): B2Vec3 {
		this.x += v.x
		this.y += v.y
		this.z += v.z
		return this
	}

	public SelfAddXYZ(x: number, y: number, z: number): B2Vec3 {
		this.x += x
		this.y += y
		this.z += z
		return this
	}

	public SelfSub(v: B2Vec3): B2Vec3 {
		this.x -= v.x
		this.y -= v.y
		this.z -= v.z
		return this
	}

	public SelfSubXYZ(x: number, y: number, z: number): B2Vec3 {
		this.x -= x
		this.y -= y
		this.z -= z
		return this
	}

	public SelfMul(s: number): B2Vec3 {
		this.x *= s
		this.y *= s
		this.z *= s
		return this
	}

	public static DotV3V3(a: B2Vec3, b: B2Vec3): number {
		return B2DotV3V3(a, b)
	}

	public static CrossV3V3(a: B2Vec3, b: B2Vec3, out: B2Vec3): B2Vec3 {
		return B2CrossV3V3(a, b, out)
	}
}

function B2DotV3V3(a: B2Vec3, b: B2Vec3): number {
	return a.x * b.x + a.y * b.y + a.z * b.z
}

function B2CrossV3V3(a: B2Vec3, b: B2Vec3, out: B2Vec3): B2Vec3 {
	const a_x: number = a.x,
		a_y = a.y,
		a_z = a.z
	const b_x: number = b.x,
		b_y = b.y,
		b_z = b.z
	out.x = a_y * b_z - a_z * b_y
	out.y = a_z * b_x - a_x * b_z
	out.z = a_x * b_y - a_y * b_x
	return out
}

///  A 2-by-2 matrix. Stored in column-major order.
export class B2Mat22 {
	public static IDENTITY = new B2Mat22()

	public ex: B2Vec2 = new B2Vec2(1, 0)
	public ey: B2Vec2 = new B2Vec2(0, 1)

	public Clone(): B2Mat22 {
		return new B2Mat22().Copy(this)
	}

	public static FromVV(c1: B2Vec2, c2: B2Vec2): B2Mat22 {
		return new B2Mat22().SetVV(c1, c2)
	}

	public static FromSSSS(
		r1c1: number,
		r1c2: number,
		r2c1: number,
		r2c2: number
	): B2Mat22 {
		return new B2Mat22().SetSSSS(r1c1, r1c2, r2c1, r2c2)
	}

	public static FromAngle(radians: number): B2Mat22 {
		return new B2Mat22().SetAngle(radians)
	}

	public SetSSSS(
		r1c1: number,
		r1c2: number,
		r2c1: number,
		r2c2: number
	): B2Mat22 {
		this.ex.Set(r1c1, r2c1)
		this.ey.Set(r1c2, r2c2)
		return this
	}

	public SetVV(c1: B2Vec2, c2: B2Vec2): B2Mat22 {
		this.ex.Copy(c1)
		this.ey.Copy(c2)
		return this
	}

	public SetAngle(radians: number): B2Mat22 {
		const c: number = Math.cos(radians)
		const s: number = Math.sin(radians)
		this.ex.Set(c, s)
		this.ey.Set(-s, c)
		return this
	}

	public Copy(other: B2Mat22): B2Mat22 {
		/// b2Assert(this !== other);
		this.ex.Copy(other.ex)
		this.ey.Copy(other.ey)
		return this
	}

	public SetIdentity(): B2Mat22 {
		this.ex.Set(1, 0)
		this.ey.Set(0, 1)
		return this
	}

	public SetZero(): B2Mat22 {
		this.ex.SetZero()
		this.ey.SetZero()
		return this
	}

	public GetAngle(): number {
		return Math.atan2(this.ex.y, this.ex.x)
	}

	public GetInverse(out: B2Mat22): B2Mat22 {
		const a: number = this.ex.x
		const b: number = this.ey.x
		const c: number = this.ex.y
		const d: number = this.ey.y
		let det: number = a * d - b * c
		if (det !== 0) {
			det = 1 / det
		}
		out.ex.x = det * d
		out.ey.x = -det * b
		out.ex.y = -det * c
		out.ey.y = det * a
		return out
	}

	public Solve(b_x: number, b_y: number, out: B2Vec2): B2Vec2 {
		const a11: number = this.ex.x,
			a12 = this.ey.x
		const a21: number = this.ex.y,
			a22 = this.ey.y
		let det: number = a11 * a22 - a12 * a21
		if (det !== 0) {
			det = 1 / det
		}
		out.x = det * (a22 * b_x - a12 * b_y)
		out.y = det * (a11 * b_y - a21 * b_x)
		return out
	}

	public SelfAbs(): B2Mat22 {
		this.ex.SelfAbs()
		this.ey.SelfAbs()
		return this
	}

	public SelfInv(): B2Mat22 {
		return this.GetInverse(this)
	}

	public SelfAddM(M: B2Mat22): B2Mat22 {
		this.ex.SelfAdd(M.ex)
		this.ey.SelfAdd(M.ey)
		return this
	}

	public SelfSubM(M: B2Mat22): B2Mat22 {
		this.ex.SelfSub(M.ex)
		this.ey.SelfSub(M.ey)
		return this
	}

	public static AbsM(M: B2Mat22, out: B2Mat22): B2Mat22 {
		return B2AbsM(M, out)
	}

	public static MulMV(M: B2Mat22, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulMV(M, v, out)
	}

	public static MulTMV(M: B2Mat22, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulTMV(M, v, out)
	}

	public static AddMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
		return B2AddMM(A, B, out)
	}

	public static MulMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
		return B2MulMM(A, B, out)
	}

	public static MulTMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
		return B2MulTMM(A, B, out)
	}
}

function B2AbsM(M: B2Mat22, out: B2Mat22): B2Mat22 {
	const M_ex: B2Vec2 = M.ex,
		M_ey: B2Vec2 = M.ey
	out.ex.x = B2Abs(M_ex.x)
	out.ex.y = B2Abs(M_ex.y)
	out.ey.x = B2Abs(M_ey.x)
	out.ey.y = B2Abs(M_ey.y)
	return out
}

function B2MulMV(M: B2Mat22, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const M_ex: B2Vec2 = M.ex,
		M_ey: B2Vec2 = M.ey
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = M_ex.x * v_x + M_ey.x * v_y
	out.y = M_ex.y * v_x + M_ey.y * v_y
	return out
}

function B2MulTMV(M: B2Mat22, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const M_ex: B2Vec2 = M.ex,
		M_ey: B2Vec2 = M.ey
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = M_ex.x * v_x + M_ex.y * v_y
	out.y = M_ey.x * v_x + M_ey.y * v_y
	return out
}

function B2AddMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
	const A_ex: B2Vec2 = A.ex,
		A_ey: B2Vec2 = A.ey
	const B_ex: B2Vec2 = B.ex,
		B_ey: B2Vec2 = B.ey
	out.ex.x = A_ex.x + B_ex.x
	out.ex.y = A_ex.y + B_ex.y
	out.ey.x = A_ey.x + B_ey.x
	out.ey.y = A_ey.y + B_ey.y
	return out
}

function B2MulMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
	const A_ex_x: number = A.ex.x,
		A_ex_y: number = A.ex.y
	const A_ey_x: number = A.ey.x,
		A_ey_y: number = A.ey.y
	const B_ex_x: number = B.ex.x,
		B_ex_y: number = B.ex.y
	const B_ey_x: number = B.ey.x,
		B_ey_y: number = B.ey.y
	out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y
	out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y
	out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y
	out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y
	return out
}

function B2MulTMM(A: B2Mat22, B: B2Mat22, out: B2Mat22): B2Mat22 {
	const A_ex_x: number = A.ex.x,
		A_ex_y: number = A.ex.y
	const A_ey_x: number = A.ey.x,
		A_ey_y: number = A.ey.y
	const B_ex_x: number = B.ex.x,
		B_ex_y: number = B.ex.y
	const B_ey_x: number = B.ey.x,
		B_ey_y: number = B.ey.y
	out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y
	out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y
	out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y
	out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y
	return out
}

///  A 3-by-3 matrix. Stored in column-major order.
export class B2Mat33 {
	public static IDENTITY = new B2Mat33()

	public ex: B2Vec3 = new B2Vec3(1, 0, 0)
	public ey: B2Vec3 = new B2Vec3(0, 1, 0)
	public ez: B2Vec3 = new B2Vec3(0, 0, 1)

	public Clone(): B2Mat33 {
		return new B2Mat33().Copy(this)
	}

	public SetVVV(c1: B2Vec3, c2: B2Vec3, c3: B2Vec3): B2Mat33 {
		this.ex.Copy(c1)
		this.ey.Copy(c2)
		this.ez.Copy(c3)
		return this
	}

	public Copy(other: B2Mat33): B2Mat33 {
		/// b2Assert(this !== other);
		this.ex.Copy(other.ex)
		this.ey.Copy(other.ey)
		this.ez.Copy(other.ez)
		return this
	}

	public SetIdentity(): B2Mat33 {
		this.ex.SetXYZ(1, 0, 0)
		this.ey.SetXYZ(0, 1, 0)
		this.ez.SetXYZ(0, 0, 1)
		return this
	}

	public SetZero(): B2Mat33 {
		this.ex.SetZero()
		this.ey.SetZero()
		this.ez.SetZero()
		return this
	}

	public SelfAddM(M: B2Mat33): B2Mat33 {
		this.ex.SelfAdd(M.ex)
		this.ey.SelfAdd(M.ey)
		this.ez.SelfAdd(M.ez)
		return this
	}

	public Solve33(b_x: number, b_y: number, b_z: number, out: B2Vec3): B2Vec3 {
		const a11: number = this.ex.x,
			a21: number = this.ex.y,
			a31: number = this.ex.z
		const a12: number = this.ey.x,
			a22: number = this.ey.y,
			a32: number = this.ey.z
		const a13: number = this.ez.x,
			a23: number = this.ez.y,
			a33: number = this.ez.z
		let det: number =
			a11 * (a22 * a33 - a32 * a23) +
			a21 * (a32 * a13 - a12 * a33) +
			a31 * (a12 * a23 - a22 * a13)
		if (det !== 0) {
			det = 1 / det
		}
		out.x =
			det *
			(b_x * (a22 * a33 - a32 * a23) +
				b_y * (a32 * a13 - a12 * a33) +
				b_z * (a12 * a23 - a22 * a13))
		out.y =
			det *
			(a11 * (b_y * a33 - b_z * a23) +
				a21 * (b_z * a13 - b_x * a33) +
				a31 * (b_x * a23 - b_y * a13))
		out.z =
			det *
			(a11 * (a22 * b_z - a32 * b_y) +
				a21 * (a32 * b_x - a12 * b_z) +
				a31 * (a12 * b_y - a22 * b_x))
		return out
	}

	public Solve22(b_x: number, b_y: number, out: B2Vec2): B2Vec2 {
		const a11: number = this.ex.x,
			a12: number = this.ey.x
		const a21: number = this.ex.y,
			a22: number = this.ey.y
		let det: number = a11 * a22 - a12 * a21
		if (det !== 0) {
			det = 1 / det
		}
		out.x = det * (a22 * b_x - a12 * b_y)
		out.y = det * (a11 * b_y - a21 * b_x)
		return out
	}

	public GetInverse22(M: B2Mat33): void {
		const a: number = this.ex.x,
			b: number = this.ey.x,
			c: number = this.ex.y,
			d: number = this.ey.y
		let det: number = a * d - b * c
		if (det !== 0) {
			det = 1 / det
		}

		M.ex.x = det * d
		M.ey.x = -det * b
		M.ex.z = 0
		M.ex.y = -det * c
		M.ey.y = det * a
		M.ey.z = 0
		M.ez.x = 0
		M.ez.y = 0
		M.ez.z = 0
	}

	public GetSymInverse33(M: B2Mat33): void {
		let det: number = B2DotV3V3(
			this.ex,
			B2CrossV3V3(this.ey, this.ez, B2Vec3.s_t0)
		)
		if (det !== 0) {
			det = 1 / det
		}

		const a11: number = this.ex.x,
			a12: number = this.ey.x,
			a13: number = this.ez.x
		const a22: number = this.ey.y,
			a23: number = this.ez.y
		const a33: number = this.ez.z

		M.ex.x = det * (a22 * a33 - a23 * a23)
		M.ex.y = det * (a13 * a23 - a12 * a33)
		M.ex.z = det * (a12 * a23 - a13 * a22)

		M.ey.x = M.ex.y
		M.ey.y = det * (a11 * a33 - a13 * a13)
		M.ey.z = det * (a13 * a12 - a11 * a23)

		M.ez.x = M.ex.z
		M.ez.y = M.ey.z
		M.ez.z = det * (a11 * a22 - a12 * a12)
	}

	public static MulM33V3(A: B2Mat33, v: B2Vec3, out: B2Vec3): B2Vec3 {
		return B2MulM33V3(A, v, out)
	}

	public static MulM33XYZ(
		A: B2Mat33,
		x: number,
		y: number,
		z: number,
		out: B2Vec3
	): B2Vec3 {
		return B2MulM33XYZ(A, x, y, z, out)
	}

	public static MulM33V2(A: B2Mat33, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulM33V2(A, v, out)
	}

	public static MulM33XY(
		A: B2Mat33,
		x: number,
		y: number,
		out: B2Vec2
	): B2Vec2 {
		return B2MulM33XY(A, x, y, out)
	}
}

function B2MulM33V3(A: B2Mat33, v: B2Vec3, out: B2Vec3): B2Vec3 {
	const v_x: number = v.x,
		v_y: number = v.y,
		v_z: number = v.z
	out.x = A.ex.x * v_x + A.ey.x * v_y + A.ez.x * v_z
	out.y = A.ex.y * v_x + A.ey.y * v_y + A.ez.y * v_z
	out.z = A.ex.z * v_x + A.ey.z * v_y + A.ez.z * v_z
	return out
}

function B2MulM33XYZ(
	A: B2Mat33,
	x: number,
	y: number,
	z: number,
	out: B2Vec3
): B2Vec3 {
	out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z
	out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z
	out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z
	return out
}

function B2MulM33V2(A: B2Mat33, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = A.ex.x * v_x + A.ey.x * v_y
	out.y = A.ex.y * v_x + A.ey.y * v_y
	return out
}

function B2MulM33XY(A: B2Mat33, x: number, y: number, out: B2Vec2): B2Vec2 {
	out.x = A.ex.x * x + A.ey.x * y
	out.y = A.ex.y * x + A.ey.y * y
	return out
}

///  Rotation
export class B2Rot {
	public static IDENTITY = new B2Rot()

	public s: number = 0
	public c: number = 1

	constructor(angle: number = 0) {
		if (angle) {
			this.s = Math.sin(angle)
			this.c = Math.cos(angle)
		}
	}

	public Clone(): B2Rot {
		return new B2Rot().Copy(this)
	}

	public Copy(other: B2Rot): B2Rot {
		this.s = other.s
		this.c = other.c
		return this
	}

	public SetAngle(angle: number): B2Rot {
		this.s = Math.sin(angle)
		this.c = Math.cos(angle)
		return this
	}

	public SetIdentity(): B2Rot {
		this.s = 0
		this.c = 1
		return this
	}

	public GetAngle(): number {
		return Math.atan2(this.s, this.c)
	}

	public GetXAxis(out: B2Vec2): B2Vec2 {
		out.x = this.c
		out.y = this.s
		return out
	}

	public GetYAxis(out: B2Vec2): B2Vec2 {
		out.x = -this.s
		out.y = this.c
		return out
	}

	public static MulRR(q: B2Rot, r: B2Rot, out: B2Rot): B2Rot {
		return B2MulRR(q, r, out)
	}

	public static MulTRR(q: B2Rot, r: B2Rot, out: B2Rot): B2Rot {
		return B2MulTRR(q, r, out)
	}

	public static MulRV(q: B2Rot, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulRV(q, v, out)
	}

	public static MulTRV(q: B2Rot, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulTRV(q, v, out)
	}
}

function B2MulRR(q: B2Rot, r: B2Rot, out: B2Rot): B2Rot {
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	const q_c: number = q.c,
		q_s: number = q.s
	const r_c: number = r.c,
		r_s: number = r.s
	out.s = q_s * r_c + q_c * r_s
	out.c = q_c * r_c - q_s * r_s
	return out
}

function B2MulTRR(q: B2Rot, r: B2Rot, out: B2Rot): B2Rot {
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	const q_c: number = q.c,
		q_s: number = q.s
	const r_c: number = r.c,
		r_s: number = r.s
	out.s = q_c * r_s - q_s * r_c
	out.c = q_c * r_c + q_s * r_s
	return out
}

function B2MulRV(q: B2Rot, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const q_c: number = q.c,
		q_s: number = q.s
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = q_c * v_x - q_s * v_y
	out.y = q_s * v_x + q_c * v_y
	return out
}

function B2MulTRV(q: B2Rot, v: B2Vec2, out: B2Vec2): B2Vec2 {
	const q_c: number = q.c,
		q_s: number = q.s
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = q_c * v_x + q_s * v_y
	out.y = -q_s * v_x + q_c * v_y
	return out
}

///  A transform contains translation and rotation. It is used to represent
///  the position and orientation of rigid frames.
export class B2Transform {
	public static IDENTITY = new B2Transform()

	public p: B2Vec2 = new B2Vec2()
	public q: B2Rot = new B2Rot()

	public Clone(): B2Transform {
		return new B2Transform().Copy(this)
	}

	public Copy(other: B2Transform): B2Transform {
		/// b2Assert(this !== other);
		this.p.Copy(other.p)
		this.q.Copy(other.q)
		return this
	}

	public SetIdentity(): B2Transform {
		this.p.SetZero()
		this.q.SetIdentity()
		return this
	}

	public SetPositionRotation(position: B2Vec2, q: B2Rot): B2Transform {
		this.p.Copy(position)
		this.q.Copy(q)
		return this
	}

	public SetPositionAngle(pos: B2Vec2, a: number): B2Transform {
		this.p.Copy(pos)
		this.q.SetAngle(a)
		return this
	}

	public SetPosition(position: B2Vec2): B2Transform {
		this.p.Copy(position)
		return this
	}

	public SetPositionXY(x: number, y: number): B2Transform {
		this.p.Set(x, y)
		return this
	}

	public SetRotation(rotation: B2Rot): B2Transform {
		this.q.Copy(rotation)
		return this
	}

	public SetRotationAngle(radians: number): B2Transform {
		this.q.SetAngle(radians)
		return this
	}

	public GetPosition(): B2Vec2 {
		return this.p
	}

	public GetRotation(): B2Rot {
		return this.q
	}

	public GetRotationAngle(): number {
		return this.q.GetAngle()
	}

	public GetAngle(): number {
		return this.q.GetAngle()
	}

	public static MulXV(T: B2Transform, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulXV(T, v, out)
	}

	public static MulTXV(T: B2Transform, v: B2Vec2, out: B2Vec2): B2Vec2 {
		return B2MulTXV(T, v, out)
	}

	public static MulXX(
		A: B2Transform,
		B: B2Transform,
		out: B2Transform
	): B2Transform {
		return B2MulXX(A, B, out)
	}

	public static MulTXX(
		A: B2Transform,
		B: B2Transform,
		out: B2Transform
	): B2Transform {
		return B2MulTXX(A, B, out)
	}
}

function B2MulXV(T: B2Transform, v: B2Vec2, out: B2Vec2): B2Vec2 {
	//  float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	//  float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
	//
	//  return B2Vec2(x, y);
	const T_q_c: number = T.q.c,
		T_q_s: number = T.q.s
	const v_x: number = v.x,
		v_y: number = v.y
	out.x = T_q_c * v_x - T_q_s * v_y + T.p.x
	out.y = T_q_s * v_x + T_q_c * v_y + T.p.y
	return out
}

function B2MulTXV(T: B2Transform, v: B2Vec2, out: B2Vec2): B2Vec2 {
	//  float32 px = v.x - T.p.x;
	//  float32 py = v.y - T.p.y;
	//  float32 x = (T.q.c * px + T.q.s * py);
	//  float32 y = (-T.q.s * px + T.q.c * py);
	//
	//  return B2Vec2(x, y);
	const T_q_c: number = T.q.c,
		T_q_s: number = T.q.s
	const p_x: number = v.x - T.p.x
	const p_y: number = v.y - T.p.y
	out.x = T_q_c * p_x + T_q_s * p_y
	out.y = -T_q_s * p_x + T_q_c * p_y
	return out
}

function B2MulXX(
	A: B2Transform,
	B: B2Transform,
	out: B2Transform
): B2Transform {
	B2MulRR(A.q, B.q, out.q)
	B2AddVV(B2MulRV(A.q, B.p, out.p), A.p, out.p)
	return out
}

function B2MulTXX(
	A: B2Transform,
	B: B2Transform,
	out: B2Transform
): B2Transform {
	B2MulTRR(A.q, B.q, out.q)
	B2MulTRV(A.q, B2SubVV(B.p, A.p, out.p), out.p)
	return out
}

///  This describes the motion of a body/shape for TOI computation.
///  Shapes are defined with respect to the body origin, which may
///  no coincide with the center of mass. However, to support dynamics
///  we must interpolate the center of mass position.
export class B2Sweep {
	public localCenter: B2Vec2 = new B2Vec2()
	public c0: B2Vec2 = new B2Vec2()
	public c: B2Vec2 = new B2Vec2()
	public a0: number = 0
	public a: number = 0
	public alpha0: number = 0

	public Clone(): B2Sweep {
		return new B2Sweep().Copy(this)
	}

	public Copy(other: B2Sweep): B2Sweep {
		/// b2Assert(this !== other);
		this.localCenter.Copy(other.localCenter)
		this.c0.Copy(other.c0)
		this.c.Copy(other.c)
		this.a0 = other.a0
		this.a = other.a
		this.alpha0 = other.alpha0
		return this
	}

	public GetTransform(xf: B2Transform, beta: number): B2Transform {
		const one_minus_beta: number = 1 - beta
		xf.p.x = one_minus_beta * this.c0.x + beta * this.c.x
		xf.p.y = one_minus_beta * this.c0.y + beta * this.c.y
		const angle: number = one_minus_beta * this.a0 + beta * this.a
		xf.q.SetAngle(angle)

		xf.p.SelfSub(B2MulRV(xf.q, this.localCenter, B2Vec2.s_t0))
		return xf
	}

	public Advance(alpha: number): void {
		/// b2Assert(this.alpha0 < 1);
		const beta: number = (alpha - this.alpha0) / (1 - this.alpha0)
		const one_minus_beta: number = 1 - beta
		this.c0.x = one_minus_beta * this.c0.x + beta * this.c.x
		this.c0.y = one_minus_beta * this.c0.y + beta * this.c.y
		this.a0 = one_minus_beta * this.a0 + beta * this.a
		this.alpha0 = alpha
	}

	public Normalize(): void {
		const d: number = B2_two_pi * Math.floor(this.a0 / B2_two_pi)
		this.a0 -= d
		this.a -= d
	}
}
