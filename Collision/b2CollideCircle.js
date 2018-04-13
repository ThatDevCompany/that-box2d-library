import { B2_maxFloat, B2_epsilon } from '../Common/b2Settings';
import { B2Vec2, B2Transform } from '../Common/b2Math';
const B2CollideCircles_s_pA = new B2Vec2();
const B2CollideCircles_s_pB = new B2Vec2();
export function B2CollideCircles(manifold, circleA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    const pA = B2Transform.MulXV(xfA, circleA.m_p, B2CollideCircles_s_pA);
    const pB = B2Transform.MulXV(xfB, circleB.m_p, B2CollideCircles_s_pB);
    const distSqr = B2Vec2.DistanceSquaredVV(pA, pB);
    const radius = circleA.m_radius + circleB.m_radius;
    if (distSqr > radius * radius) {
        return;
    }
    manifold.type = 0 /* e_circles */;
    manifold.localPoint.Copy(circleA.m_p);
    manifold.localNormal.SetZero();
    manifold.pointCount = 1;
    manifold.points[0].localPoint.Copy(circleB.m_p);
    manifold.points[0].id.key = 0;
}
const B2CollidePolygonAndCircle_s_c = new B2Vec2();
const B2CollidePolygonAndCircle_s_cLocal = new B2Vec2();
const B2CollidePolygonAndCircle_s_faceCenter = new B2Vec2();
export function B2CollidePolygonAndCircle(manifold, polygonA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle position in the frame of the polygon.
    const c = B2Transform.MulXV(xfB, circleB.m_p, B2CollidePolygonAndCircle_s_c);
    const cLocal = B2Transform.MulTXV(xfA, c, B2CollidePolygonAndCircle_s_cLocal);
    // Find the min separating edge.
    let normalIndex = 0;
    let separation = (-B2_maxFloat);
    const radius = polygonA.m_radius + circleB.m_radius;
    const vertexCount = polygonA.m_count;
    const vertices = polygonA.m_vertices;
    const normals = polygonA.m_normals;
    for (let i = 0; i < vertexCount; ++i) {
        const s = B2Vec2.DotVV(normals[i], B2Vec2.SubVV(cLocal, vertices[i], B2Vec2.s_t0));
        if (s > radius) {
            // Early out.
            return;
        }
        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }
    // Vertices that subtend the incident face.
    const vertIndex1 = normalIndex;
    const vertIndex2 = (vertIndex1 + 1) % vertexCount;
    const v1 = vertices[vertIndex1];
    const v2 = vertices[vertIndex2];
    // If the center is inside the polygon ...
    if (separation < B2_epsilon) {
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        manifold.localNormal.Copy(normals[normalIndex]);
        B2Vec2.MidVV(v1, v2, manifold.localPoint);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
        return;
    }
    // Compute barycentric coordinates
    const u1 = B2Vec2.DotVV(B2Vec2.SubVV(cLocal, v1, B2Vec2.s_t0), B2Vec2.SubVV(v2, v1, B2Vec2.s_t1));
    const u2 = B2Vec2.DotVV(B2Vec2.SubVV(cLocal, v2, B2Vec2.s_t0), B2Vec2.SubVV(v1, v2, B2Vec2.s_t1));
    if (u1 <= 0) {
        if (B2Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        B2Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v1);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else if (u2 <= 0) {
        if (B2Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        B2Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v2);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else {
        const faceCenter = B2Vec2.MidVV(v1, v2, B2CollidePolygonAndCircle_s_faceCenter);
        separation = B2Vec2.DotVV(B2Vec2.SubVV(cLocal, faceCenter, B2Vec2.s_t1), normals[vertIndex1]);
        if (separation > radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = 1 /* e_faceA */;
        manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize();
        manifold.localPoint.Copy(faceCenter);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
}
