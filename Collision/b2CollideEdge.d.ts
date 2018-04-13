import { B2Transform } from '../Common/b2Math';
import { B2Manifold } from './b2Collision';
import { B2CircleShape } from './Shapes/b2CircleShape';
import { B2PolygonShape } from './Shapes/b2PolygonShape';
import { B2EdgeShape } from './Shapes/b2EdgeShape';
export declare function B2CollideEdgeAndCircle(manifold: B2Manifold, edgeA: B2EdgeShape, xfA: B2Transform, circleB: B2CircleShape, xfB: B2Transform): void;
export declare function B2CollideEdgeAndPolygon(manifold: B2Manifold, edgeA: B2EdgeShape, xfA: B2Transform, polygonB: B2PolygonShape, xfB: B2Transform): void;
