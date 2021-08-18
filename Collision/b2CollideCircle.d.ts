import { B2Transform } from '../Common/b2Math';
import { B2Manifold } from './b2Collision';
import { B2CircleShape } from './Shapes/b2CircleShape';
import { B2PolygonShape } from './Shapes/b2PolygonShape';
export declare function B2CollideCircles(manifold: B2Manifold, circleA: B2CircleShape, xfA: B2Transform, circleB: B2CircleShape, xfB: B2Transform): void;
export declare function B2CollidePolygonAndCircle(manifold: B2Manifold, polygonA: B2PolygonShape, xfA: B2Transform, circleB: B2CircleShape, xfB: B2Transform): void;
